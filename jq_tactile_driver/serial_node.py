#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

import serial


FRAME_HEADER = bytes([0xAA, 0x55, 0x03, 0x99])

# From spec: Sensor Type mapping (0x01..0x06)
SENSOR_TYPE_MAP = {
    0x01: "LH",
    0x02: "RH",
    0x03: "LF",
    0x04: "RF",
    0x05: "FB",
    0x06: "WB",
}

# From spec: payload lengths depend on packet sequence
PAYLOAD_LEN_SEQ1 = 128
PAYLOAD_LEN_SEQ2 = 144  # last 128 sensor + 16 gyro


@dataclass
class PartialFrame:
    """Holds partially assembled frame (seq1 + seq2) for one sensor type."""
    sensor_first128: Optional[bytes] = None
    sensor_last128: Optional[bytes] = None
    gyro16: Optional[bytes] = None
    t_first: float = 0.0


class JQTactileSerialNode(Node):
    def __init__(self) -> None:
        super().__init__("jq_tactile_serial_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("read_timeout_s", 0.02)
        self.declare_parameter("frame_timeout_s", 0.25)  # drop partial if seq2 never arrives
        self.declare_parameter("base_topic", "/jq_tactile")

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = int(self.get_parameter("baud").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.frame_timeout_s = float(self.get_parameter("frame_timeout_s").value)
        self.base_topic = self.get_parameter("base_topic").get_parameter_value().string_value

        # Per-sensor publishers created lazily
        self.tactile_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.gyro_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.info_pubs: Dict[str, rclpy.publisher.Publisher] = {}

        self.partial: Dict[int, PartialFrame] = {}  # key = sensor_type byte

        self.resync_count = 0
        self.dropped_partial = 0
        self.packets_ok = 0

        self._open_serial()

        # Read loop timer (fast); each tick reads whatever is available
        self.timer = self.create_timer(0.001, self._spin_once)

        self.get_logger().info(
            f"jq_tactile_driver reading {self.port} @ {self.baud} baud"
        )

    def _open_serial(self) -> None:
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=self.read_timeout_s,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            # Clear buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

    def _ensure_publishers(self, sensor_name: str) -> None:
        if sensor_name in self.tactile_pubs:
            return

        tactile_topic = f"{self.base_topic}/{sensor_name}/tactile_bytes"
        gyro_topic = f"{self.base_topic}/{sensor_name}/gyro_bytes"
        info_topic = f"{self.base_topic}/{sensor_name}/frame_info"

        self.tactile_pubs[sensor_name] = self.create_publisher(UInt8MultiArray, tactile_topic, 10)
        self.gyro_pubs[sensor_name] = self.create_publisher(UInt8MultiArray, gyro_topic, 10)
        self.info_pubs[sensor_name] = self.create_publisher(String, info_topic, 10)

        self.get_logger().info(f"Publishing {sensor_name} to {tactile_topic} and {gyro_topic}")

    def _publish_frame(self, sensor_type: int, sensor256: bytes, gyro16: bytes) -> None:
        sensor_name = SENSOR_TYPE_MAP.get(sensor_type, f"UNKNOWN_{sensor_type:02X}")
        self._ensure_publishers(sensor_name)

        tactile_msg = UInt8MultiArray()
        tactile_msg.data = list(sensor256)

        gyro_msg = UInt8MultiArray()
        gyro_msg.data = list(gyro16)

        self.tactile_pubs[sensor_name].publish(tactile_msg)
        self.gyro_pubs[sensor_name].publish(gyro_msg)

        info = String()
        info.data = f"packets_ok={self.packets_ok} resync={self.resync_count} dropped_partial={self.dropped_partial}"
        self.info_pubs[sensor_name].publish(info)

    def _read_exactly(self, n: int) -> Optional[bytes]:
        """Read exactly n bytes or return None if timeout/insufficient."""
        data = bytearray()
        while len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def _find_header(self) -> bool:
        """
        Synchronize to FRAME_HEADER in a stream. Returns True if header found,
        otherwise False (need more data).
        """
        # Keep a small rolling window
        window = bytearray()
        while True:
            b = self.ser.read(1)
            if not b:
                return False
            window += b
            if len(window) > len(FRAME_HEADER):
                window = window[-len(FRAME_HEADER):]
            if bytes(window) == FRAME_HEADER:
                return True

    def _spin_once(self) -> None:
        # Drop stale partial frames
        now = time.time()
        for stype, pf in list(self.partial.items()):
            if pf.t_first > 0.0 and (now - pf.t_first) > self.frame_timeout_s:
                self.partial.pop(stype, None)
                self.dropped_partial += 1

        try:
            # Attempt to find header
            if not self._find_header():
                return  # no more data currently

            # Read sequence + sensor type
            meta = self._read_exactly(2)
            if meta is None:
                self.resync_count += 1
                return

            seq = meta[0]
            sensor_type = meta[1]

            # Determine payload length by seq (per spec)
            if seq == 0x01:
                payload_len = PAYLOAD_LEN_SEQ1
            elif seq == 0x02:
                payload_len = PAYLOAD_LEN_SEQ2
            else:
                # Unknown; resync
                self.resync_count += 1
                return

            payload = self._read_exactly(payload_len)
            if payload is None:
                self.resync_count += 1
                return

            # Assemble frame
            pf = self.partial.get(sensor_type, PartialFrame())
            if seq == 0x01:
                pf.sensor_first128 = payload
                pf.t_first = time.time()
                self.partial[sensor_type] = pf
            else:
                # seq==0x02: payload is last128 + gyro16
                pf.sensor_last128 = payload[:128]
                pf.gyro16 = payload[128:144]
                if pf.sensor_first128 is not None:
                    sensor256 = pf.sensor_first128 + pf.sensor_last128
                    self.packets_ok += 1
                    self._publish_frame(sensor_type, sensor256, pf.gyro16)
                # Clear partial regardless (avoid mixing across frames)
                self.partial.pop(sensor_type, None)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}. Attempting reopen...")
            try:
                self.ser.close()
            except Exception:
                pass
            time.sleep(0.2)
            self._open_serial()
        except Exception as e:
            self.get_logger().error(f"Unexpected error in read loop: {e}")


def main() -> None:
    rclpy.init()
    node = JQTactileSerialNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
