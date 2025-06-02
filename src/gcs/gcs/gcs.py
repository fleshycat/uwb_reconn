import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from uwb_msgs.msg import Ranging
from px4_msgs.msg import TrajectorySetpoint as TrajectorySetpointMsg

import struct
import serial
import threading
import time
import math

# ANSI escape codes for terminal clear
_CLEAR_SCREEN = "\x1b[2J\x1b[H"

class ParseState:
    WAIT_SYNC1      = 0
    WAIT_SYNC2      = 1
    WAIT_LENGTH     = 2
    WAIT_HEADER     = 3
    WAIT_DATA       = 4
    WAIT_CHECKSUM_1 = 5
    WAIT_CHECKSUM_2 = 6

class JFiProtocol:
    HEADER_SIZE   = 5    # SYNC1, SYNC2, LENGTH, SEQ, SID
    CHECKSUM_SIZE = 2    # 16-bit XOR checksum

    def __init__(self):
        self.state          = ParseState.WAIT_SYNC1
        self.buffer         = bytearray()
        self.checksum       = 0
        self.payload_length = None

    @staticmethod
    def create_packet(payload: bytes, seq: int = 0, sid: int = 1) -> bytes:
        """
        Build a J-Fi packet: [SYNC1][SYNC2][LENGTH][SEQ][SID][PAYLOAD...][CHECKSUM_L][CHECKSUM_H]
        """
        sync1  = 0xAA
        sync2  = 0x55
        length = JFiProtocol.HEADER_SIZE + len(payload) + JFiProtocol.CHECKSUM_SIZE

        # Pack header: SYNC1, SYNC2, LENGTH, SEQ, SID
        header = struct.pack('BBBBB', sync1, sync2, length, seq, sid)
        packet = header + payload

        # Compute and append checksum
        checksum = JFiProtocol.compute_checksum(packet)
        packet += struct.pack('<H', checksum)
        return packet

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """
        16-bit XOR over all bytes
        """
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum

    def reset_parser(self) -> None:
        self.state          = ParseState.WAIT_SYNC1
        self.buffer         = bytearray()
        self.checksum       = 0
        self.payload_length = None

    def validate_checksum(self) -> bool:
        """
        XOR over everything except the last two bytes, compare to received checksum
        """
        if len(self.buffer) < JFiProtocol.CHECKSUM_SIZE:
            return False
        computed = 0
        for b in self.buffer[:-JFiProtocol.CHECKSUM_SIZE]:
            computed ^= b
        return computed == self.checksum

    def parse(self, byte: int) -> bytes | None:
        """
        Feed one byte into the state machine.
        Returns the complete packet bytes (including header+payload+checksum) once valid.
        """
        if self.state == ParseState.WAIT_SYNC1:
            if byte == 0xAA:
                self.reset_parser()
                self.buffer.append(byte)
                self.state = ParseState.WAIT_SYNC2

        elif self.state == ParseState.WAIT_SYNC2:
            if byte == 0x55:
                self.buffer.append(byte)
                self.state = ParseState.WAIT_LENGTH
            else:
                self.reset_parser()

        elif self.state == ParseState.WAIT_LENGTH:
            self.buffer.append(byte)
            self.payload_length = byte
            self.state = ParseState.WAIT_HEADER

        elif self.state == ParseState.WAIT_HEADER:
            self.buffer.append(byte)
            if len(self.buffer) == JFiProtocol.HEADER_SIZE:
                self.state = ParseState.WAIT_DATA

        elif self.state == ParseState.WAIT_DATA:
            self.buffer.append(byte)
            data_len = self.payload_length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
            if len(self.buffer) == JFiProtocol.HEADER_SIZE + data_len:
                self.state = ParseState.WAIT_CHECKSUM_1

        elif self.state == ParseState.WAIT_CHECKSUM_1:
            self.buffer.append(byte)
            self.checksum = byte
            self.state = ParseState.WAIT_CHECKSUM_2

        elif self.state == ParseState.WAIT_CHECKSUM_2:
            self.buffer.append(byte)
            self.checksum |= (byte << 8)
            if self.validate_checksum():
                packet = bytes(self.buffer)
            else:
                packet = None
            self.reset_parser()
            return packet

        return None

class GCS(Node):
    """
    A minimal GCS-side node that opens the J-Fi serial port,
    parses incoming packets (either UWB or target), and republishes them as ROS2 topics.
    """
    def __init__(self):
        super().__init__('gcs')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('system_id', 0)  # GCS ID if needed (default 0)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value

        port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate  = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # J-Fi parser
        self.parser = JFiProtocol()

        # Set up serial
        try:
            self.serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f'Opened serial port: {port_name} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port_name}: {e}')
            raise

        # --- “각 드론(SID 1~4)의 최신 UWB/Target 데이터를 저장할 딕셔너리” ---
        #   - self.uwb_data[system_id] = (range, rss, err, posx, posy, posz)
        #   - self.target_data[system_id] = (lat, lon, alt)
        self.uwb_data    = {i: None for i in range(1,5)}
        self.target_data = {i: None for i in range(1,5)}

        # Start receive thread
        self.receive_thread = threading.Thread(target=self._serial_receive_loop, daemon=True)
        self.receive_thread.start()

        # --- 화면 갱신 타이머 (1Hz) ---
        self.refresh_timer = self.create_timer(1.0, self._print_table)

    def _serial_receive_loop(self):
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for b in data:
                        packet = self.parser.parse(b)
                        if packet is not None:
                            self._handle_packet(packet)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break

    def _handle_packet(self, packet: bytes):
        header = packet[:JFiProtocol.HEADER_SIZE]
        sync1, sync2, length, seq_jfi, sid = struct.unpack('BBBBB', header)

        payload_len = length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
        payload = packet[JFiProtocol.HEADER_SIZE : JFiProtocol.HEADER_SIZE + payload_len]

        # --- UWB 데이터 (페이로드 길이 62) ---
        # format: '<B i B d d d d d d f f'
        if payload_len == 62:
            anchor_id, range, seq, \
            posx, posy, posz, \
            orix, oriy, oriz, \
            rss, err = struct.unpack('<B i B d d d d d d f f', payload)

            # sid(=드론 ID)에 해당하는 최신 UWB 정보 업데이트
            # 저장 형식: (range, rss, error_est, posx, posy, posz)
            self.uwb_data[sid] = (range, rss, err, posx, posy, posz)

        # --- Target 데이터 (페이로드 길이 24) ---
        # format: '<d d d'
        elif payload_len == 24:
            lat, lon, alt = struct.unpack('<d d d', payload)
            # sid(=드론 ID)에 해당하는 최신 Target 정보 업데이트
            self.target_data[sid] = (lat, lon, alt)

        else:
            self.get_logger().warn(f'Unknown payload length: {payload_len} bytes (SID={sid})')
    
    def _print_table(self):
        """
        1초마다 터미널을 클리어한 뒤, 4대 드론의 최신 UWB/Target 데이터를 표 형태로 출력합니다.
        """
        # 터미널 클리어
        print(_CLEAR_SCREEN, end='')

        # 컬럼 헤더 출력
        hdr = (
            f"{'DRONE':>5} | "
            f"{'RANGE(mm)':>9} | {'RSS':>5} | {'ERR':>5} | "
            f"{'POS_X':>9} | {'POS_Y':>9} | {'POS_Z':>9} || "
            f"{'TGT_X':>11} | {'TGT_Y':>11} | {'TGT_Z':>9}"
        )
        print(hdr)
        print("-" * len(hdr))

        # 1~4번 드론 순서대로 한 줄씩
        for system_id in range(1, 5):
            uwb = self.uwb_data[system_id]
            tgt = self.target_data[system_id]

            if uwb is None:
                # 아직 수신된 UWB 정보가 없으면 빈 칸으로 출력
                rng_str = rss_str = err_str = posx_str = posy_str = posz_str = "   -    "
            else:
                rng_mm, rss, err, posx, posy, posz = uwb
                rng_str  = f"{rng_mm:9d}"
                rss_str  = f"{rss:5.1f}"
                err_str  = f"{err:5.1f}"
                posx_str = f"{posx:9.3f}"
                posy_str = f"{posy:9.3f}"
                posz_str = f"{posz:9.3f}"

            if tgt is None:
                tgt_x_str = tgt_y_str = tgt_z_str = "     -     "
            else:
                lat, lon, alt = tgt
                tgt_x_str = f"{lat:11.7f}"
                tgt_y_str = f"{lon:11.7f}"
                tgt_z_str = f"{alt:9.3f}"

            line = (
                f"{system_id:>5d} | "
                f"{rng_str} | {rss_str} | {err_str} | "
                f"{posx_str} | {posy_str} | {posz_str} || "
                f"{tgt_x_str} | {tgt_y_str} | {tgt_z_str}"
            )
            print(line)

        print("\n(최신 데이터가 없으면 ‘–’로 표시됩니다.)\n")

def main(args=None):
    rclpy.init(args=args)
    node = GCS()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
