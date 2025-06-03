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
        if len(self.buffer) < JFiProtocol.CHECKSUM_SIZE:
            return False
        computed = 0
        for b in self.buffer[:-JFiProtocol.CHECKSUM_SIZE]:
            computed ^= b
        return computed == self.checksum

    def parse(self, byte: int) -> bytes | None:
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
    A GCS-side node that parses incoming packets (either UWB or target) and displays
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

        # --- 드론별 시퀀스/카운터 초기화 (SID 1~4) ---
        self.last_seq    = {i: None for i in range(1,5)}  # 직전 seq 저장
        self.recv_count  = {i: 0    for i in range(1,5)}  # 누적 수신된 패킷 수
        self.loss_count  = {i: 0    for i in range(1,5)}  # 누적 손실된 패킷 수

        # --- 각 드론(SID 1~4)의 최신 UWB/Target 데이터를 저장할 딕셔너리 ---
        #    self.uwb_data[sid]    = (range, rss, err, posx, posy, posz)
        #    self.target_data[sid] = (lat, lon, alt)
        self.uwb_data    = {i: None for i in range(1,5)}
        self.target_data = {i: None for i in range(1,5)}

        # Start receive thread
        self.receive_thread = threading.Thread(target=self._serial_receive_loop, daemon=True)
        self.receive_thread.start()

        # --- 화면 갱신 타이머 (25Hz) ---
        self.refresh_timer = self.create_timer(0.04, self._print_table)

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
        sync1, sync2, length, seq, sid = struct.unpack('BBBBB', header)

        # 손실 패킷 계산
        prev_seq = self.last_seq[sid]
        if prev_seq is None:
            # 첫 수신: 손실 없음, 수신 카운트만 1
            self.recv_count[sid] = 1
            self.loss_count[sid] = 0
        else:
            expected = (prev_seq + 1) & 0xFF
            if seq != expected:
                # 손실 개수 diff 계산
                diff = (seq - expected) & 0xFF
                self.loss_count[sid] += diff
            self.recv_count[sid] += 1

        # 현재 seq를 저장
        self.last_seq[sid] = seq

        payload_len = length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
        payload = packet[JFiProtocol.HEADER_SIZE : JFiProtocol.HEADER_SIZE + payload_len]

        # --- UWB 데이터 (페이로드 길이 62) ---
        if payload_len == 62:
            anchor_id, range, seq_range, \
            posx, posy, posz, \
            orix, oriy, oriz, \
            rss, err = struct.unpack('<B i B d d d d d d f f', payload)

            # sid(=드론 ID)에 해당하는 최신 UWB 정보 업데이트
            self.uwb_data[sid] = (range, rss, err, posx, posy, posz)

        # --- Target 데이터 (페이로드 길이 24) ---
        elif payload_len == 24:
            lat, lon, alt = struct.unpack('<d d d', payload)
            # sid(=드론 ID)에 해당하는 최신 Target 정보 업데이트
            self.target_data[sid] = (lat, lon, alt)

        else:
            self.get_logger().warn(f'Unknown payload length: {payload_len} bytes (SID={sid})')

    def _print_table(self):
        """
        25초마다 터미널을 클리어한 뒤, 4대 드론의 최신 UWB/Target 데이터를 표 형태로 출력합니다.
        """
        # 터미널 클리어
        print(_CLEAR_SCREEN, end='')

        # 컬럼 헤더: DRONE | RCVD | LOSS | LOSS(%) | … UWB/Target …
        hdr = (
            f"{'DRONE':>5s} | "
            f"{'RCVD':>5s} | {'LOSS':>5s} | {'LOSS(%)':>7s} | "
            f"{'RANGE(mm)':>9s} | {'RSS':>5s} | {'ERR':>5s} | "
            f"{'POS_X':>9s} | {'POS_Y':>9s} | {'POS_Z':>9s} || "
            f"{'TGT_X':>11s} | {'TGT_Y':>11s} | {'TGT_Z':>9s}"
        )
        print(hdr)
        print("-" * len(hdr))

        # 1~4번 드론 순서대로 한 줄씩
        for sid in range(1, 5):
            # 1) RCVD / LOSS / LOSS(%) 계산
            recv = self.recv_count[sid]
            loss = self.loss_count[sid]
            total = recv + loss
            if total > 0:
                loss_pct = loss / total * 100.0
            else:
                loss_pct = 0.0
            recv_str = f"{recv:5d}"
            loss_str = f"{loss:5d}"
            pct_str  = f"{loss_pct:7.1f}"

            # 2) UWB 데이터 문자열
            uwb = self.uwb_data[sid]
            if uwb is None:
                rng_str  =   "   -    "
                rss_str  =   "   - "
                err_str  =   "   - "
                posx_str =   "   -    "
                posy_str =   "   -    "
                posz_str =   "   -    "
            else:
                rng_mm, rss, err_val, posx, posy, posz = uwb
                rng_str  = f"{rng_mm:9d}"
                rss_str  = f"{rss:5.1f}"
                err_str  = f"{err_val:5.1f}"
                posx_str = f"{posx:9.3f}"
                posy_str = f"{posy:9.3f}"
                posz_str = f"{posz:9.3f}"

            # 3) Target 데이터 문자열
            tgt = self.target_data[sid]
            if tgt is None:
                tgt_x_str =   "     -     "
                tgt_y_str =   "     -     "
                tgt_z_str =   "   -    "
            else:
                lat, lon, alt = tgt
                tgt_x_str = f"{lat:11.7f}"
                tgt_y_str = f"{lon:11.7f}"
                tgt_z_str = f"{alt:9.3f}"

            line = (
                f"{sid:>5d} | "
                f"{recv_str} | {loss_str} | {pct_str} | "
                f"{rng_str} | {rss_str} | {err_str} | "
                f"{posx_str} | {posy_str} | {posz_str} || "
                f"{tgt_x_str} | {tgt_y_str} | {tgt_z_str}"
            )
            print(line)

        print("\n(받은 데이터가 없으면 ‘–’로 표시됩니다.)\n")

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
