import struct
import threading
import serial

class ParseState:
    WAIT_SYNC1      = 0
    WAIT_SYNC2      = 1
    WAIT_LENGTH     = 2
    WAIT_HEADER     = 3
    WAIT_DATA       = 4
    WAIT_CHECKSUM_1 = 5
    WAIT_CHECKSUM_2 = 6

class JFiProtocol:
    HEADER_SIZE   = 5   # SYNC1, SYNC2, LENGTH, SEQ, SID
    CHECKSUM_SIZE = 2   # 16-bit checksum (little endian)

    def __init__(self):
        self.state          = ParseState.WAIT_SYNC1
        self.buffer         = bytearray()
        self.checksum       = 0
        self.payload_length = None

    @staticmethod
    def create_packet(payload: bytes, seq: int = 0, sid: int = 1) -> bytes:
        """
        J-Fi 패킷 생성기 (헤더 + 페이로드 + 체크섬)
          - SYNC1=0xAA, SYNC2=0x55
          - HEADER: [SYNC1, SYNC2, LENGTH, SEQ, SID]
          - CHECKSUM: 전체 바이트(HEADER+PAYLOAD)에 대해 16-bit XOR
        Args:
            payload: 페이로드 바이트
            seq:     J-Fi 시퀀스 번호 (0~255)
            sid:     송신자 시스템 ID (0~255)
        Returns:
            완성된 J-Fi 패킷 (bytes)
        """
        sync1 = 0xAA
        sync2 = 0x55

        # 총 길이 = HEADER(5) + PAYLOAD + CHECKSUM(2)
        length = JFiProtocol.HEADER_SIZE + len(payload) + JFiProtocol.CHECKSUM_SIZE

        # HEADER: SYNC1, SYNC2, LENGTH, SEQ, SID
        header = struct.pack('BBBBB', sync1, sync2, length, seq, sid)
        packet = header + payload

        checksum = JFiProtocol.compute_checksum(packet)
        packet += struct.pack('<H', checksum)  # little-endian 16-bit

        return packet

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """
        16-bit XOR 체크섬 계산
        """
        c = 0
        for b in data:
            c ^= b
        return c

    def reset_parser(self) -> None:
        self.state          = ParseState.WAIT_SYNC1
        self.buffer         = bytearray()
        self.checksum       = 0
        self.payload_length = None

    def validate_checksum(self) -> bool:
        """
        버퍼에 쌓인 바이트(마지막 2바이트 제외)에 대해 XOR을 계산한 뒤,
        마지막 2바이트(수신된 체크섬)와 비교.
        """
        if len(self.buffer) < JFiProtocol.CHECKSUM_SIZE:
            return False
        calc = 0
        for b in self.buffer[:-JFiProtocol.CHECKSUM_SIZE]:
            calc ^= b
        return calc == self.checksum

    def parse(self, byte: int) -> bytes | None:
        """
        한 바이트씩 feed하여 패킷 전체가 완성되면 ‘완성된 패킷(bytes)’을 반환하고,
        그렇지 않으면 None을 반환.
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
            packet = None
            if self.validate_checksum():
                packet = bytes(self.buffer)
            self.reset_parser()
            return packet

        return None

class JFiInterface:
    """
    실제 시리얼 포트를 열고, J-Fi 패킷을 보내거나 수신할 때 사용합니다.

    self.serial_port:      pyserial.Serial 객체
    self.parser:           JFiProtocol 인스턴스
    self.recv_thread:      수신 스레드
    self.receive_callback: 수신된 완성 패킷을 처리할 콜백 함수
    """

    def __init__(self, port_name: str, baudrate: int, receive_callback):
        """
        Args:
          port_name:         시리얼 포트 이름 (예: '/dev/ttyUSB0')
          baudrate:          시리얼 통신 속도 (예: 115200)
          receive_callback:  패킷이 완성되었을 때 호출할 함수(packet_bytes, header_fields)
        """
        self.parser = JFiProtocol()
        self.serial_port = None
        self.receive_callback = receive_callback

        try:
            self.serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.1)
        except serial.SerialException as e:
            raise RuntimeError(f"Cannot open serial port {port_name}: {e}")

        # 수신 전용 스레드 시작
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

    def send_packet(self, payload: bytes, seq: int, sid: int) -> None:
        """
        payload: 바이트열 그대로 전달.
        seq, sid: 0~255 정수
        """
        packet = JFiProtocol.create_packet(payload, seq=seq, sid=sid)
        self.serial_port.write(packet)

    def _recv_loop(self):
        """
        시리얼 포트에서 바이트를 읽어 와서, parser.parse(byte) 한 뒤,
        완성된 패킷이 들어오면 self.receive_callback(packet_bytes, header).
        """
        while True:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for b in data:
                        packet = self.parser.parse(b)
                        if packet:
                            # 헤더 필드 추출
                            header = packet[:JFiProtocol.HEADER_SIZE]
                            sync1, sync2, length, seq, sid = struct.unpack('BBBBB', header)
                            # 페이로드만 추출
                            payload_len = length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
                            payload = packet[JFiProtocol.HEADER_SIZE : JFiProtocol.HEADER_SIZE + payload_len]
                            self.receive_callback(payload, seq, sid)
            except serial.SerialException:
                break
