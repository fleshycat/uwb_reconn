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
        J-Fi packet (header + payload + checksum)
          - SYNC1=0xAA, SYNC2=0x55
          - HEADER: [SYNC1, SYNC2, LENGTH, SEQ, SID]
          - CHECKSUM: 16-bit XOR for all bytes
        Args:
            payload: data to be sent (bytes)
            seq:     J-Fi sequence num (0~255)
            sid:     Source System ID (0~255)
        Returns:
            Created J-Fi packet (bytes)
        """
        sync1 = 0xAA
        sync2 = 0x55

        # Length = HEADER(5) + PAYLOAD + CHECKSUM(2)
        length = JFiProtocol.HEADER_SIZE + len(payload) + JFiProtocol.CHECKSUM_SIZE

        # HEADER: SYNC1, SYNC2, LENGTH, SEQ, SID
        header = struct.pack('BBBBB', sync1, sync2, length, seq, sid)
        packet = header + payload

        checksum = JFiProtocol.compute_checksum(packet)
        packet += struct.pack('<H', checksum)  # little-endian 16-bit

        return packet

    @staticmethod
    def compute_checksum(data: bytes) -> int:
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
        if len(self.buffer) < JFiProtocol.CHECKSUM_SIZE:
            return False
        calc = 0
        for b in self.buffer[:-JFiProtocol.CHECKSUM_SIZE]:
            calc ^= b
        return calc == self.checksum

    def parse(self, byte: int) -> bytes | None:
        """
        Feed each byte and return the 'completed packet' when the entire packet is completed,
        Otherwise, return None.
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
    Use to open a physical serial port and send or receive J-Fi packets.

    self.serial_port:      pyserial.Serial object
    self.receive_callback: Callback function to handle received completion packets
    """

    def __init__(self, port_name: str, baudrate: int, receive_callback):
        self.parser = JFiProtocol()
        self.serial_port = None
        self.receive_callback = receive_callback

        try:
            self.serial_port = serial.Serial(port_name, baudrate=baudrate, timeout=0.1)
        except serial.SerialException as e:
            raise RuntimeError(f"Cannot open serial port {port_name}: {e}")

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

    def send_packet(self, payload: bytes, seq: int, sid: int) -> None:
        packet = JFiProtocol.create_packet(payload, seq=seq, sid=sid)
        self.serial_port.write(packet)

    def _recv_loop(self):
        while True:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for b in data:
                        packet = self.parser.parse(b)
                        if packet:
                            # extract header
                            header = packet[:JFiProtocol.HEADER_SIZE]
                            sync1, sync2, length, seq, sid = struct.unpack('BBBBB', header)
                            # extract payload
                            payload_len = length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
                            payload = packet[JFiProtocol.HEADER_SIZE : JFiProtocol.HEADER_SIZE + payload_len]
                            self.receive_callback(payload, seq, sid)
            except serial.SerialException:
                break
