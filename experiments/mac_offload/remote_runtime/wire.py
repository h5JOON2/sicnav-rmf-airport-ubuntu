import json
import struct

MAX_HEADER = 256 * 1024
MAX_PAYLOAD = 8 * 1024 * 1024


def receive_exact(sock, length):
    chunks = bytearray()
    while len(chunks) < length:
        part = sock.recv(length-len(chunks))
        if not part:
            raise EOFError('TCP connection closed')
        chunks.extend(part)
    return bytes(chunks)


def send_packet(sock, header, payload=b''):
    raw = json.dumps(header, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(raw) > MAX_HEADER or len(payload) > MAX_PAYLOAD:
        raise ValueError('Packet exceeds configured bound')
    sock.sendall(struct.pack('!II', len(raw), len(payload)) + raw + payload)


def receive_packet(sock):
    header_size, body_size = struct.unpack('!II', receive_exact(sock, 8))
    if not 1 <= header_size <= MAX_HEADER or body_size > MAX_PAYLOAD:
        raise ValueError('Invalid packet size')
    def invalid(value):
        raise ValueError('Non-finite JSON number: '+value)
    header = json.loads(receive_exact(sock, header_size), parse_constant=invalid)
    if not isinstance(header, dict):
        raise ValueError('Header must be an object')
    return header, receive_exact(sock, body_size)
