import sys, socket, struct, threading, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from remote_runtime.wire import send_packet, receive_packet, MAX_PAYLOAD
from remote_runtime.command_guard import CommandGuard

class ProtocolTests(unittest.TestCase):
    def test_binary_packet_boundaries(self):
        a,b=socket.socketpair()
        try:
            send_packet(a,{'seq':1},b'\0\n\xff')
            send_packet(a,{'seq':2})
            self.assertEqual(receive_packet(b),({'seq':1},b'\0\n\xff'))
            self.assertEqual(receive_packet(b),({'seq':2},b''))
        finally:a.close();b.close()

    def test_fragmented_header_and_body(self):
        a,b=socket.socketpair()
        raw=b'{"seq":3}'
        data=struct.pack('!II',len(raw),3)+raw+b'abc'
        def writer():
            for value in data:a.sendall(bytes([value]))
        t=threading.Thread(target=writer);t.start()
        try:self.assertEqual(receive_packet(b),({'seq':3},b'abc'))
        finally:t.join();a.close();b.close()

    def test_oversized_and_nonfinite_headers_rejected(self):
        a,b=socket.socketpair()
        try:
            a.sendall(struct.pack('!II',1,MAX_PAYLOAD+1))
            with self.assertRaises(ValueError):receive_packet(b)
            with self.assertRaises(ValueError):send_packet(a,{'v':float('nan')})
        finally:a.close();b.close()

class GuardTests(unittest.TestCase):
    def setUp(self):
        self.g=CommandGuard('robot',max_age=1.)
        self.g.note_state(1,0,20.,100.)
        self.packet={'session':'robot','epoch':0,'state_seq':1,'action_timestamp':20.,'action':{'v':.2,'r':.1}}

    def test_old_reply_and_repeated_heartbeat_expire(self):
        self.assertTrue(self.g.accept(self.packet,100.2))
        self.assertTrue(self.g.accept(self.packet,100.8))
        self.assertEqual(self.g.command(101.01,20.9),(0.,0.))
        self.assertFalse(self.g.accept(self.packet,101.1))

    def test_bad_command_and_previous_session_rejected(self):
        for change in [{'session':'other'},{'epoch':1},{'state_seq':-1},{'action':{'v':float('nan'),'r':0}}, {'action':{'v':1,'r':0}}, {'action_timestamp':19.}]:
            packet={**self.packet,**change}
            # Exactly one second old action remains valid until deadline; test beyond it.
            if change=={'action_timestamp':19.}:packet['action_timestamp']=18.9
            self.assertFalse(self.g.accept(packet,100.2))

    def test_reset_and_disconnect_stop_without_waiting_for_network(self):
        self.g.accept(self.packet,100.2)
        self.g.invalidate();self.assertEqual(self.g.command(100.3,20.3),(0.,0.))
        self.g.accept(self.packet,100.3)
        self.g.reset(1);self.assertEqual(self.g.command(100.4,0.),(0.,0.))
        self.assertFalse(self.g.accept(self.packet,100.4))

if __name__=='__main__':unittest.main()
