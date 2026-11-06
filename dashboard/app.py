"""Read-only local waveforms and continuous capture for the integrated FPGA."""
import argparse
import binascii
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from protocol import BALANCE, VISION_V3, Parser
from dashboard.store import Capture, Store

HERE = Path(__file__).resolve().parent


def publish_chunk(store, parser, chunk):
    store.capture_bytes(chunk)
    records = parser.feed(chunk)
    store.stats(parser)
    for record in records:
        store.feed(record)


def serial_worker(store, args, stop):
    try:
        import serial
    except ImportError:
        store.connection(False, '缺少 pyserial；运行 python -m pip install pyserial')
        return
    last_error = None
    while not stop.is_set():
        parser = Parser()
        try:
            with serial.Serial(args.port, args.baud, timeout=.1) as stream:
                stream.reset_input_buffer()
                store.connection(True)
                last_error = None
                print('串口已打开，等待 AA56 + VTX1 数据。', flush=True)
                last_print = 0
                while not stop.is_set():
                    chunk = stream.read(max(1, min(stream.in_waiting, 4096)))
                    if chunk:
                        publish_chunk(store, parser, chunk)
                    now = time.monotonic()
                    if now-last_print >= 3:
                        snapshot = store.snapshot()
                        s, status = snapshot['latest'] or {}, snapshot['status']
                        print('{}；角度={} 位置={} 目标={} PWM={} X/Y={}/{}；包={} CRC={}'.format(
                            status['mode'], s.get('angle_adc','--'), s.get('position_counts','--'),
                            s.get('position_target_counts','--'), s.get('pwm_applied','--'),
                            s.get('x','--'), s.get('y','--'), parser.packets, parser.bad_crc), flush=True)
                        last_print = now
        except (OSError, serial.SerialException) as exc:
            error = str(exc)
            store.connection(False, error)
            if error != last_error:
                print('串口暂不可用：'+error+'；每 2 秒自动重试。', flush=True)
                last_error = error
            stop.wait(2)
        else:
            store.connection(False)


def _packet(body):
    return body+struct.pack('>H', binascii.crc_hqx(body, 0xffff))


def demo_worker(store, stop):
    """Exercise the real binary parser; visibly labeled simulated observations."""
    store.connection(True)
    parser = Parser()
    start = time.monotonic()
    sequence = 0
    while not stop.is_set():
        t = time.monotonic()-start
        phase = int(t//8) % 4
        state = 4
        flags = (0x38 if phase == 0 else 0xb8 if phase == 1 else 0x3f if phase == 2 else 0x78)
        destination = round(390*math.sin(t*.4)) if phase == 2 else (1040 if phase == 1 else 0)
        trajectory = round(destination*.8)
        position = round(trajectory+8*math.sin(t*2))
        delta = round(8*2*math.cos(t*2)/20)
        target = 19993+round(25*math.sin(t*.7))
        filtered = target+round(30*math.sin(t*3))
        angle = filtered+round(4*math.sin(t*17))
        pwm = round(500*math.sin(t*3))
        x = max(0, min(52992, round((103.5+75*math.sin(t*.4))*256)))
        y = round((77.5+30*math.cos(t*.35))*256)
        k1 = int(t//32)+int(phase>=1)
        k2 = int(t//32)*2+max(0,phase-1)
        body = VISION_V3.pack(b'VTX1',3,flags,x,y,500,x if flags&1 else 65535,sequence&65535,0,destination,
                            position,trajectory,destination,0,delta,0,pwm,k1,k2,
                            int(t*1000)&0xffffffff,sequence&65535,0,0,0x4f,pwm,filtered,0)
        balance = BALANCE.pack(b'\xaa\x56',1,0x4f,sequence&65535,angle,target,pwm,position,delta)
        publish_chunk(store, parser, _packet(balance)+_packet(body))
        sequence += 1
        stop.wait(.028)


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, data, content_type):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            parsed = urlsplit(self.path)
            if parsed.path == '/api/snapshot':
                try:
                    since = max(0,int(parse_qs(parsed.query).get('since',['0'])[0]))
                except ValueError:
                    return self.reply(400,b'bad since','text/plain')
                data = json.dumps(store.snapshot(since),ensure_ascii=False,allow_nan=False).encode('utf-8')
                return self.reply(200,data,'application/json; charset=utf-8')
            if parsed.path == '/api/export.csv':
                return self.reply(200,store.csv_bytes(),'text/csv; charset=utf-8')
            assets = {'/':('index.html','text/html'),'/index.html':('index.html','text/html'),
                      '/app.js':('app.js','text/javascript'),'/style.css':('style.css','text/css')}
            if parsed.path in assets:
                filename, mime = assets[parsed.path]
                return self.reply(200,(HERE/filename).read_bytes(),mime+'; charset=utf-8')
            return self.reply(404,b'not found','text/plain')

        def do_POST(self):
            path=urlsplit(self.path).path
            if path not in ('/api/mark','/api/clear-waveforms'):
                return self.reply(404,b'not found','text/plain')
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 512:
                    raise ValueError('bad length')
                value = json.loads(self.rfile.read(length))
                expected='disturbance' if path=='/api/mark' else 'clear_waveforms'
                if value != {'kind':expected}:
                    raise ValueError('bad marker')
            except (ValueError, json.JSONDecodeError):
                return self.reply(400,b'bad marker','text/plain')
            result=store.mark() if path=='/api/mark' else store.clear_waveforms()
            return self.reply(200,json.dumps(result,ensure_ascii=False).encode('utf-8'),'application/json; charset=utf-8')

        def log_message(self, *args):
            pass
    return Handler


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM9')
    ap.add_argument('--baud',type=int,default=115200)
    ap.add_argument('--http-port',type=int,default=8765)
    ap.add_argument('--host',choices=('127.0.0.1',),default='127.0.0.1')
    ap.add_argument('--counts-per-rev',type=float,default=1040)
    ap.add_argument('--log-dir',type=Path,default=ROOT/'logs'/'live')
    ap.add_argument('--demo',action='store_true')
    ap.add_argument('--seconds',type=float,default=0,help='自动停止秒数；0 为持续运行')
    args = ap.parse_args(argv)
    if args.counts_per_rev <= 0 or args.baud <= 0 or args.seconds < 0:
        ap.error('标定、波特率必须为正数，seconds 不能为负数')
    return args


def main():
    args = parse_args()
    if hasattr(sys.stdout,'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    # Bind before opening COM: a second instance must not seize the serial port.
    store = Store(counts_per_rev=args.counts_per_rev,demo=args.demo,port=args.port,baud=args.baud)
    try:
        server = ThreadingHTTPServer((args.host,args.http_port),make_handler(store))
    except OSError as exc:
        print('无法启动网页：{}。请先在旧网页后台按 Ctrl+C，或用 --http-port 指定其他端口。'.format(exc),flush=True)
        return 2
    server.daemon_threads = True
    metadata = dict(source='DEMO' if args.demo else 'SERIAL',port=args.port,baud=args.baud,
                    counts_per_rev=args.counts_per_rev,firmware='VTX1 v3/v4 / AA56 v1',
                    capture_time='host monotonic elapsed plus board uptime',
                    position_reference='absolute encoder counts on v3; legacy frames may be relative',
                    PWM='raw motor request and signed latched duty, period 3600')
    manifest = ROOT/'diagnostics'/'build_validation.json'
    if manifest.exists():
        metadata['local_expected_firmware'] = json.loads(manifest.read_text(encoding='utf-8')).get('file')
    store.capture = Capture(args.log_dir/('demo' if args.demo else 'serial'),metadata)
    stop = threading.Event()
    worker = threading.Thread(target=demo_worker if args.demo else serial_worker,
                              args=(store,stop) if args.demo else (store,args,stop),daemon=True)
    worker.start()
    if args.seconds:
        def finish():
            if not stop.wait(args.seconds):
                server.shutdown()
        threading.Thread(target=finish,daemon=True).start()
    print('倒立摆完整波形：http://127.0.0.1:{}/'.format(args.http_port),flush=True)
    print('模式：'+('演示数据，未连接硬件' if args.demo else args.port+' @ '+str(args.baud)),flush=True)
    print('自动记录：'+str(store.capture.csv_path),flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        worker.join(timeout=2)
        server.server_close()
        with store.lock:
            store.capture.close(dict(packets=store.packets,bad_crc=store.bad_crc,skipped=store.skipped))
        print('采集已停止，CSV、事件和原始串口文件已保存。',flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
