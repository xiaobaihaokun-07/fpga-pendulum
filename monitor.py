"""Local X-only integrated pendulum monitor; no image transfer or PC control."""
import argparse
import csv
import datetime
import json
import pathlib
import queue
import threading
import time
from protocol import Parser

ROOT = pathlib.Path(__file__).resolve().parent
STATE = ['等待起摆', '第一段起摆', '回摆', '恢复/等待接管', '直立平衡', '故障', '动态补摆', '上电待机·按板载复位启动']
LINK_TIMEOUT_S = 1.5
CAMERA_TIMEOUT_S = .650


class TelemetryState:
    """Packet arrival and sensor-frame progress are separate clocks."""
    def __init__(self, start):
        self.start = start
        self.latest = {'balance': {}, 'vision': {}}
        self.updated = {}
        self.camera_sequence = None
        self.camera_updated = None

    def update(self, record, now):
        kind = record['kind']
        self.latest[kind] = record
        self.updated[kind] = now
        if kind == 'vision':
            if not record['configured'] or not record['locked']:
                self.camera_sequence = self.camera_updated = None
            elif self.camera_sequence != record['sequence']:
                self.camera_sequence = record['sequence']
                self.camera_updated = now

    def snapshot(self, now):
        b, v = self.latest['balance'], self.latest['vision']
        balance_age = now-self.updated.get('balance', self.start)
        vision_age = now-self.updated.get('vision', self.start)
        camera_age = None if self.camera_updated is None else now-self.camera_updated
        link_stale = not b or not v or balance_age > LINK_TIMEOUT_S or vision_age > LINK_TIMEOUT_S
        camera_stale = camera_age is not None and camera_age >= CAMERA_TIMEOUT_S
        mode = description(b, v)
        if link_stale:
            mode = '等待串口数据' if not b and not v else '串口数据未齐全或已过期'
        elif camera_stale and v.get('tracking') and not v.get('braking') and b.get('state') == 4:
            mode = '跟踪：摄像头帧号停止更新'
        return dict(balance=b, vision=v, mode=mode, balance_age=balance_age,
                    vision_age=vision_age, camera_age=camera_age,
                    link_stale=bool(link_stale), camera_stale=camera_stale)


def publish(events, value):
    # The console has no UI consumer. A paused GUI needs only recent status;
    # CSV/raw capture is independent and must never block on rendering.
    if events is None:
        return
    try:
        events.put_nowait(value)
    except queue.Full:
        try:
            events.get_nowait()
        except queue.Empty:
            pass
        events.put_nowait(value)


def description(b, v):
    if v.get('windmill_mode'):
        from dashboard.store import mode_name
        return mode_name(b.get('state'),False,v)
    if b.get('state')==5 and v.get('fault_reason'):
        from dashboard.store import mode_name
        return mode_name(5,False,v)
    if v.get('meteor_mode'):
        if v.get('meteor_wait_bottom'):
            if v.get('meteor_releasing'):
                return 'K3 流星锤 · 启动推动，释放直立'
            return 'K3 流星锤 · 等待摆杆下垂（电机关闭）'
        actions={0:'下垂准备起摆',1:'首次推动',2:'反向甩起',3:'过顶下落 / 等待补摆',5:'故障停机',6:'按摆杆相位补摆'}
        return 'K3 流星锤 · '+actions.get(b.get('state'),'等待起摆状态')
    if b and b.get('state') != 4:
        return STATE[b['state']] if b['state'] < len(STATE) else '未知状态'
    if v.get('turning'):
        return 'K1 位置移动'
    if v.get('k1_pending'):
        return 'K1 位置设定 · 等待连按结束（1秒）'
    if v.get('braking'):
        return '平滑停车'
    if v.get('tracking'):
        return '红色目标跟踪' if v.get('target_live') else '跟踪：等待有效目标'
    return '定点平衡' if b else '等待串口数据'


def acquire(args, events, stop):
    import serial
    parser = Parser()
    folder = ROOT/'logs'
    folder.mkdir(exist_ok=True)
    stem = 'pendulum_vision_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    columns = ['elapsed_s', 'packet', 'state', 'adc_ok', 'angle', 'target_adc', 'pwm',
               'arm_counts', 'delta_50ms', 'tracking', 'braking', 'target_live', 'x', 'accepted_x',
               'red_pixels', 'center_counts', 'goal_counts', 'configured', 'locked',
               'camera_sequence', 'bad_crc', 'turning', 'balance_valid', 'camera_valid',
               'balance_age_s', 'vision_age_s', 'camera_frame_age_s', 'camera_stale']
    start = last_print = time.monotonic()
    telemetry = TelemetryState(start)
    try:
        with serial.Serial(args.port, args.baud, timeout=.1) as stream, \
                (folder/(stem+'.csv')).open('w', encoding='utf-8-sig', newline='') as output, \
                (folder/(stem+'.raw.bin')).open('wb') as raw:
            stream.reset_input_buffer()
            writer = csv.DictWriter(output, fieldnames=columns)
            writer.writeheader()
            print('串口 {} @ {}；日志 {}'.format(args.port, args.baud, output.name), flush=True)
            saved = 0
            while not stop.is_set() and (not args.seconds or time.monotonic()-start < args.seconds):
                chunk = stream.read(max(1, min(stream.in_waiting, 4096)))
                part = chunk[:max(0, 1048576-saved)]
                raw.write(part)
                saved += len(part)
                now = time.monotonic()
                for rec in parser.feed(chunk):
                    telemetry.update(rec, now)
                    snapshot = telemetry.snapshot(now)
                    b, v = snapshot['balance'], snapshot['vision']
                    row = dict(b)
                    row.update(v)
                    row.update(elapsed_s=round(now-start, 4), packet=rec['kind'],
                               camera_sequence=v.get('sequence'), bad_crc=parser.bad_crc,
                               balance_age_s=round(snapshot['balance_age'], 4),
                               vision_age_s=round(snapshot['vision_age'], 4),
                               camera_frame_age_s='' if snapshot['camera_age'] is None else round(snapshot['camera_age'], 4),
                               camera_stale=snapshot['camera_stale'])
                    writer.writerow({k: row.get(k, '') for k in columns})
                if now-last_print >= .5:
                    output.flush()
                    status = telemetry.snapshot(now)
                    b, v = status['balance'], status['vision']
                    status.update(packets=parser.packets, bad_crc=parser.bad_crc,
                                  skipped=parser.skipped, csv=output.name)
                    publish(events, status)
                    print('{}；ADC={} PWM={} 位置={} 原始X={} 采用X={} 跟踪目标={} CRC={}'.format(
                        status['mode'], b.get('angle', '--'), b.get('pwm', '--'),
                        b.get('arm_counts', '--'), v.get('x'), v.get('accepted_x'), v.get('goal_counts', '--'),
                        parser.bad_crc), flush=True)
                    last_print = now
            output.flush()
    except Exception as exc:
        publish(events, {'error': str(exc)})
        print('串口错误：'+str(exc), flush=True)
    finally:
        publish(events, {'done': True})


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--console', action='store_true')
    ap.add_argument('--smoke-test', action='store_true', help='只检查窗口，不打开串口')
    args = ap.parse_args()
    events, stop = queue.Queue(maxsize=4), threading.Event()
    if args.console:
        try:
            acquire(args, None, stop)
        except KeyboardInterrupt:
            stop.set()
        return
    import tkinter as tk
    from tkinter import ttk
    app = tk.Tk()
    app.title('倒立摆 + 摄像头 X 跟踪 — '+args.port)
    app.geometry('800x660')
    ttk.Label(app, text='自动起摆 · K1 正反转圈 · K2 红色目标跟踪', font=('Microsoft YaHei UI', 18)).pack(pady=18)
    mode = tk.StringVar(value='等待串口数据')
    ttk.Label(app, textvariable=mode, font=('Microsoft YaHei UI', 16)).pack(pady=8)
    body = ttk.Frame(app)
    body.pack(fill='x', padx=35)
    labels = [('angle', '摆杆 ADC / 动态目标'), ('position', '水平臂位置 / 速度'),
              ('pwm', '原始 PWM 指令 / ADC 状态'), ('x', '原始 X / 控制采用 X'),
              ('count', '红色像素 / 摄像头状态'), ('goal', '跟踪目标 / 跟踪中心'),
              ('link', '串口数据包 / CRC 异常')]
    variables = {}
    for i, (key, label) in enumerate(labels):
        ttk.Label(body, text=label).grid(row=i, column=0, sticky='w', pady=12)
        variables[key] = tk.StringVar(value='--')
        ttk.Label(body, textvariable=variables[key], font=('Microsoft YaHei UI', 12)).grid(row=i, column=1, sticky='w', padx=20)
    ttk.Label(app, text='位置、跟踪目标和中心均相对本次平衡接管点；1040 编码器计数为一圈。').pack(pady=12)
    ttk.Label(app, text='三帧中值滤波 + 5 像素死区；K2 退出后锁定实际停车位置。K1 转圈期间 K2 不打断本圈。').pack()
    path = tk.StringVar(value='自动保存 CSV；窗口只监测，不向电机发送命令。')
    ttk.Label(app, textvariable=path, wraplength=730).pack(pady=12)
    def poll():
        while True:
            try:
                value = events.get_nowait()
            except queue.Empty:
                break
            if 'error' in value:
                mode.set('串口错误：'+value['error'])
                continue
            if 'done' in value:
                if not args.smoke_test:
                    mode.set('采集已停止')
                continue
            b, v = value['balance'], value['vision']
            stale = value['link_stale']
            mode.set(value['mode'])
            variables['angle'].set('{} / {}'.format(b.get('angle', '--'), b.get('target_adc', '--')))
            pos, delta = b.get('arm_counts'), b.get('delta_50ms')
            variables['position'].set('--' if pos is None else '{:.1f}° / {:.1f}°/s'.format(pos*360/1040, delta*20*360/1040))
            variables['pwm'].set('{} / {}'.format(b.get('pwm', '--'), '正常' if b.get('adc_ok') else '未就绪'))
            x = v.get('x')
            accepted = v.get('accepted_x')
            variables['x'].set('-- / --' if stale or value['camera_stale'] else '{} / {}'.format(
                '--' if x is None else '{:.2f}'.format(x),
                '--' if accepted is None else '{:.2f}'.format(accepted)))
            camera_state = '初始化/等待同步'
            if v.get('locked'):
                camera_state = '帧号停止更新' if value['camera_stale'] else '已锁定'
            if value['camera_age'] is not None:
                camera_state += '（帧龄 {:.2f}s）'.format(value['camera_age'])
            variables['count'].set('{} / {}'.format(v.get('red_pixels', '--'), camera_state))
            if v.get('engaged'):
                variables['goal'].set('{} / {} 计数'.format(v.get('goal_counts', '--'), v.get('center_counts', '--')))
            elif v.get('turning') and v.get('version') in (2, 3):
                variables['goal'].set('K1 当前目标：{} 计数'.format(v.get('goal_counts', '--')))
            elif v.get('version') in (2, 3):
                variables['goal'].set('原地保持：{} 计数'.format(v.get('goal_counts', '--')))
            else:
                variables['goal'].set('未启用跟踪')
            variables['link'].set('{} / {}'.format(value['packets'], value['bad_crc']))
            path.set(value['csv'])
        if not stop.is_set():
            app.after(100, poll)
    def close():
        stop.set()
        app.destroy()
    app.protocol('WM_DELETE_WINDOW', close)
    worker = None
    if args.smoke_test:
        mode.set('窗口检查：未连接串口')
        app.after(700, close)
    else:
        worker = threading.Thread(target=acquire, args=(args, events, stop), daemon=True)
        worker.start()
    poll()
    app.mainloop()
    if worker:
        worker.join(timeout=1)
    if args.smoke_test:
        print('PASS: integrated monitor rendered without opening serial')


if __name__ == '__main__':
    main()
