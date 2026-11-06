"""Coherent firmware snapshots, bounded plots, and continuous local capture."""
from collections import OrderedDict, deque
import csv
import datetime
import io
import json
from pathlib import Path
import threading
import time

STATES = ('等待起摆', '自动起摆·首次推动', '自动起摆·回程', '恢复搜索/等待接管',
          '直立平衡', '故障·电机停用', '动态补摆', '上电待机·按板载复位启动')
FIELDS = ('id', 'received_at', 'elapsed_s', 'board_uptime_ms', 'pair_sequence',
          'coherent', 'protocol_version', 'state_code', 'state', 'mode', 'adc_ok',
          'balance_valid', 'angle_adc', 'filtered_angle_adc', 'target_adc',
          'angle_error_adc', 'angle_delta_5ms', 'position_counts', 'position_target_counts',
          'destination_counts', 'position_error_counts', 'speed_counts_s', 'speed_target_counts_s',
          'pwm_command', 'pwm_applied', 'x', 'y', 'accepted_x', 'red_pixels',
          'tracking', 'braking', 'turning', 'target_live', 'camera_valid',
          'configured', 'locked', 'camera_sequence', 'camera_age_s', 'k1_presses',
          'k2_presses', 'home_counts', 'center_counts', 'camera_error',
          'camera_failed_register', 'bad_crc', 'capture_segment', 'returning', 'meteor_mode', 'k3_presses', 'meteor_wait_bottom',
          'meteor_releasing', 'meteor_release_exhausted', 'fault_reason', 'k1_pending',
          'angle_setting','setting_degrees','setting_k1_presses','setting_k2_presses','setting_k3_presses',
          'windmill_mode','windmill_fault','windmill_preparing','windmill_reversing','windmill_negative','windmill_acquiring')
EVENT_FIELDS = ('id', 'elapsed_s', 'board_uptime_ms', 'kind', 'text', 'mode')


def mode_name(code, valid, vision):
    if vision and vision.get('windmill_mode'):
        if vision.get('windmill_fault') or code==5:
            return 'K3 大风车 · 水平臂行程保护，按K3退出'
        if vision.get('adc_ok') is False:
            return 'K3 大风车 · 角度数据无效，暂停驱动'
        if vision.get('windmill_acquiring'):
            return 'K3 大风车 · 先甩起转圈，确认连续两圈'
        if vision.get('windmill_preparing'):
            return 'K3 大风车 · 先制动水平臂，确定中心'
        if vision.get('windmill_reversing'):
            return 'K3 大风车 · 单向控制，正在抑制反转'
        return 'K3 大风车 · 维持单向转动，按圈逐步收范围'
    if code==5:
        reasons={1:'首次推动位移不足',2:'回程位移超限',3:'回程超时且位移不足',
                 4:'补摆超时且位移不足',5:'摆臂行程保护'}
        prefix='K3 流星锤 · 故障停机' if vision and vision.get('meteor_mode') else '故障·电机停用'
        reason=reasons.get((vision or {}).get('fault_reason'))
        return prefix+('：'+reason if reason else '')
    if vision and vision.get('meteor_mode'):
        if vision.get('meteor_wait_bottom'):
            if vision.get('meteor_releasing'):
                return 'K3 流星锤 · 启动推动，释放直立'
            if vision.get('meteor_release_exhausted'):
                return 'K3 流星锤 · 启动推动结束，等待下垂'
            return 'K3 流星锤 · 等待摆杆下垂（电机关闭）'
        if vision.get('adc_ok') is False:
            return 'K3 流星锤 · 角度数据无效，暂停驱动'
        actions={0:'下垂准备起摆',1:'首次推动',2:'反向甩起',3:'过顶下落 / 等待补摆',6:'按摆杆相位补摆'}
        return 'K3 流星锤 · '+actions.get(code,'等待起摆状态')
    if code != 4:
        return STATES[code] if code is not None and 0 <= code < len(STATES) else '等待平衡状态'
    if not valid:
        return '退出平衡/等待接管'
    if not vision:
        return '直立平衡（模式未回传）'
    if vision.get('angle_setting'):
        return 'K4 目标角度设置 · {}°'.format(vision.get('setting_degrees',0))
    if vision.get('turning'):
        speed = vision.get('planned_speed_50ms_q8', 0)
        distance = vision.get('destination_counts', 0)-vision.get('absolute_counts', 0)
        return 'K1 顺时针定位' if (speed > 0 or speed == 0 and distance >= 0) else 'K1 逆时针定位'
    if vision.get('k1_pending'):
        return 'K1 位置设定 · 等待连按结束（1秒）'
    if vision.get('braking'):
        return '平滑停车/模式交接'
    if vision.get('tracking'):
        return 'K2 红色目标跟踪' if vision.get('target_live') else 'K2 跟踪·等待有效目标'
    if vision.get('returning'):
        return '平滑自动回位'
    return '定点直立保持'


class Capture:
    def __init__(self, root, metadata):
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        stem = 'pendulum_live_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.csv_path = root/(stem+'.csv')
        self.events_path = root/(stem+'_events.csv')
        self.raw_path = root/(stem+'.raw.bin')
        self.json_path = root/(stem+'.json')
        self.output = self.csv_path.open('w', encoding='utf-8-sig', newline='')
        self.events = self.events_path.open('w', encoding='utf-8-sig', newline='')
        self.raw = self.raw_path.open('wb')
        self.writer = csv.DictWriter(self.output, fieldnames=FIELDS)
        self.writer.writeheader()
        self.event_writer = csv.DictWriter(self.events, fieldnames=EVENT_FIELDS)
        self.event_writer.writeheader()
        self.metadata = dict(metadata, started_at=datetime.datetime.now().astimezone().isoformat(),
                             csv=str(self.csv_path), events=str(self.events_path), raw=str(self.raw_path),
                             schema_version=3, firmware_manifest_is_readback=False)
        self.rows = self.raw_bytes = 0
        self.last_flush = time.monotonic()
        self.closed = False
        self._metadata()

    def _metadata(self):
        self.json_path.write_text(json.dumps(dict(self.metadata, rows=self.rows,
                                 raw_bytes=self.raw_bytes, closed=self.closed),
                                 ensure_ascii=False, indent=2), encoding='utf-8')

    def sample(self, sample):
        self.writer.writerow({key: sample.get(key) for key in FIELDS})
        self.rows += 1

    def event(self, event):
        self.event_writer.writerow({key: event.get(key) for key in EVENT_FIELDS})

    def bytes(self, data):
        self.raw.write(data)
        self.raw_bytes += len(data)

    def flush(self, force=False):
        if force or time.monotonic()-self.last_flush >= .5:
            self.output.flush()
            self.events.flush()
            self.raw.flush()
            self.last_flush = time.monotonic()

    def close(self, stats=None):
        if self.closed:
            return
        self.flush(True)
        for stream in (self.output, self.events, self.raw):
            stream.close()
        self.closed = True
        self.metadata.update(stats or {})
        self.metadata['finished_at'] = datetime.datetime.now().astimezone().isoformat()
        self._metadata()


class Store:
    def __init__(self, capture=None, counts_per_rev=1040, clock=time.monotonic, demo=False, port='COM9', baud=115200):
        self.lock = threading.RLock()
        self.clock = clock
        self.started = clock()
        self.session_id = str(time.time_ns())
        self.view_generation = 0
        self.view_origin = 0.0
        self.capture = capture
        self.counts_per_rev = counts_per_rev
        self.demo = demo
        self.port, self.baud = port, baud
        self.samples = deque(maxlen=18000)
        self.events = deque(maxlen=500)
        self.pending = OrderedDict()
        self.latest_balance = self.latest_vision = None
        self.balance_at = self.vision_at = None
        self.camera_changed = None
        self.camera_sequence = None
        self.connected = False
        self.error = None
        self.bad_crc = self.skipped = self.packets = 0
        self.next_id = self.next_event_id = self.segment = 0
        self.seen_v3 = False
        self.last_uptime = None
        self.last_buttons = None
        self.last_mode = None
        self.capture_error = None

    def connection(self, connected, error=None):
        with self.lock:
            if not connected and not self.connected and error == self.error:
                return
            self.connected, self.error = connected, error
            if connected:
                # Never pair pre-disconnection records with a new stream.
                self.pending.clear()
                self.latest_balance = self.latest_vision = None
                self.balance_at = self.vision_at = None
                self.camera_changed = self.camera_sequence = None
                self.seen_v3 = False
                self.last_uptime = self.last_buttons = self.last_mode = None
                self.segment += 1
            self._event('connection', '串口已连接' if connected else '串口断开'+('：'+error if error else ''))

    def stats(self, parser):
        with self.lock:
            self.bad_crc, self.skipped, self.packets = parser.bad_crc, parser.skipped, parser.packets

    def capture_bytes(self, data):
        with self.lock:
            self._capture('bytes', data)

    def _capture(self, method, value):
        if not self.capture or self.capture_error:
            return
        try:
            getattr(self.capture, method)(value)
            self.capture.flush()
        except OSError as exc:
            self.capture_error = '记录文件写入失败：'+str(exc)

    def _event(self, kind, text, uptime=None, mode=None):
        self.next_event_id += 1
        event = dict(id=self.next_event_id, elapsed_s=round(self.clock()-self.started, 4),
                     board_uptime_ms=uptime, kind=kind, text=text, mode=mode)
        self.events.append(event)
        self._capture('event', event)
        return event

    def mark(self):
        with self.lock:
            return self._event('disturbance', '手动标记：外部扰动', self.last_uptime, self.last_mode)

    def clear_waveforms(self):
        """Reset the shared plot window, without rewinding continuous log time."""
        with self.lock:
            self.view_origin = self.clock()-self.started
            self.view_generation += 1
            self.samples.clear()
            self.events.clear()
            self.pending.clear()
            return dict(session_id=self.session_id, view_generation=self.view_generation)

    def _view_row(self, row):
        return dict(row, elapsed_s=round(max(0.0,row['elapsed_s']-self.view_origin),4))

    def feed(self, record):
        with self.lock:
            now = self.clock()
            if record['kind'] == 'balance':
                self.latest_balance, self.balance_at = record, now
                self.pending[record['sequence']] = (record, now)
                while len(self.pending) > 64:
                    self.pending.popitem(last=False)
                if not self.seen_v3:
                    v = self.latest_vision if self.vision_at is not None and now-self.vision_at < 1.5 else {}
                    self._sample(record, v or {}, now, False)
                return
            self.latest_vision, self.vision_at = record, now
            if not record['configured'] or not record['locked']:
                self.camera_sequence = self.camera_changed = None
            elif record['sequence'] != self.camera_sequence:
                self.camera_sequence, self.camera_changed = record['sequence'], now
            if record['version'] >= 3:
                self.seen_v3 = True
                pair = self.pending.pop(record['pair_sequence'], None)
                coherent = pair is not None and now-pair[1] <= 1.5
                self._sample(pair[0] if coherent else {}, record, now, coherent)

    def _sample(self, b, v, now, coherent):
        version = v.get('version')
        code = v.get('state', b.get('state'))
        valid = v.get('balance_valid', b.get('balance_valid', False))
        angle, target = b.get('angle'), b.get('target_adc')
        control_angle = v.get('filtered_angle_adc', angle)
        position = v.get('absolute_counts', b.get('arm_counts'))
        trajectory = v.get('trajectory_counts') if (version or 0) >= 3 and valid else None
        destination = v.get('destination_counts') if (version or 0) >= 3 and valid else None
        delta = v.get('speed_delta_50ms', b.get('delta_50ms'))
        # Older firmware does not measure speed outside balance.
        speed = delta*20 if delta is not None and ((version or 0) >= 3 or code == 4) else None
        planned_speed = v.get('planned_speed_50ms_q8')
        mode = mode_name(code, valid, v)
        self.next_id += 1
        sample = dict(id=self.next_id, received_at=datetime.datetime.now().astimezone().isoformat(),
                      elapsed_s=round(now-self.started, 4), board_uptime_ms=v.get('uptime_ms'),
                      pair_sequence=v.get('pair_sequence', b.get('sequence')), coherent=coherent,
                      protocol_version=version, state_code=code,
                      state=STATES[code] if code is not None and 0 <= code < len(STATES) else '未知',
                      mode=mode, adc_ok=v.get('adc_ok', b.get('adc_ok')),
                      balance_valid=valid, angle_adc=angle, filtered_angle_adc=control_angle,
                      target_adc=target, angle_error_adc=control_angle-target if control_angle is not None and target is not None else None,
                      angle_delta_5ms=v.get('angle_delta_5ms'), position_counts=position,
                      position_target_counts=trajectory, destination_counts=destination,
                      position_error_counts=trajectory-position if trajectory is not None and position is not None else None,
                      speed_counts_s=speed, speed_target_counts_s=planned_speed*20/256 if planned_speed is not None and valid else None,
                      pwm_command=v.get('motor_command', b.get('pwm')), pwm_applied=v.get('applied_pwm'),
                      x=v.get('x'), y=v.get('y'), accepted_x=v.get('accepted_x'), red_pixels=v.get('red_pixels'),
                      camera_sequence=v.get('sequence'), camera_age_s=round(now-self.camera_changed, 4) if self.camera_changed is not None else None,
                      k1_presses=v.get('k1_presses'), k2_presses=v.get('k2_presses'),
                      k3_presses=v.get('k3_presses'), meteor_mode=v.get('meteor_mode', False),
                      windmill_mode=v.get('windmill_mode',False),windmill_fault=v.get('windmill_fault',False),
                      windmill_preparing=v.get('windmill_preparing',False),
                      windmill_reversing=v.get('windmill_reversing',False),windmill_negative=v.get('windmill_negative',False),
                      windmill_acquiring=v.get('windmill_acquiring',False),
                      meteor_wait_bottom=v.get('meteor_wait_bottom', False),
                      meteor_releasing=v.get('meteor_releasing', False),
                      meteor_release_exhausted=v.get('meteor_release_exhausted', False),
                      fault_reason=v.get('fault_reason', 0),
                      k1_pending=v.get('k1_pending', False),
                      angle_setting=v.get('angle_setting'),setting_degrees=v.get('setting_degrees'),
                      setting_k1_presses=v.get('setting_k1_presses'),setting_k2_presses=v.get('setting_k2_presses'),
                      setting_k3_presses=v.get('setting_k3_presses'),
                      home_counts=v.get('home_counts'), center_counts=v.get('center_counts'),
                      camera_error=v.get('camera_error'), camera_failed_register=v.get('camera_failed_register'),
                      bad_crc=self.bad_crc, capture_segment=self.segment)
        for key in ('tracking', 'braking', 'turning', 'target_live', 'camera_valid', 'configured', 'locked', 'returning'):
            sample[key] = v.get(key)
        if (version or 0) >= 3:
            uptime = v['uptime_ms']
            reset = self.last_uptime is not None and ((uptime-self.last_uptime) & 0xffffffff) > 0x80000000
            if reset:
                self.segment += 1
                sample['capture_segment'] = self.segment
                self._event('reset', '检测到开发板重新计时/复位', uptime, mode)
                self.last_buttons = None
            buttons = v['k1_presses'], v['k2_presses'], v.get('k3_presses', 0)
            if self.last_buttons is not None:
                for i, key in enumerate(('K1', 'K2', 'K3')):
                    count = (buttons[i]-self.last_buttons[i]) & 0xffff
                    if 0 < count < 32768:
                        self._event('button', '{} 按下 ×{}；采样时状态：{}'.format(key, count, mode), uptime, mode)
            self.last_buttons, self.last_uptime = buttons, uptime
        if mode != self.last_mode:
            self._event('mode', mode, sample['board_uptime_ms'], mode)
            self.last_mode = mode
        self.samples.append(sample)
        self._capture('sample', sample)

    def snapshot(self, since=0):
        with self.lock:
            now = self.clock()
            age = now-self.vision_at if self.vision_at is not None else None
            balance_age = now-self.balance_at if self.balance_at is not None else None
            fresh = self.connected and balance_age is not None and balance_age < 1.5
            if self.seen_v3:
                fresh = fresh and age is not None and age < 1.5
            camera_age = now-self.camera_changed if self.camera_changed is not None else None
            latest = self.samples[-1] if self.samples else None
            mode = latest['mode'] if latest else '等待串口数据'
            if not self.connected:
                mode = '串口未连接'
            elif age is not None and age >= 1.5:
                mode = '遥测已过期'
            elif self.seen_v3 and not fresh:
                mode = '角度包缺失或已过期'
            elif not self.seen_v3 and latest:
                mode += ' · 旧遥测缺少完整字段'
            elif (camera_age is not None and camera_age >= .650 and latest and
                  latest.get('tracking') and latest.get('state_code') == 4 and
                  latest.get('balance_valid') and not latest.get('braking')):
                mode = '跟踪：摄像头帧号停止更新'
            samples = list(self.samples)
            samples = samples[-1000:] if since <= 0 else [s for s in samples if s['id'] > since][:2000]
            return dict(status=dict(connected=self.connected, fresh=fresh, error=self.error,
                        session_id=self.session_id, view_generation=self.view_generation,
                        capture_error=self.capture_error, mode=mode, demo=self.demo, port=self.port, baud=self.baud,
                        frames=self.next_id, packets=self.packets, bad_crc=self.bad_crc, skipped_bytes=self.skipped,
                        vision_age_s=age, balance_age_s=balance_age, camera_age_s=camera_age,
                        version=self.latest_vision.get('version') if self.latest_vision else None,
                        counts_per_rev=self.counts_per_rev,
                        log_csv=str(self.capture.csv_path) if self.capture else None,
                        log_events=str(self.capture.events_path) if self.capture else None),
                        latest=self._view_row(latest) if latest else None,
                        samples=[self._view_row(s) for s in samples],
                        events=[self._view_row(e) for e in list(self.events)[-60:]])

    def csv_bytes(self):
        with self.lock:
            output = io.StringIO(newline='')
            writer = csv.DictWriter(output, fieldnames=FIELDS)
            writer.writeheader()
            for sample in self.samples:
                row=self._view_row(sample)
                writer.writerow({key: row.get(key) for key in FIELDS})
            return ('\ufeff'+output.getvalue()).encode('utf-8')
