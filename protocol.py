"""CRC-protected AA56 and VTX1 v1 through v6 observation packets."""
import binascii
import struct

BALANCE = struct.Struct('>2sBBHHHhii')
VISION = struct.Struct('>4sBBHHHii')
VISION_V2 = struct.Struct('>4sBBHHHHii')
VISION_V3 = struct.Struct('>4sBBHHHHHiiiiiiiihHHIHBHBiii')
VISION_V4 = struct.Struct(VISION_V3.format+'BH')
VISION_V5 = struct.Struct(VISION_V4.format+'BHHHH')
VISION_V6 = struct.Struct(VISION_V5.format)


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = self.skipped = self.packets = 0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            starts = [(self.buffer.find(h), h) for h in (b'\xaa\x56', b'VTX1')]
            starts = [(i, h) for i, h in starts if i >= 0]
            if not starts:
                # Preserve a fragmented header without buffering unbounded noise.
                keep = 0
                for h in (b'\xaa\x56', b'VTX1'):
                    for n in range(1, len(h)):
                        if self.buffer.endswith(h[:n]):
                            keep = max(keep, n)
                discard = len(self.buffer)-keep
                self.skipped += discard
                del self.buffer[:discard]
                break
            start, magic = min(starts)
            self.skipped += start
            del self.buffer[:start]
            if magic == b'VTX1':
                if len(self.buffer) < 5:
                    break
                version = self.buffer[4]
                if version not in (1, 2, 3, 4, 5, 6):
                    self.skipped += 1
                    del self.buffer[0]
                    continue
                fmt = {1: VISION, 2: VISION_V2, 3: VISION_V3, 4: VISION_V4, 5: VISION_V5, 6: VISION_V6}[version]
            else:
                fmt = BALANCE
            packet_size = fmt.size+2
            if len(self.buffer) < packet_size:
                break
            packet = bytes(self.buffer[:packet_size])
            if binascii.crc_hqx(packet[:-2], 0xffff) != int.from_bytes(packet[-2:], 'big'):
                self.bad_crc += 1
                self.skipped += 1
                del self.buffer[0]
                continue
            fields = fmt.unpack(packet[:-2])
            if fmt is BALANCE and fields[1] != 1:
                self.skipped += 1
                del self.buffer[0]
                continue
            if fmt is BALANCE:
                _, _, flags, sequence, angle, target, pwm, position, delta = fields
                rec = dict(kind='balance', sequence=sequence, flags=flags,
                           angle=angle, target_adc=target, pwm=pwm,
                           arm_counts=position, delta_50ms=delta, state=(flags >> 4) & 7,
                            balance_valid=bool(flags & 2), adc_ok=bool(flags & 4),returning=bool(flags & 128))
            else:
                if fmt in (VISION_V3, VISION_V4, VISION_V5, VISION_V6):
                    (_, _, flags, x, y, count, accepted, sequence, center, goal,
                     absolute, trajectory, destination, home, speed, target_speed,
                     applied_pwm, k1, k2, uptime, pair_sequence, camera_error,
                     failed_register, control_flags, motor_command, filtered_angle,
                     angle_delta) = fields[:27]
                elif fmt is VISION_V2:
                    _, _, flags, x, count, accepted, sequence, center, goal = fields
                else:
                    _, _, flags, x, count, sequence, center, goal = fields
                    accepted = 65535
                valid = bool(flags & 8)
                if ((valid and x > 207*256) or (not valid and x != 65535) or
                        (accepted != 65535 and (accepted > 207*256 or not flags & 1)) or
                        (fmt in (VISION_V3, VISION_V4, VISION_V5, VISION_V6) and ((valid and y > 155*256) or (not valid and y != 65535)))):
                    self.skipped += 1
                    del self.buffer[0]
                    continue
                rec = dict(kind='vision', flags=flags, sequence=sequence,
                           x=x/256.0 if valid else None, red_pixels=count,
                           version=fields[1], accepted_x=accepted/256.0 if accepted!=65535 else None,
                           center_counts=center, goal_counts=goal,
                           tracking=bool(flags & 1), engaged=bool(flags & 2),
                           target_live=bool(flags & 4), camera_valid=valid,
                           configured=bool(flags & 16), locked=bool(flags & 32),
                           braking=bool(flags & 64), turning=bool(flags & 128))
                if fmt in (VISION_V3, VISION_V4, VISION_V5, VISION_V6):
                    rec.update(y=y/256.0 if valid else None, absolute_counts=absolute,
                               trajectory_counts=trajectory, destination_counts=destination,
                               home_counts=home, speed_delta_50ms=speed,
                               planned_speed_50ms_q8=target_speed, applied_pwm=applied_pwm,
                               k1_presses=k1, k2_presses=k2, uptime_ms=uptime,
                               pair_sequence=pair_sequence, camera_error=camera_error,
                               camera_failed_register=failed_register,
                               state=(control_flags >> 4) & 7, adc_ok=bool(control_flags & 4),
                               balance_valid=bool(control_flags & 2),returning=bool(control_flags & 128), motor_command=motor_command,
                               filtered_angle_adc=filtered_angle, angle_delta_5ms=angle_delta)
                if fmt in (VISION_V4, VISION_V5, VISION_V6):
                    rec.update(meteor_mode=bool(fields[27] & 1),
                               meteor_wait_bottom=bool(fields[27] & 2),
                               meteor_releasing=bool(fields[27] & 4),
                               meteor_release_exhausted=bool(fields[27] & 8),
                               fault_reason=(fields[27] >> 4) & 7,
                               k1_pending=bool(fields[27] & 128),k3_presses=fields[28])
                if fmt in (VISION_V5, VISION_V6):
                    setting, degrees, sk1, sk2, sk3 = fields[29:34]
                    setting_flags=setting
                    setting=setting_flags & 1
                    if (setting_flags & ~(127 if fmt is VISION_V6 else 1)) or degrees>999 or (not setting and (degrees or sk1 or sk2 or sk3)):
                        self.skipped += 1
                        del self.buffer[:packet_size]
                        continue
                    rec.update(windmill_mode=bool(setting_flags & 2),windmill_fault=bool(setting_flags & 4),
                               windmill_preparing=bool(setting_flags & 8),
                               windmill_reversing=bool(setting_flags & 16),windmill_negative=bool(setting_flags & 32),
                               windmill_acquiring=bool(setting_flags & 64),
                               angle_setting=bool(setting),setting_degrees=degrees,
                               setting_k1_presses=sk1,setting_k2_presses=sk2,setting_k3_presses=sk3)
            result.append(rec)
            self.packets += 1
            del self.buffer[:packet_size]
        return result
