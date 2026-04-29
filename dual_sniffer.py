#!/usr/bin/env python3
"""
dual_sniffer.py v1.5
Параллельный опрос двух устройств (200 Гц CSV + 10 Гц NMEA+MAG)
с синхронизацией, интерполяцией магнитных данных и расчётом позиций датчиков.
ИСПРАВЛЕНИЯ v1.5:
  - Рейка теперь по умолчанию перпендикулярна направлению движения (--rail-angle 90.0)
  - Исправлен баг в _print_console (ground_spe -> course)
  - Полная структура geo полей, корректный if mag_
"""

import argparse
import serial
import time
import json
import math
import sys
import threading
import queue
import logging
from datetime import datetime
from typing import Optional, Dict, List, Any

# ============================================================================
# 🛠️ КОНФИГУРАЦИЯ И ЛОГИРОВАНИЕ
# ============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('dual_sniffer')

DEFAULT_CONSOLE_THROTTLE_HZ = 1
DEFAULT_SENSOR_OFFSETS_M = [0.0, 1.2, 2.4, 3.6]
DEFAULT_RECONNECT_DELAY_S = 1.0
STARTUP_TIMEOUT_S = 60.0

# ============================================================================
# 🌍 ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================================

def dmm_to_decimal(dmm_str: str, direction: str) -> Optional[float]:
    if not dmm_str: return None
    try:
        dmm = float(dmm_str)
        if dmm == 0.0: return 0.0
        degrees = int(dmm / 100)
        minutes = dmm - degrees * 100
        decimal = degrees + minutes / 60.0
        return -decimal if direction.upper() in ['S', 'W'] else decimal
    except (ValueError, TypeError):
        return None

def calculate_azimuth(x: float, y: float, declination: float = 0.0) -> float:
    if x == 0 and y == 0: return 0.0
    return (math.degrees(math.atan2(y, x)) + declination + 360) % 360

def calculate_field_strength(x: float, y: float, z: float) -> float:
    return math.sqrt(x**2 + y**2 + z**2)

def interpolate_mag(t_target: float, t_prev: float, t_next: float,
                    m_prev: Dict[str, float], m_next: Dict[str, float]) -> Dict[str, float]:
    dt = t_next - t_prev
    if abs(dt) < 1e-9: return m_prev.copy()
    alpha = max(0.0, min(1.0, (t_target - t_prev) / dt))
    return {
        'x': m_prev['x'] + alpha * (m_next['x'] - m_prev['x']),
        'y': m_prev['y'] + alpha * (m_next['y'] - m_prev['y']),
        'z': m_prev['z'] + alpha * (m_next['z'] - m_prev['z'])
    }

def calculate_sensor_offsets(lat: float, lon: float, rail_bearing: float, offsets: List[float]) -> List[Dict]:
    """
    Расчёт координат датчиков вдоль рейки с заданным пеленгом.
    :param rail_bearing: направление рейки в градусах (0=север, по часовой)
    """
    if lat is None or lon is None or rail_bearing is None: return []
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat))
    if abs(m_lon) < 1.0: m_lon = 1.0
    br_rad = math.radians(rail_bearing)
    return [{
        'id': i, 'offset_m': round(off, 2),
        'latitude': round(lat + (off * math.cos(br_rad)) / m_lat, 7),
        'longitude': round(lon + (off * math.sin(br_rad)) / m_lon, 7)
    } for i, off in enumerate(offsets)]

# ============================================================================
# 📡 ПАРСЕРЫ
# ============================================================================

def parse_plain_csv(line: str, sys_time: float) -> Optional[Dict]:
    line = line.strip()
    if not line or line.startswith('$'): return None
    f = line.split(',')
    if len(f) < 13: return None

    def sf(v):
        try: return float(v) if v.strip() else None
        except: return None

    return {
        'timestamp_sys': sys_time, 'type': 'geo_csv',
        'sensor_id': f[0].strip(), 'utc_time': sf(f[1]), 'state': f[2].strip(),
        'latitude': dmm_to_decimal(f[3].strip(), f[4].strip()), 'n_s_indica': f[4].strip(),
        'longitude': dmm_to_decimal(f[5].strip(), f[6].strip()), 'e_w_indica': f[6].strip(),
        'ground_spe': sf(f[7]), 'position': sf(f[8]), 'date': sf(f[9]),
        'f2': sf(f[10]), 'f3': sf(f[11]), 'alarm': f[12].strip() in ('1', 'True', 'true'),
        '_course': sf(f[8]), '_status': f[2].strip()  # Внутренние алиасы
    }

def parse_nmea_extended(line: str, sys_time: float, declination: float = 0.0) -> Optional[Dict]:
    line = line.strip('\r\n')
    if not line.startswith('$'): return None
    
    mag_data = None
    if ';' in line:
        nmea_part, mag_part = line.split(';', 1)
        if '*' in nmea_part: nmea_part = nmea_part.split('*')[0]
        try:
            vals = [float(v) for v in mag_part.split(',')[:3]]
            if len(vals) == 3:
                mag_data = {
                    'x': vals[0], 'y': vals[1], 'z': vals[2],
                    'azimuth': calculate_azimuth(vals[0], vals[1], declination),
                    'field_strength': calculate_field_strength(*vals)
                }
        except ValueError: pass
        line = nmea_part
    elif '*' in line:
        line = line.split('*')[0]

    fields = line.split(',')
    if len(fields) < 12 or fields[0] not in ('$GNRMC', '$GPRMC'): return None
    
    try:
        res = {
            'timestamp_sys': sys_time, 'type': 'nmea_extended',
            'latitude': dmm_to_decimal(fields[3], fields[4]),
            'longitude': dmm_to_decimal(fields[5], fields[6]),
            'speed_knots': float(fields[7]) if fields[7] else None,
            'course_true': float(fields[8]) if fields[8] else None,
            'status': fields[2], 'time_utc': fields[1], 'date_ddmmyy': fields[9]
        }
        if mag_data:  # ✅ ИСПРАВЛЕНО
            res['magnetometer'] = mag_data
            
        res['valid'] = (res['status'] == 'A' and res.get('latitude') is not None)
        return res
    except Exception:
        return None

# ============================================================================
# 🧵 ПОТОКОВЫЕ ВОРКЕРЫ
# ============================================================================

class SerialWorker:
    def __init__(self, port, baud, parser, q, name, stop_evt):
        self.port, self.baud, self.parser, self.q, self.name, self.stop_evt = port, baud, parser, q, name, stop_evt
        self.ser, self.stats = None, {'read': 0, 'parsed': 0, 'errors': 0}

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            logger.info(f"[{self.name}] Подключен к {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            logger.error(f"[{self.name}] Ошибка подключения: {e}")
            return False

    def run(self):
        delay = DEFAULT_RECONNECT_DELAY_S
        while not self.stop_evt.is_set():
            if not self.ser or not self.ser.is_open:
                if not self.connect():
                    time.sleep(delay)
                    delay = min(delay * 2, 30)
                    continue
                delay = DEFAULT_RECONNECT_DELAY_S
            try:
                line = self.ser.readline()
                if not line: continue
                self.stats['read'] += 1
                parsed = self.parser(line.decode('utf-8', errors='ignore'), time.time())
                if parsed:
                    self.stats['parsed'] += 1
                    self.q.put(parsed)
                else:
                    self.stats['errors'] += 1
            except serial.SerialException:
                logger.warning(f"[{self.name}] Разрыв соединения, переподключение...")
                if self.ser and self.ser.is_open: self.ser.close()
            except Exception as e:
                self.stats['errors'] += 1
                if self.stats['errors'] % 100 == 0: logger.warning(f"[{self.name}] Ошибка: {e}")
        if self.ser and self.ser.is_open: self.ser.close()
        logger.info(f"[{self.name}] Завершён: {self.stats}")

# ============================================================================
# 🔗 STREAM MERGER
# ============================================================================

class StreamMerger:
    def __init__(self, q_geo, q_mag, declination, offsets, rail_angle, callback, stop_evt):
        self.q_geo, self.q_mag = q_geo, q_mag
        self.declination, self.offsets, self.rail_angle = declination, offsets, rail_angle
        self.callback, self.stop_evt = callback, stop_evt
        self.mag_buffer = []
        self.last_out, self.throttle = 0, 1.0 / DEFAULT_CONSOLE_THROTTLE_HZ
        self.stats = {'merged': 0, 'interpolated': 0}

    def _get_mag_context(self, target_t: float) -> Optional[Dict]:
        self.mag_buffer = [m for m in self.mag_buffer if m['timestamp_sys'] >= target_t - 2.0]
        valid = [m for m in self.mag_buffer if m.get('magnetometer')]
        if not valid: return None
        valid.sort(key=lambda x: x['timestamp_sys'])

        for m in valid:
            if abs(m['timestamp_sys'] - target_t) < 0.005:
                return {'timestamp_sys': target_t, 'latitude': m.get('latitude'),
                        'longitude': m.get('longitude'), 'course_true': m.get('course_true'),
                        'magnetometer': m['magnetometer']}

        for i in range(len(valid) - 1):
            t0, t1 = valid[i]['timestamp_sys'], valid[i+1]['timestamp_sys']
            if t0 <= target_t <= t1:
                m_interp = interpolate_mag(target_t, t0, t1, valid[i]['magnetometer'], valid[i+1]['magnetometer'])
                m_interp['azimuth'] = calculate_azimuth(m_interp['x'], m_interp['y'], self.declination)
                m_interp['field_strength'] = calculate_field_strength(m_interp['x'], m_interp['y'], m_interp['z'])
                ref = valid[i] if (target_t - t0) < (t1 - target_t) else valid[i+1]
                return {'timestamp_sys': target_t, 'latitude': ref.get('latitude'),
                        'longitude': ref.get('longitude'), 'course_true': ref.get('course_true'),
                        'magnetometer': m_interp}

        lst = valid[-1]
        return {'timestamp_sys': target_t, 'latitude': lst.get('latitude'),
                'longitude': lst.get('longitude'), 'course_true': lst.get('course_true'),
                'magnetometer': lst['magnetometer']}

    def _build_record(self, geo, mag_ctx, t):
        lat = geo.get('latitude') or (mag_ctx.get('latitude') if mag_ctx else None)
        lon = geo.get('longitude') or (mag_ctx.get('longitude') if mag_ctx else None)

        # Направление движения (азимут рейки)
        azimuth = None
        if mag_ctx and mag_ctx.get('magnetometer'):
            azimuth = mag_ctx['magnetometer'].get('azimuth')
        if azimuth is None:
            azimuth = geo.get('_course') or (mag_ctx.get('course_true') if mag_ctx else None)

        # 📐 НАПРАВЛЕНИЕ РЕЙКИ = курс + угол смещения (по умолчанию 90° = перпендикуляр)
        rail_bearing = (azimuth + self.rail_angle) % 360.0 if azimuth is not None else None

        mag_t = mag_ctx.get('timestamp_sys') if mag_ctx else None
        delta = abs(t - mag_t) * 1000 if mag_t is not None else 0.0

        geo_out = {k: v for k, v in geo.items() if not k.startswith('_')}

        return {
            'timestamp_sys': t, 'timestamp_iso': datetime.now().isoformat(),
            'sync': {'geo_time': geo.get('timestamp_sys'), 'mag_time': mag_t,
                     'delta_ms': round(delta, 2), 'interpolated': bool(mag_ctx and mag_t != t)},
            'geo': geo_out,
            'gps': {'latitude': lat, 'longitude': lon, 'azimuth_true': azimuth,
                    'course_true': geo.get('_course') or (mag_ctx.get('course_true') if mag_ctx else None),
                    'valid_geo': geo.get('_status') == 'A'},
            'mag': mag_ctx.get('magnetometer') if mag_ctx else None,
            'sensors': calculate_sensor_offsets(lat, lon, rail_bearing, self.offsets),
            'valid': bool(lat and lon and azimuth)
        }

    def run(self):
        logger.info(f"⏳ Ожидание стартовых кадров (макс. {STARTUP_TIMEOUT_S} сек)...")
        start_time = time.time()
        fg = fm = None
        
        while time.time() - start_time < STARTUP_TIMEOUT_S and not self.stop_evt.is_set():
            if fg is None:
                try: fg = self.q_geo.get(timeout=0.5)
                except queue.Empty: pass
            if fm is None:
                try: fm = self.q_mag.get(timeout=0.5)
                except queue.Empty: pass
            if fg and fm: break
            time.sleep(0.1)

        if not fg or not fm:
            missing = []
            if not fg: missing.append("GEO (/dev/ttyUSB0)")
            if not fm: missing.append("MAG (/dev/ttyACM0)")
            logger.error(f"⛔ Таймаут ожидания кадров от: {', '.join(missing)}.")
            logger.error("💡 GPS может требовать время на поиск спутников. Проверьте антенну.")
            return

        self.mag_buffer.append(fm)
        logger.info("✅ Стартовые кадры получены. Синхронизация запущена...")

        while not self.stop_evt.is_set():
            try:
                geo = self.q_geo.get(timeout=0.05)
                t = geo['timestamp_sys']
                while not self.q_mag.empty():
                    try: self.mag_buffer.append(self.q_mag.get_nowait())
                    except queue.Empty: break

                ctx = self._get_mag_context(t)
                rec = self._build_record(geo, ctx, t)
                self.stats['merged'] += 1
                if ctx and ctx['timestamp_sys'] != t: self.stats['interpolated'] += 1

                self.callback(rec)
                now = time.time()
                if now - self.last_out >= self.throttle:
                    self._print_console(rec)
                    self.last_out = now
            except queue.Empty: continue
            except KeyboardInterrupt: break
            except Exception as e: logger.warning(f"Merger error: {e}")

        logger.info(f"Merger завершён: {self.stats}")

    def _print_console(self, rec):
        s = "✅" if rec['valid'] else "⚠️"
        lat = rec['gps'].get('latitude') or rec['geo'].get('latitude')
        lon = rec['gps'].get('longitude') or rec['geo'].get('longitude')
        # ✅ Исправлено: берём azimuth_true или course, а не ground_spe
        az = rec['gps'].get('azimuth_true') or rec['geo'].get('_course')
        d = rec['sync'].get('delta_ms', 0)
        
        ls = f"{lat:.5f}" if lat is not None else "------"
        os_ = f"{lon:.5f}" if lon is not None else "------"
        azs = f"{az:.1f}°" if az is not None else "---°"
        ds = f"{d:.1f}" if d is not None else "--"
        print(f"{s} #{self.stats['merged']:05d} t={rec['timestamp_sys']:.3f} | 📍 {ls},{os_} | 🧭 {azs} | Δ={ds}мс | сенсоров: {len(rec['sensors'])}", flush=True)

# ============================================================================
# 💾 ВЫВОД & CLI
# ============================================================================

class OutputHandler:
    def __init__(self, path=None):
        self.path, self.file, self.lock = path, None, threading.Lock()
        if path: self.file = open(path, 'w', encoding='utf-8', buffering=1); logger.info(f"JSONL: {path}")
    def write(self, rec):
        with self.lock:
            if self.file: self.file.write(json.dumps(rec, ensure_ascii=False) + '\n')
    def close(self):
        with self.lock:
            if self.file: self.file.close(); logger.info(f"Файл сохранён")

def parse_args():
    p = argparse.ArgumentParser(description='Dual Sniffer v1.5')
    p.add_argument('--geo-port', default='/dev/ttyUSB0')
    p.add_argument('--geo-baud', type=int, default=115200)
    p.add_argument('--mag-port', default='/dev/ttyACM0')
    p.add_argument('--mag-baud', type=int, default=9600)
    p.add_argument('--declination', '-d', type=float, default=0.0)
    p.add_argument('--offsets', type=str, default=','.join(map(str, DEFAULT_SENSOR_OFFSETS_M)))
    p.add_argument('--rail-angle', type=float, default=90.0,
                   help='Угол ориентации рейки относительно направления движения (90°=перпендикуляр, 0°=вдоль)')
    p.add_argument('--output', '-o', type=str, default=None)
    return p.parse_args()

def main():
    args = parse_args()
    logger.info(f"Dual Sniffer v1.5 | GEO:{args.geo_port}@{args.geo_baud} | MAG:{args.mag_port}@{args.mag_baud}")
    logger.info(f"Decl:{args.declination}° | Offsets:[{args.offsets}]м | RailAngle:{args.rail_angle}°")
    
    try: offsets = [float(x) for x in args.offsets.split(',')]
    except ValueError: logger.error("Неверный формат --offsets"); sys.exit(1)

    q_geo, q_mag = queue.Queue(1000), queue.Queue(100)
    stop = threading.Event()
    out = OutputHandler(args.output)

    w_geo = SerialWorker(args.geo_port, args.geo_baud, parse_plain_csv, q_geo, "GEO", stop)
    w_mag = SerialWorker(args.mag_port, args.mag_baud, lambda txt, ts: parse_nmea_extended(txt, ts, args.declination), q_mag, "MAG", stop)
    merger = StreamMerger(q_geo, q_mag, args.declination, offsets, args.rail_angle, out.write, stop)

    threads = [
        threading.Thread(target=w_geo.run, daemon=True),
        threading.Thread(target=w_mag.run, daemon=True),
        threading.Thread(target=merger.run)
    ]
    for t in threads[:-1]: t.start()
    
    try: merger.run()
    except KeyboardInterrupt: logger.info("Ctrl+C received")
    finally: stop.set(); time.sleep(0.3); out.close(); logger.info("Завершено.")

if __name__ == '__main__':
    main()