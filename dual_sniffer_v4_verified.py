#!/usr/bin/env python3
"""
Система для синхронного сбора данных с 4-датчиковой сенсорной косы (200 Гц) 
и GPS+компас (10 Гц) с расчётом точных координат каждого датчика в реальном времени.
1. GPS даёт координаты антенны (lat, lon)
2. Компас даёт магнитный азимут (azimuth_magnetic = atan2(-Y, X))
3. Склонение считается офлайн через WMM
4. azimuth_true = azimuth_magnetic + declination
5. Для каждого датчика:
   - Рассчитываем пеленг: bearing = azimuth_true + atan2(dy, dx)
   - Решаем прямую геодезическую задачу: (lat_sensor, lon_sensor) = GEOD.Direct(lat, lon, bearing, distance)
6. Результат: 4 точки косы, перпендикулярной направлению движения
"""

import argparse
import serial
import time
import json
import math
import struct
import sys
import threading
import queue
import logging
import os
from datetime import datetime, timezone
from typing import Optional, Dict, List, Any

# ============================================================================
# 🛠️ ЗАВИСИМОСТИ
# ============================================================================
try:
    from geographiclib.geodesic import Geodesic
    GEOD = Geodesic.WGS84
except ImportError:
    sys.exit("❌ Установите: pip install geographiclib")

try:
    from pygeomag import GeoMag
    # Инициализация без аргументов. Библиотека сама ищет WMM.COF в текущей папке.
    # Убедитесь, что файл WMM.COF лежит рядом со скриптом!
    if not os.path.exists("WMM.COF"):
        sys.exit("❌ Ошибка: Файл WMM.COF не найден!\n💡 Скачайте его (WMM2020) и положите в папку со скриптом.")
    GEO_MAG = GeoMag()
except ImportError:
    sys.exit("❌ Установите: pip install pygeomag")
except Exception as e:
    sys.exit(f"❌ Ошибка инициализации pygeomag: {e}")

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S')
logger = logging.getLogger('dual_sniffer_v4_verified')

DEFAULT_CONSOLE_THROTTLE_HZ = 1
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

def parse_nmea_utc_to_timestamp(time_str: str, date_str: str) -> Optional[float]:
    if not time_str or not date_str: return None
    try:
        hh = int(time_str[0:2])
        mm = int(time_str[2:4])
        ss = float(time_str[4:])
        dd = int(date_str[0:2])
        mo = int(date_str[2:4])
        yy = int(date_str[4:6]) + 2000
        dt = datetime(yy, mo, dd, hh, mm, ss, tzinfo=timezone.utc)
        return dt.timestamp()
    except Exception:
        return None

def get_declination(lat: float, lon: float, year: int = 2026) -> float:
    """
    Возвращает магнитное склонение через pygeomag (офлайн).
    Использует позиционные аргументы для совместимости.
    """
    try:
        # calculate(latitude, longitude, altitude, decimal_year)
        res = GEO_MAG.calculate(lat, lon, 0.0, float(year))
        return res.d  # declination in degrees
    except Exception as e:
        logger.warning(f"⚠️ Ошибка расчёта склонения: {e}. Использую 0.0")
        return 0.0

def build_dynamic_geometry(forward_offset: float, spacing: float) -> Dict[int, Dict]:
    """Генерирует матрицу смещений датчиков."""
    return {i: {'dx': forward_offset, 'dy': (i - 2.5) * spacing} for i in range(1, 5)}

def calculate_sensor_coords_geodetic(lat: float, lon: float, azimuth_true: float, 
                                     geometry: Dict[int, Dict]) -> List[Dict]:
    """Рассчитывает координаты датчиков на основе истинного азимута устройства."""
    if lat is None or lon is None or azimuth_true is None: return []
    results = []
    for sid, off in sorted(geometry.items()):
        dx, dy = off['dx'], off['dy']
        dist = math.hypot(dx, dy)
        # Угол смещения датчика относительно носа устройства
        bearing_offset = math.degrees(math.atan2(dy, dx))
        # Истинный пеленг на датчик = Азимут устройства + Смещение
        true_bearing = (azimuth_true + bearing_offset) % 360.0
        
        # Прямая геодезическая задача
        res = GEOD.Direct(lat, lon, true_bearing, dist)
        results.append({
            'id': sid, 
            'offset_m': {'dx': dx, 'dy': dy},
            'latitude': round(res['lat2'], 7), 
            'longitude': round(res['lon2'], 7),
            'distance_m': round(dist, 3), 
            'bearing_from_gps': round(true_bearing, 2)
        })
    return results

# ============================================================================
# 🔢 ПАРСЕРЫ
# ============================================================================

def parse_binary_frame(data: bytes, sys_time: float) -> Optional[Dict]:
    if len(data) != 23 or data[0] != 0x00 or data[3] != 0xFF or data[-4:] != b'\x78\x56\x34\x12':
        return None
    try:
        sensor_id = data[2]
        if sensor_id not in (1, 2, 3, 4): return None
        abs_val = struct.unpack('<i', data[7:11])[0] / 10.0
        grad_val = struct.unpack('<i', data[11:15])[0] / 10.0
        return {
            'timestamp_sys': sys_time, 'sensor_id': sensor_id,
            'amplitude_nt': abs_val, 'gradient_nt': grad_val,
            'filter_mode': data[6], 'length': struct.unpack('<H', data[15:17])[0],
            'checksum': struct.unpack('<H', data[17:19])[0], 'command': data[5]
        }
    except struct.error:
        return None

def parse_nmea_extended(line: str, sys_time: float, declination_override: Optional[float] = None) -> Optional[Dict]:
    line = line.strip('\r\n')
    if not line.startswith('$'): return None
    
    mag_data = None
    if ';' in line:
        nmea_part, mag_part = line.split(';', 1)
        if '*' in nmea_part: nmea_part = nmea_part.split('*')[0]
        try:
            vals = [float(v) for v in mag_part.split(',')[:3]]
            if len(vals) == 3:
                mag_data = {'x': vals[0], 'y': vals[1], 'z': vals[2],
                            'field_strength': math.sqrt(sum(v**2 for v in vals))}
        except ValueError: pass
        line = nmea_part
    elif '*' in line:
        line = line.split('*')[0]

    fields = line.split(',')
    if len(fields) < 12 or fields[0] not in ('$GNRMC', '$GPRMC'): return None
    
    try:
        res = {
            'timestamp_sys': sys_time,
            'latitude': dmm_to_decimal(fields[3], fields[4]),
            'longitude': dmm_to_decimal(fields[5], fields[6]),
            'speed_knots': float(fields[7]) if fields[7] else None,
            'course_true': float(fields[8]) if fields[8] else None,
            'status': fields[2], 'time_utc': fields[1], 'date_ddmmyy': fields[9]
        }
        res['gps_utc_ts'] = parse_nmea_utc_to_timestamp(fields[1], fields[9]) or sys_time
        
        if mag_data:
            res['magnetometer'] = mag_data
            
        # РАСЧЁТ СКЛОНЕНИЯ И АЗИМУТА
        if declination_override is not None:
            res['declination_deg'] = declination_override
        elif res.get('latitude') is not None and res.get('longitude') is not None:
            year = int(res['date_ddmmyy'][-2:]) + 2000 if res.get('date_ddmmyy') else 2026
            res['declination_deg'] = get_declination(res['latitude'], res['longitude'], year)
        else:
            res['declination_deg'] = 0.0
            
        if mag_data and 'x' in mag_data:
            # Магнитный азимут: угол вектора поля относительно оси X 
            # (направление противоположно разъему с проводами) устройства.

            # Почему минус тут -mag_data['y']: В стандарте NED (North-East-Down), 
            # который используют почти все GPS-компасы (включая BN-880/QMC5883L)
            # и современные библиотеки.
            # 1. Математика vs Навигация: В тригонометрии поворот «вправо» от оси 
            # X — это движение в сторону отрицательных значений углов (или переход к 270°). 
            # Навигация же требует, чтобы поворот вправо увеличивал угол с 0° до 90°.
            # 2. Тот самый «Минус»: В авиации и робототехнике (стандарт NED — North-East-Down) 
            # ось Y направлена направо. Чтобы получить навигационный угол
            # через стандартный арктангенс, используется именно формула atan2(-Y, X).
            az_mag = (math.degrees(math.atan2(-mag_data['y'], mag_data['x'])) + 360) % 360
            res['azimuth_magnetic'] = round(az_mag, 4)
            
            # Истинный азимут устройства = Магнитный + Склонение
            res['azimuth_true'] = round((az_mag + res['declination_deg'] + 360) % 360, 4)
        else:
            res['azimuth_magnetic'] = res.get('course_true')
            res['azimuth_true'] = res.get('course_true')
            
        res['valid'] = (res['status'] == 'A' and res.get('latitude') is not None)
        return res
    except Exception:
        return None

# ============================================================================
# 🧵 ВОРКЕРЫ
# ============================================================================

class BinarySensorReader:
    def __init__(self, port, baud, name):
        self.port, self.baud, self.name = port, baud, name
        self.lock = threading.Lock()
        self.latest = {1: None, 2: None, 3: None, 4: None}
        self.stop_evt = threading.Event()
        self.stats = {'read': 0, 'parsed': 0, 'errors': 0}
        self.buffer = b''

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2); self.ser.reset_input_buffer()
            logger.info(f"[{self.name}] Подключен к {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            logger.error(f"[{self.name}] Ошибка: {e}"); return False

    def run(self):
        delay = DEFAULT_RECONNECT_DELAY_S
        while not self.stop_evt.is_set():
            if not hasattr(self, 'ser') or not self.ser.is_open:
                if not self.connect(): time.sleep(delay); delay = min(delay*2, 30); continue
                delay = DEFAULT_RECONNECT_DELAY_S
            try:
                chunk = self.ser.read(512)
                if not chunk: continue
                self.buffer += chunk
                self.stats['read'] += len(chunk)

                while True:
                    marker_pos = self.buffer.find(b'\x78\x56\x34\x12')
                    if marker_pos == -1 or len(self.buffer) < marker_pos + 4:
                        break
                    
                    start_pos = marker_pos - 19
                    if start_pos < 0:
                        self.buffer = self.buffer[marker_pos + 4:]
                        continue

                    frame = self.buffer[start_pos : marker_pos + 4]
                    self.buffer = self.buffer[marker_pos + 4:]

                    if len(frame) == 23:
                        parsed = parse_binary_frame(frame, time.time())
                        if parsed and parsed['sensor_id'] in (1, 2, 3, 4):
                            self.stats['parsed'] += 1
                            with self.lock: self.latest[parsed['sensor_id']] = parsed
                        else: self.stats['errors'] += 1
            except serial.SerialException:
                logger.warning(f"[{self.name}] Разрыв...")
                if self.ser and self.ser.is_open: self.ser.close()
            except Exception as e:
                self.stats['errors'] += 1
                if self.stats['errors'] % 100 == 0: logger.warning(f"[{self.name}] Ошибка: {e}")
                
        if hasattr(self, 'ser') and self.ser.is_open: self.ser.close()
        logger.info(f"[{self.name}] Завершён. Байт: {self.stats['read']}, кадров: {self.stats['parsed']}")


class GPSReader:
    def __init__(self, port, baud, parser, out_queue, name, decl_override):
        self.port, self.baud, self.parser, self.out_q, self.name = port, baud, parser, out_queue, name
        self.decl_override = decl_override
        self.stop_evt = threading.Event()
        self.stats = {'read': 0, 'parsed': 0, 'errors': 0}

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2); self.ser.reset_input_buffer()
            logger.info(f"[{self.name}] Подключен к {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            logger.error(f"[{self.name}] Ошибка: {e}"); return False

    def run(self):
        delay = DEFAULT_RECONNECT_DELAY_S
        while not self.stop_evt.is_set():
            if not hasattr(self, 'ser') or not self.ser.is_open:
                if not self.connect(): time.sleep(delay); delay = min(delay*2, 30); continue
                delay = DEFAULT_RECONNECT_DELAY_S
            try:
                line = self.ser.readline()
                if not line: continue
                self.stats['read'] += 1
                parsed = self.parser(line.decode('utf-8', errors='ignore'), time.time(), self.decl_override)
                if parsed and parsed.get('valid'):
                    self.stats['parsed'] += 1
                    self.out_q.put(parsed)
                elif parsed: self.stats['parsed'] += 1
                else: self.stats['errors'] += 1
            except serial.SerialException:
                logger.warning(f"[{self.name}] Разрыв...")
                if self.ser and self.ser.is_open: self.ser.close()
            except Exception as e:
                self.stats['errors'] += 1
                if self.stats['errors'] % 100 == 0: logger.warning(f"[{self.name}] Ошибка: {e}")
        if hasattr(self, 'ser') and self.ser.is_open: self.ser.close()
        logger.info(f"[{self.name}] Завершён: {self.stats}")

# ============================================================================
# 📦 ПАКЕТНЫЙ ПРОЦЕССОР
# ============================================================================

class PacketProcessor:
    def __init__(self, sensor_reader, geometry, output_handler):
        self.sensors = sensor_reader
        self.geometry = geometry
        self.out = output_handler
        self.stats = {'packets': 0}
        self.last_console = 0

    def process_packet(self, gps_data):
        t_utc = gps_data['gps_utc_ts']
        t_sys = gps_data['timestamp_sys']
        lat = gps_data.get('latitude')
        lon = gps_data.get('longitude')
        
        # Используем истинный азимут устройства (от компаса) для ориентации косы
        az_true = gps_data.get('azimuth_true')
        
        with self.sensors.lock:
            sensor_readings = [self.sensors.latest[sid] for sid in [1,2,3,4]]
        
        # Рассчитываем координаты датчиков, поворачивая их на az_true
        sensors_geo = calculate_sensor_coords_geodetic(lat, lon, az_true, self.geometry)
        
        sensors_out = []
        for geo, reading in zip(sensors_geo, sensor_readings):
            entry = geo.copy()
            if reading:
                entry.update({'amplitude_nt': reading['amplitude_nt'], 
                              'gradient_nt': reading['gradient_nt'],
                              'timestamp_sys': reading['timestamp_sys']})
            else:
                entry.update({'amplitude_nt': None, 'gradient_nt': None, 'timestamp_sys': None})
            sensors_out.append(entry)
            
        record = {
            'timestamp_utc': t_utc,
            'timestamp_iso': datetime.fromtimestamp(t_utc, tz=timezone.utc).isoformat(),
            'sync': {
                'system_latency_ms': round((t_sys - t_utc) * 1000, 2) if t_sys and t_utc else None
            },
            'gps': {
                'latitude': lat, 'longitude': lon,
                'azimuth_magnetic': gps_data.get('azimuth_magnetic'),
                'azimuth_true': az_true,
                'declination_deg': gps_data.get('declination_deg'),
                'course_true': gps_data.get('course_true'),
                'valid': gps_data.get('valid', False)
            },
            'geometry_params': {k: v for k, v in self.geometry.items()},
            'sensors': sensors_out,
            'valid': bool(lat and lon and az_true and all(s.get('amplitude_nt') is not None for s in sensors_out))
        }

        self.stats['packets'] += 1
        self.out.write(record)

        now = time.time()
        if now - self.last_console >= 1.0/DEFAULT_CONSOLE_THROTTLE_HZ:
            self._print_console(record)
            self.last_console = now

    def _print_console(self, rec):
        s = "✅" if rec['valid'] else "⚠️"
        lat = rec['gps'].get('latitude')
        lon = rec['gps'].get('longitude')
        az = rec['gps'].get('azimuth_true')
        decl = rec['gps'].get('declination_deg')
        ls = f"{lat:.5f}" if lat is not None else "------"
        os_ = f"{lon:.5f}" if lon is not None else "------"
        azs = f"{az:.1f}°" if az is not None else "---°"
        decls = f"{decl:.1f}°" if decl is not None else "---°"
        print(f"{s} #{self.stats['packets']:05d} UTC={rec['timestamp_utc']:.3f} | 📍 {ls},{os_} | 🧭 {azs} | Decl: {decls} | датчиков: {len(rec['sensors'])}", flush=True)

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
    p = argparse.ArgumentParser(description='Dual Sniffer v4 Verified (pygeomag)')
    p.add_argument('--sensor-port', default='/dev/ttyUSB0')
    p.add_argument('--sensor-baud', type=int, default=115200)
    p.add_argument('--gps-port', default='/dev/ttyACM0')
    p.add_argument('--gps-baud', type=int, default=9600)
    p.add_argument('--forward-offset', type=float, default=1.0)
    p.add_argument('--spacing', type=float, default=0.3)
    p.add_argument('--declination-override', type=float, default=None)
    p.add_argument('--output', '-o', type=str, default=None)
    return p.parse_args()

def main():
    args = parse_args()
    logger.info(f"Dual Sniffer v4 Verified | Sensors:{args.sensor_port}@{args.sensor_baud} | GPS:{args.gps_port}@{args.gps_baud}")
    logger.info(f"Geometry: forward={args.forward_offset}m, spacing={args.spacing}m")
    logger.info("⏱️ Синхронизация: спутниковое UTC (NMEA) | Магнитное склонение: pygeomag (WMM2020)")
    
    geometry = build_dynamic_geometry(args.forward_offset, args.spacing)
    gps_q = queue.Queue(maxsize=10)
    out = OutputHandler(args.output)

    sensor_reader = BinarySensorReader(args.sensor_port, args.sensor_baud, "SENSORS(100Hz)")
    gps_reader = GPSReader(args.gps_port, args.gps_baud, parse_nmea_extended, gps_q, "GPS(10Hz)", args.declination_override)
    processor = PacketProcessor(sensor_reader, geometry, out)

    t_sensor = threading.Thread(target=sensor_reader.run, daemon=True)
    t_gps = threading.Thread(target=gps_reader.run, daemon=True)
    t_sensor.start()
    t_gps.start()

    logger.info("⏳ Ожидание первого валидного GPS-кадра...")
    try:
        while True:
            gps_data = gps_q.get(timeout=STARTUP_TIMEOUT_S)
            processor.process_packet(gps_data)
    except queue.Empty:
        logger.error("Таймаут ожидания GPS.")
    except KeyboardInterrupt:
        logger.info("Ctrl+C received")
    finally:
        sensor_reader.stop_evt.set()
        gps_reader.stop_evt.set()
        time.sleep(0.3)
        out.close()
        logger.info(f"Завершено. Пакетов: {processor.stats['packets']}")

if __name__ == '__main__':
    main()