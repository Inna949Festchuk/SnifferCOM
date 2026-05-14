#!/usr/bin/env python3
"""
Система для синхронного сбора данных с N-датчиковой сенсорной косы (200 Гц) 
и GPS+компас (10 Гц) с расчётом точных координат каждого датчика в реальном времени.

Принцип работы:
1. GPS даёт координаты антенны (lat, lon)
2. Компас даёт магнитный азимут (azimuth_magnetic = atan2(-Y, X))
3. Склонение вычисляется офлайн через WMM
4. azimuth_true = azimuth_magnetic + declination
5. Для каждого датчика:
   - Рассчитываем пеленг: bearing = azimuth_true + atan2(dy, dx)
   - Решаем прямую геодезическую задачу: (lat_sensor, lon_sensor) = GEOD.Direct(lat, lon, bearing, distance)
6. Результат: N точек косы, перпендикулярной направлению движения

Использование (более подробно см. README.md):
#  Терминал 1 — сбор данных (пример для Windows)
python importercom.py --sensor-port COM4 --gps-port COM5 --num-sensors 4 --output field_data.jsonl

#  Терминал 1 — сбор данных (пример для Linux)
python importercom.py --sensor-port /dev/ttyUSB0 --gps-port /dev/ttyACM0 --num-sensors 4 --output field_data.jsonl

# Терминал 2 — сервер карты
python3 serve_map.py

# Открыть Браузер
# Ввести http://localhost:8000/map.html
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
from typing import Optional, Dict, List, Any, Tuple

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
logger = logging.getLogger('importercom')

DEFAULT_CONSOLE_THROTTLE_HZ = 1
DEFAULT_RECONNECT_DELAY_S = 1.0
STARTUP_TIMEOUT_S = 60.0
FRAME_LENGTH = 23  # Длина бинарного кадра датчика в байтах
END_MARKER = b'\x78\x56\x34\x12'  # Маркер конца кадра


# ============================================================================
# 🌍 ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================================

def dmm_to_decimal(dmm_str: str, direction: str) -> Optional[float]:
    """
    Преобразует координаты из формата DMM (градусы/минуты) в десятичные градусы.
    
    Формат NMEA: ddmm.mmmm, где dd - градусы, mm.mmmm - минуты.
    
    Args:
        dmm_str: Строка с координатами в формате DMM
        direction: Направление ('N', 'S', 'E', 'W')
    
    Returns:
        Координата в десятичных градусах или None при ошибке
    """
    if not dmm_str:
        return None
    try:
        dmm = float(dmm_str)
        if dmm == 0.0:
            return 0.0
        degrees = int(dmm / 100)
        minutes = dmm - degrees * 100
        decimal = degrees + minutes / 60.0
        # Отрицательные значения для южной широты и западной долготы
        return -decimal if direction.upper() in ['S', 'W'] else decimal
    except (ValueError, TypeError):
        return None


def parse_nmea_utc_to_timestamp(time_str: str, date_str: str) -> Optional[float]:
    """
    Преобразует UTC время и дату из NMEA строки в Unix timestamp.
    
    Args:
        time_str: Время в формате HHMMSS.SS
        date_str: Дата в формате DDMMYY
    
    Returns:
        Unix timestamp (секунды с 1970-01-01) или None при ошибке
    """
    if not time_str or not date_str:
        return None
    try:
        hh = int(time_str[0:2])
        mm = int(time_str[2:4])
        ss = float(time_str[4:])
        dd = int(date_str[0:2])
        mo = int(date_str[2:4])
        yy = int(date_str[4:6]) + 2000
        dt = datetime(yy, mo, dd, hh, mm, int(ss), 
                      int((ss % 1) * 1000000), tzinfo=timezone.utc)
        return dt.timestamp()
    except Exception:
        return None


def get_declination(lat: float, lon: float, year: int = 2026) -> float:
    """
    Возвращает магнитное склонение через pygeomag (офлайн).
    
    Магнитное склонение — это угол между истинным севером и магнитным севером.
    Положительное склонение означает, что магнитный север находится к востоку
    от истинного севера.
    
    Args:
        lat: Широта в десятичных градусах
        lon: Долгота в десятичных градусах
        year: Год для модели WMM (по умолчанию 2026)
    
    Returns:
        Магнитное склонение в градусах (восточное положительное)
    """
    try:
        # calculate(latitude, longitude, altitude, decimal_year)
        res = GEO_MAG.calculate(lat, lon, 0.0, float(year))
        return res.d  # declination in degrees
    except Exception as e:
        logger.warning(f"⚠️ Ошибка расчёта склонения: {e}. Использую 0.0")
        return 0.0


def build_dynamic_geometry(forward_offset: float, spacing: float, num_sensors: int) -> Dict[int, Dict[str, float]]:
    """
    Генерирует матрицу смещений датчиков для заданного количества.
    
    Геометрия косы:
    - Датчики расположены в линию, перпендикулярную направлению движения
    - Центр косы находится между датчиками (симметричное расположение)
    - Все датчики имеют одинаковое смещение вперёд (forward_offset)
    
    Args:
        forward_offset: Смещение всех датчиков вперёд от GPS-антенны (метры)
        spacing: Расстояние между датчиками (метры)
        num_sensors: Количество датчиков (1-N)
    
    Returns:
        Словарь {id_датчика: {'dx': смещение_вперёд, 'dy': поперечное_смещение}}
    """
    # Центр косы находится между датчиками
    # Для 4 датчиков: позиции -1.5, -0.5, 0.5, 1.5 (при spacing=0.3 даёт -0.45, -0.15, 0.15, 0.45)
    center_offset = (num_sensors - 1) / 2
    geometry = {}
    for i in range(1, num_sensors + 1):
        # i = 1,2,3,4 для 4 датчиков
        # (i - 1) даёт 0,1,2,3
        # (i - 1 - center_offset) даёт -1.5, -0.5, 0.5, 1.5
        geometry[i] = {
            'dx': forward_offset,  # Все датчики на одном расстоянии вперёд
            'dy': (i - 1 - center_offset) * spacing
        }
    return geometry


def calculate_sensor_coords_geodetic(lat: float, lon: float, azimuth_true: float, 
                                     geometry: Dict[int, Dict[str, float]]) -> List[Dict[str, Any]]:
    """
    Рассчитывает географические координаты датчиков на основе истинного азимута устройства.
    
    Алгоритм:
    1. Для каждого датчика вычисляем расстояние до GPS-антенны: dist = hypot(dx, dy)
    2. Вычисляем угол смещения датчика относительно носа устройства: bearing_offset = atan2(dy, dx)
    3. Истинный пеленг на датчик = Азимут устройства + bearing_offset
    4. Решаем прямую геодезическую задачу: по точке (lat, lon), пеленгу и расстоянию
    
    Args:
        lat: Широта GPS-антенны (десятичные градусы)
        lon: Долгота GPS-антенны (десятичные градусы)
        azimuth_true: Истинный азимут направления движения устройства (градусы от севера)
        geometry: Словарь с геометрией датчиков {id: {'dx': float, 'dy': float}}
    
    Returns:
        Список словарей с координатами каждого датчика
    """
    if lat is None or lon is None or azimuth_true is None:
        return []
    
    results = []
    for sid, off in sorted(geometry.items()):
        dx = off['dx']
        dy = off['dy']
        dist = math.hypot(dx, dy)  # Евклидово расстояние
        
        # Угол смещения датчика относительно носа устройства (0° = прямо вперёд)
        # atan2(dy, dx): положительный dy = влево (в навигации угол увеличивается влево)
        bearing_offset = math.degrees(math.atan2(dy, dx))
        
        # Истинный пеленг на датчик = Азимут устройства + Смещение
        true_bearing = (azimuth_true + bearing_offset) % 360.0
        
        # Прямая геодезическая задача (Vincenty's formulae)
        # Вычисляем координаты датчика по известным координатам антенны,
        # пеленгу и расстоянию
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

def parse_binary_frame(frame: bytes, sys_time: float) -> Optional[Dict[str, Any]]:
    """
    Парсит бинарный кадр от датчика (фиксированная длина 23 байта).
    
    Формат кадра (23 байта):
    - байт 0: 0x00 (маркер начала?)
    - байт 1: номер команды
    - байт 2: ID датчика (0x01..0x04)
    - байт 3: 0xFF (маркер?)
    - байт 4: резерв
    - байт 5: команда
    - байт 6: режим фильтра
    - байты 7-10: амплитуда (int32, младшие байты первыми, делённая на 10)
    - байты 11-14: градиент (int32, младшие байты первыми, делённая на 10)
    - байты 15-16: длина (uint16)
    - байты 17-18: контрольная сумма (uint16)
    - байты 19-22: маркер конца (0x78, 0x56, 0x34, 0x12)
    
    Args:
        frame: Бинарные данные кадра (должны быть ровно 23 байта)
        sys_time: Системное время получения кадра (Unix timestamp)
    
    Returns:
        Словарь с распарсенными данными или None при ошибке
    """
    # Проверка длины и маркеров
    if (len(frame) != FRAME_LENGTH or
        frame[0] != 0x00 or
        frame[3] != 0xFF or
        frame[-4:] != END_MARKER):
        return None
    
    try:
        # Байт 2 (индекс 2) — адрес отправителя (0x01, 0x02, 0x03, 0x04)
        sensor_id = frame[2]
        
        # Проверяем, что номер датчика в допустимом диапазоне (1-254)
        if sensor_id not in range(1, 255):
            return None
        
        # Распаковка амплитуды (int32, little-endian) с делением на 10
        # Устройство передаёт значение в десятых долях нТл
        amplitude_nt = struct.unpack('<i', frame[7:11])[0] / 10.0
        
        # Распаковка градиента (int32, little-endian) с делением на 10
        gradient_nt = struct.unpack('<i', frame[11:15])[0] / 10.0
        
        return {
            'timestamp_sys': sys_time,
            'sensor_id': sensor_id,
            'amplitude_nt': amplitude_nt,
            'gradient_nt': gradient_nt,
            'filter_mode': frame[6],
            'length': struct.unpack('<H', frame[15:17])[0],
            'checksum': struct.unpack('<H', frame[17:19])[0],
            'command': frame[5]
        }
    except (struct.error, IndexError) as e:
        logger.debug(f"Ошибка распаковки кадра: {e}")
        return None


def parse_nmea_extended(line: str, sys_time: float, 
                        declination_override: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """
    Парсит NMEA строку GPS с возможным добавлением магнитометра.
    
    Поддерживаемые форматы:
    - Стандартный NMEA: $GNRMC,...
    - Расширенный с магнитометром: $GNRMC,...;X,Y,Z
    (где X,Y,Z - показания магнитометра в мкТл)
    
    Магнитный азимут вычисляется по формуле: azimuth = atan2(-Y, X)
    Минус перед Y возникает из-за стандарта NED (North-East-Down):
    в навигации угол увеличивается при повороте вправо, а atan2(Y,X) даёт
    увеличение влево, поэтому используется -Y.
    
    Args:
        line: Строка для парсинга
        sys_time: Системное время получения строки
        declination_override: Принудительное значение склонения (если не None)
    
    Returns:
        Словарь с распарсенными данными или None при ошибке
    """
    line = line.strip('\r\n')
    if not line.startswith('$'):
        return None
    
    mag_data = None
    
    # Проверка на расширенный формат (NMEA + магнитометр через ";")
    if ';' in line:
        nmea_part, mag_part = line.split(';', 1)
        # Удаляем контрольную сумму, если есть
        if '*' in nmea_part:
            nmea_part = nmea_part.split('*')[0]
        try:
            vals = [float(v) for v in mag_part.split(',')[:3]]
            if len(vals) == 3:
                mag_data = {
                    'x': vals[0], 
                    'y': vals[1], 
                    'z': vals[2],
                    'field_strength': math.sqrt(sum(v**2 for v in vals))
                }
        except ValueError:
            pass
        line = nmea_part
    elif '*' in line:
        # Стандартный NMEA с контрольной суммой
        line = line.split('*')[0]
    
    fields = line.split(',')
    # Проверяем, что это RMC сообщение (рекомендованный минимум) и достаточно полей
    if len(fields) < 12 or fields[0] not in ('$GNRMC', '$GPRMC'):
        return None
    
    try:
        res = {
            'timestamp_sys': sys_time,
            'latitude': dmm_to_decimal(fields[3], fields[4]),
            'longitude': dmm_to_decimal(fields[5], fields[6]),
            'speed_knots': float(fields[7]) if fields[7] else None,
            'course_true': float(fields[8]) if fields[8] else None,
            'status': fields[2],
            'time_utc': fields[1],
            'date_ddmmyy': fields[9]
        }
        res['gps_utc_ts'] = parse_nmea_utc_to_timestamp(fields[1], fields[9]) or sys_time
        
        if mag_data:
            res['magnetometer'] = mag_data
        
        # РАСЧЁТ СКЛОНЕНИЯ
        if declination_override is not None:
            res['declination_deg'] = declination_override
        elif res.get('latitude') is not None and res.get('longitude') is not None:
            year = int(res['date_ddmmyy'][-2:]) + 2000 if res.get('date_ddmmyy') else 2026
            res['declination_deg'] = get_declination(res['latitude'], res['longitude'], year)
        else:
            res['declination_deg'] = 0.0
        
        # РАСЧЁТ АЗИМУТА
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
            
            # Истинный азимут = Магнитный азимут + Склонение
            # Восточное склонение прибавляется, западное вычитается
            res['azimuth_true'] = round((az_mag + res['declination_deg']) % 360, 4)
        else:
            # Если данных магнитометра нет, используем курс от GPS (если есть)
            res['azimuth_magnetic'] = res.get('course_true')
            res['azimuth_true'] = res.get('course_true')
        
        # Валидность: статус 'A' (активный) и координаты не None
        res['valid'] = (res['status'] == 'A' and res.get('latitude') is not None)
        return res
    except Exception as e:
        logger.debug(f"Ошибка парсинга NMEA: {e}")
        return None


# ============================================================================
# 🧵 ПОТОКИ ДЛЯ ЧТЕНИЯ СЕНСОРОВ И GPS
# ============================================================================

class BinarySensorReader:
    """
    Потоковый читатель бинарных данных с датчиков.
    
    Поддерживает автоматическое переподключение при обрыве связи.
    Данные буферизируются для поиска целых кадров.
    """
    
    def __init__(self, port: str, baud: int, name: str, num_sensors: int):
        """
        Args:
            port: Последовательный порт (например, 'COM4' или '/dev/ttyUSB0')
            baud: Скорость передачи (обычно 115200)
            name: Имя для логирования
            num_sensors: Ожидаемое количество датчиков
        """
        self.port = port
        self.baud = baud
        self.name = name
        self.num_sensors = num_sensors
        self.lock = threading.Lock()
        self.latest = {i: None for i in range(1, num_sensors + 1)}
        self.stop_evt = threading.Event()
        self.stats = {'read': 0, 'parsed': 0, 'errors': 0}
        self.buffer = b''  # Буфер для накопления данных между чтениями
        self.ser = None
    
    def connect(self) -> bool:
        """Устанавливает соединение с последовательным портом."""
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
        """Основной цикл чтения и обработки данных."""
        delay = DEFAULT_RECONNECT_DELAY_S
        
        while not self.stop_evt.is_set():
            # Проверка соединения
            if self.ser is None or not self.ser.is_open:
                if not self.connect():
                    time.sleep(delay)
                    delay = min(delay * 2, 30)
                    continue
                delay = DEFAULT_RECONNECT_DELAY_S
            
            try:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                
                self.buffer += chunk
                self.stats['read'] += len(chunk)
                
                # Поиск полных кадров в буфере
                while True:
                    # Ищем маркер конца кадра
                    end_pos = self.buffer.find(END_MARKER)
                    if end_pos == -1 or len(self.buffer) < end_pos + 4:
                        # Нет маркера или недостаточно данных для завершения кадра
                        break
                    
                    # Вычисляем начало кадра: отступаем на (FRAME_LENGTH - 4) байт назад
                    # Так как маркер занимает 4 байта в конце кадра
                    start_pos = end_pos - (FRAME_LENGTH - 4)
                    
                    if start_pos < 0:
                        # Недостаточно данных для полного кадра,
                        # удаляем найденный маркер и продолжаем поиск
                        self.buffer = self.buffer[end_pos + 4:]
                        continue
                    
                    # Извлекаем полный кадр
                    frame = self.buffer[start_pos:end_pos + 4]
                    
                    # Удаляем обработанные данные из буфера
                    self.buffer = self.buffer[end_pos + 4:]
                    
                    # Парсим кадр
                    parsed = parse_binary_frame(frame, time.time())
                    if parsed and 1 <= parsed['sensor_id'] <= self.num_sensors:
                        self.stats['parsed'] += 1
                        with self.lock:
                            self.latest[parsed['sensor_id']] = parsed
                    else:
                        self.stats['errors'] += 1
                        
            except serial.SerialException:
                logger.warning(f"[{self.name}] Обрыв связи. Переподключение...")
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                self.stats['errors'] += 1
                if self.stats['errors'] % 100 == 0:
                    logger.warning(f"[{self.name}] Ошибка: {e}")
        
        # Завершение работы
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info(f"[{self.name}] Завершён. Байт: {self.stats['read']}, "
                   f"кадров: {self.stats['parsed']}, ошибок: {self.stats['errors']}")


class GPSReader:
    """Потоковый читатель NMEA данных GPS."""
    
    def __init__(self, port: str, baud: int, parser_func, out_queue: queue.Queue, 
                 name: str, decl_override: Optional[float] = None):
        """
        Args:
            port: Последовательный порт GPS
            baud: Скорость передачи (обычно 9600)
            parser_func: Функция парсинга строки
            out_queue: Очередь для отправки распарсенных данных
            name: Имя для логирования
            decl_override: Принудительное значение склонения (если не None)
        """
        self.port = port
        self.baud = baud
        self.parser = parser_func
        self.out_q = out_queue
        self.name = name
        self.decl_override = decl_override
        self.stop_evt = threading.Event()
        self.stats = {'read': 0, 'parsed': 0, 'errors': 0}
        self.ser = None
    
    def connect(self) -> bool:
        """Устанавливает соединение с последовательным портом."""
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
        """Основной цикл чтения NMEA строк."""
        delay = DEFAULT_RECONNECT_DELAY_S
        
        while not self.stop_evt.is_set():
            # Проверка соединения
            if self.ser is None or not self.ser.is_open:
                if not self.connect():
                    time.sleep(delay)
                    delay = min(delay * 2, 30)
                    continue
                delay = DEFAULT_RECONNECT_DELAY_S
            
            try:
                line = self.ser.readline()
                if not line:
                    continue
                
                self.stats['read'] += 1
                parsed = self.parser(line.decode('utf-8', errors='ignore'), 
                                     time.time(), self.decl_override)
                
                if parsed:
                    self.stats['parsed'] += 1
                    if parsed.get('valid'):
                        # Отправляем только валидные GPS-данные в очередь
                        self.out_q.put(parsed)
                else:
                    self.stats['errors'] += 1
                    
            except serial.SerialException:
                logger.warning(f"[{self.name}] Обрыв связи. Переподключение...")
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                self.stats['errors'] += 1
                if self.stats['errors'] % 100 == 0:
                    logger.warning(f"[{self.name}] Ошибка: {e}")
        
        # Завершение работы
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info(f"[{self.name}] Завершён. Строк: {self.stats['read']}, "
                   f"распарсено: {self.stats['parsed']}, ошибок: {self.stats['errors']}")


# ============================================================================
# 📦 ПАКЕТНЫЙ ПРОЦЕССОР
# ============================================================================

class PacketProcessor:
    """
    Обработчик синхронизированных данных с датчиков и GPS.
    
    Для каждого полученного GPS-кадра:
    1. Берёт актуальные показания датчиков (не старше 100 мс)
    2. Рассчитывает координаты каждого датчика
    3. Формирует выходную запись
    """
    
    def __init__(self, sensor_reader: BinarySensorReader, geometry: Dict[int, Dict[str, float]],
                 output_handler: 'OutputHandler', max_sensor_age_ms: float = 100.0):
        """
        Args:
            sensor_reader: Объект для чтения данных датчиков
            geometry: Геометрия расположения датчиков
            output_handler: Обработчик вывода данных
            max_sensor_age_ms: Максимальный возраст данных датчика (мс)
        """
        self.sensors = sensor_reader
        self.geometry = geometry
        self.out = output_handler
        self.max_sensor_age = max_sensor_age_ms / 1000.0  # Перевод в секунды
        self.stats = {'packets': 0, 'valid_packets': 0}
        self.last_console = 0.0
    
    def process_packet(self, gps_data: Dict[str, Any]):
        """
        Обрабатывает один GPS-кадр и создаёт запись с данными датчиков.
        
        Args:
            gps_data: Распарсенные данные GPS
        """
        t_utc = gps_data.get('gps_utc_ts')
        t_sys = gps_data.get('timestamp_sys')
        lat = gps_data.get('latitude')
        lon = gps_data.get('longitude')
        az_true = gps_data.get('azimuth_true')
        
        # Получение актуальных показаний датчиков
        with self.sensors.lock:
            sensor_readings = []
            for sid in range(1, self.sensors.num_sensors + 1):
                reading = self.sensors.latest.get(sid)
                # Проверяем свежесть данных: не старше max_sensor_age
                if reading and (t_sys - reading['timestamp_sys']) <= self.max_sensor_age:
                    sensor_readings.append(reading)
                else:
                    sensor_readings.append(None)
        
        # Расчёт координат датчиков
        sensors_geo = calculate_sensor_coords_geodetic(lat, lon, az_true, self.geometry)
        
        # Объединение географических данных с показаниями датчиков
        sensors_out = []
        for geo, reading in zip(sensors_geo, sensor_readings):
            entry = geo.copy()
            if reading:
                entry.update({
                    'amplitude_nt': reading['amplitude_nt'],
                    'gradient_nt': reading['gradient_nt'],
                    'timestamp_sys': reading['timestamp_sys']
                })
            else:
                entry.update({
                    'amplitude_nt': None,
                    'gradient_nt': None,
                    'timestamp_sys': None
                })
            sensors_out.append(entry)
        
        # Проверка валидности записи: есть GPS, азимут и все датчики
        all_sensors_valid = all(s.get('amplitude_nt') is not None for s in sensors_out)
        is_valid = bool(lat is not None and lon is not None and az_true is not None and all_sensors_valid)
        
        # Формирование выходной записи
        record = {
            'timestamp_utc': t_utc,
            'timestamp_iso': datetime.fromtimestamp(t_utc, tz=timezone.utc).isoformat() if t_utc else None,
            'sync': {
                'system_latency_ms': round((t_sys - t_utc) * 1000, 2) if t_sys and t_utc else None
            },
            'gps': {
                'latitude': lat,
                'longitude': lon,
                'azimuth_magnetic': gps_data.get('azimuth_magnetic'),
                'azimuth_true': az_true,
                'declination_deg': gps_data.get('declination_deg'),
                'course_true': gps_data.get('course_true'),
                'valid': gps_data.get('valid', False)
            },
            'geometry_params': {k: v for k, v in self.geometry.items()},
            'sensors': sensors_out,
            'valid': is_valid
        }
        
        self.stats['packets'] += 1
        if is_valid:
            self.stats['valid_packets'] += 1
        self.out.write(record)
        
        # Вывод на консоль с ограничением частоты
        now = time.time()
        if now - self.last_console >= 1.0 / DEFAULT_CONSOLE_THROTTLE_HZ:
            self._print_console(record)
            self.last_console = now
    
    def _print_console(self, rec: Dict[str, Any]):
        """Выводит краткую информацию о пакете в консоль."""
        status = "✅" if rec['valid'] else "⚠️"
        lat = rec['gps'].get('latitude')
        lon = rec['gps'].get('longitude')
        az = rec['gps'].get('azimuth_true')
        decl = rec['gps'].get('declination_deg')
        
        lat_str = f"{lat:.7f}" if lat is not None else "---"
        lon_str = f"{lon:.7f}" if lon is not None else "---"
        az_str = f"{az:.1f}°" if az is not None else "---°"
        decl_str = f"{decl:.1f}°" if decl is not None else "---°"
        
        # Сбор показаний амплитуд и градиентов всех датчиков
        amplitudes = []
        gradients = []
        for sensor in rec['sensors']:
            amp = sensor.get('amplitude_nt')
            grad = sensor.get('gradient_nt')
            amplitudes.append(f"{amp:.1f}" if amp is not None else "---")
            gradients.append(f"{grad:.1f}" if grad is not None else "---")
        
        amp_output = " | ".join(amplitudes)
        grad_output = " | ".join(gradients)
        
        ts = rec['timestamp_utc']
        ts_str = f"{ts:.3f}" if ts is not None else "---"
        
        print(f"{status} #{self.stats['packets']:05d} UTC={ts_str} "
              f"| 📍 {lat_str},{lon_str} | 🧭 {az_str} | Decl: {decl_str} "
              f"| датчиков: {len(rec['sensors'])} "
              f"| амплитуда (нТ): {amp_output} "
              f"| градиент (нТ/м): {grad_output}")


# ============================================================================
# 💾 ВЫВОД ДАННЫХ
# ============================================================================

class OutputHandler:
    """Обработчик записи данных в JSONL файл."""
    
    def __init__(self, path: Optional[str] = None):
        """
        Args:
            path: Путь к выходному файлу (если None, вывод не производится)
        """
        self.path = path
        self.file = None
        self.lock = threading.Lock()
        if path:
            self.file = open(path, 'w', encoding='utf-8', buffering=1)
            logger.info(f"Запись в JSONL файл: {path}")
    
    def write(self, record: Dict[str, Any]):
        """Записывает одну запись в файл в формате JSONL."""
        with self.lock:
            if self.file:
                self.file.write(json.dumps(record, ensure_ascii=False) + '\n')
    
    def close(self):
        """Закрывает выходной файл."""
        with self.lock:
            if self.file:
                self.file.close()
                logger.info("Файл сохранён")


# ============================================================================
# 🚀 ОСНОВНАЯ ФУНКЦИЯ
# ============================================================================

def parse_args():
    """Разбор аргументов командной строки."""
    parser = argparse.ArgumentParser(
        description='Dual Sniffer v5 - универсальная система сбора данных сенсорной косы',
        epilog='Пример: python importercom.py --sensor-port COM4 --gps-port COM5 --num-sensors 4 --output data.jsonl'
    )
    parser.add_argument('--sensor-port', type=str, default='/dev/ttyUSB0',
                        help='Последовательный порт для датчиков (по умолчанию: /dev/ttyUSB0)')
    parser.add_argument('--sensor-baud', type=int, default=115200,
                        help='Скорость порта датчиков (по умолчанию: 115200)')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyACM0',
                        help='Последовательный порт для GPS (по умолчанию: /dev/ttyACM0)')
    parser.add_argument('--gps-baud', type=int, default=9600,
                        help='Скорость порта GPS (по умолчанию: 9600)')
    parser.add_argument('--forward-offset', type=float, default=1.0,
                        help='Смещение датчиков вперёд от GPS-антенны в метрах (по умолчанию: 1.0)')
    parser.add_argument('--spacing', type=float, default=0.3,
                        help='Расстояние между датчиками в метрах (по умолчанию: 0.3)')
    parser.add_argument('--num-sensors', type=int, default=4,
                        help='Количество датчиков в косе (1-254, по умолчанию: 4)')
    parser.add_argument('--declination-override', type=float, default=None,
                        help='Принудительное значение магнитного склонения в градусах (отключает WMM)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Путь к выходному JSONL файлу (опционально)')
    parser.add_argument('--max-sensor-age', type=float, default=100.0,
                        help='Максимальный возраст данных датчика в мс (по умолчанию: 100)')
    return parser.parse_args()


def main():
    """Главная функция программы."""
    args = parse_args()
    
    logger.info("=" * 60)
    logger.info("DUAL SNIFFER v5 - Универсальная система сбора данных сенсорной косы")
    logger.info("=" * 60)
    logger.info(f"Датчики:      {args.sensor_port} @ {args.sensor_baud} бод")
    logger.info(f"GPS:          {args.gps_port} @ {args.gps_baud} бод")
    logger.info(f"Геометрия:    вперед={args.forward_offset}м, шаг={args.spacing}м, датчиков={args.num_sensors}")
    logger.info(f"Синхронизация: спутниковое UTC (NMEA) | Магнитное склонение: "
                f"{'фиксированное ' + str(args.declination_override) + '°' if args.declination_override else 'WMM (офлайн)'}")
    logger.info("=" * 60)
    
    # Построение геометрии датчиков
    geometry = build_dynamic_geometry(args.forward_offset, args.spacing, args.num_sensors)
    logger.info(f"Смещения датчиков (dx, dy):")
    for sid, offsets in geometry.items():
        logger.info(f"  Датчик {sid}: dx={offsets['dx']:.2f}м, dy={offsets['dy']:.3f}м")
    
    # Создание очереди и обработчиков
    gps_queue = queue.Queue(maxsize=10)
    output_handler = OutputHandler(args.output)
    
    # Создание читателей
    sensor_reader = BinarySensorReader(
        args.sensor_port, args.sensor_baud, 
        "SENSORS", args.num_sensors
    )
    gps_reader = GPSReader(
        args.gps_port, args.gps_baud, 
        parse_nmea_extended, gps_queue, 
        "GPS", args.declination_override
    )
    
    # Создание процессора
    processor = PacketProcessor(
        sensor_reader, geometry, output_handler,
        max_sensor_age_ms=args.max_sensor_age
    )
    
    # Запуск потоков
    sensor_thread = threading.Thread(target=sensor_reader.run, daemon=True)
    gps_thread = threading.Thread(target=gps_reader.run, daemon=True)
    sensor_thread.start()
    gps_thread.start()
    
    logger.info("⏳ Ожидание первого валидного GPS-кадра...")
    
    try:
        # Основной цикл: обработка входящих GPS кадров
        while True:
            gps_data = gps_queue.get(timeout=STARTUP_TIMEOUT_S)
            processor.process_packet(gps_data)
    except queue.Empty:
        logger.error(f"❌ Таймаут ожидания GPS ({STARTUP_TIMEOUT_S} сек). Проверьте подключение.")
    except KeyboardInterrupt:
        logger.info("🛑 Получен сигнал Ctrl+C, завершение работы...")
    finally:
        # Корректное завершение потоков
        sensor_reader.stop_evt.set()
        gps_reader.stop_evt.set()
        time.sleep(0.5)  # Даём время потокам на завершение
        
        output_handler.close()
        
        # Итоговая статистика
        logger.info("=" * 60)
        logger.info(f"СТАТИСТИКА ЗА СЕССИЮ:")
        logger.info(f"  Обработано пакетов:   {processor.stats['packets']}")
        logger.info(f"  Валидных пакетов:     {processor.stats['valid_packets']}")
        logger.info(f"  Датчики: прочитано    {sensor_reader.stats['read']} байт")
        logger.info(f"  Датчики: распарсено   {sensor_reader.stats['parsed']} кадров")
        logger.info(f"  GPS: прочитано строк  {gps_reader.stats['read']}")
        logger.info(f"  GPS: распарсено       {gps_reader.stats['parsed']}")
        logger.info("=" * 60)


if __name__ == '__main__':
    main()