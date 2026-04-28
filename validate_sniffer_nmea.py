#!/usr/bin/env python3
"""
validate_sniffer_nmea.py
Парсер данных с геопозиционера + 3-осевого магнитометра (формат NMEA-0183 + расширение)

Протокол: $GNRMC,<time>,<status>,<lat>,<N/S>,<lon>,<E/W>,<speed>,<course>,<date>,...,*<checksum>;<mag_x>,<mag_y>,<mag_z>\r\n
"""

import argparse
import serial
import time
import re
import math
import sys
from typing import Optional, Dict, List
from datetime import datetime


def dmm_to_decimal(dmm_str: str, direction: str) -> Optional[float]:
    """
    Конвертирует координату из формата NMEA DDMM.MMMM в десятичные градусы.
    
    Args:
        dmm_str: строка вида '5444.50595' (широта) или '02030.04440' (долгота)
        direction: 'N'/'S' для широты, 'E'/'W' для долготы
    
    Returns:
        float: координата в десятичных градусах, или None при ошибке
    """
    try:
        dmm = float(dmm_str)
        if dmm == 0:
            return 0.0
        # Определяем количество цифр в целой части для разделения градусов/минут
        if 'N' in direction.upper() or 'S' in direction.upper():
            # Широта: DDMM.MMMM (2 цифры градусов)
            degrees = int(dmm / 100)
        else:
            # Долгота: DDDMM.MMMM (3 цифры градусов)
            degrees = int(dmm / 100) if dmm < 10000 else int(dmm / 100)
        
        minutes = dmm - (degrees * 100)
        decimal = degrees + minutes / 60.0
        
        # Применяем знак по полушарию
        if direction.upper() in ['S', 'W']:
            decimal = -decimal
        return decimal
    except (ValueError, TypeError):
        return None


def calculate_azimuth(x: float, y: float, declination: float = 0.0) -> float:
    """
    Вычисляет азимут (курс) по данным магнитометра.
    
    Args:
        x: значение по оси X магнитометра
        y: значение по оси Y магнитометра
        declination: магнитное склонение в градусах (положительное для восточного)
    
    Returns:
        float: азимут в градусах [0..360), где 0° = север, 90° = восток
    """
    if x == 0 and y == 0:
        return 0.0
    
    # Базовый азимут из вектора магнитного поля
    azimuth = math.degrees(math.atan2(y, x))
    
    # Коррекция: atan2 возвращает [-180, +180], приводим к [0, 360)
    azimuth = (azimuth + 360) % 360
    
    # Компенсация магнитного склонения (опционально)
    azimuth = (azimuth + declination + 360) % 360
    
    return round(azimuth, 2)


def calculate_field_strength(x: float, y: float, z: float) -> float:
    """Вычисляет модуль вектора магнитного поля."""
    return round(math.sqrt(x**2 + y**2 + z**2), 2)


def validate_nmea_checksum(sentence: str) -> bool:
    """
    Проверяет контрольную сумму NMEA-0183.
    Формат: $...*HH где HH — hex-значение XOR всех байтов между $ и *
    """
    if '*' not in sentence:
        return True  # Если чексуммы нет — пропускаем проверку
    
    try:
        body, received_cs = sentence.split('*')
        received_cs = int(received_cs[:2], 16)  # Берём первые 2 символа после *
        
        # Вычисляем XOR всех символов между $ и *
        calculated_cs = 0
        for char in body[1:]:  # Пропускаем ведущий $
            calculated_cs ^= ord(char)
        
        return calculated_cs == received_cs
    except:
        return True  # При ошибке парсинга не блокируем обработку


def parse_nmea_extended(sentence: str, declination: float = 0.0) -> Optional[Dict]:
    """
    Парсит расширенную NMEA-строку с магнитными данными.
    
    Returns:
        Dict с распарсенными полями или None при ошибке
    """
    sentence = sentence.strip('\r\n')
    
    if not sentence.startswith('$'):
        return None
    
    # Проверка чексуммы (неблокирующая)
    if not validate_nmea_checksum(sentence):
        print(f"⚠️  Warning: checksum mismatch in: {sentence[:50]}...")
    
    result = {
        'timestamp': datetime.now().isoformat(),
        'raw': sentence,
        'type': 'nmea_extended',
        'valid': False
    }
    
    # Отделяем расширение с магнитными данными (после ;)
    mag_data = None
    if ';' in sentence:
        nmea_part, mag_part = sentence.split(';', 1)
        # Убираем чексумму из NMEA-части перед парсингом
        if '*' in nmea_part:
            nmea_part = nmea_part.split('*')[0]
        try:
            mag_values = list(map(float, mag_part.split(',')))
            if len(mag_values) >= 3:
                mag_data = {
                    'raw_x': mag_values[0],
                    'raw_y': mag_values[1],
                    'raw_z': mag_values[2],
                }
                # Вычисляем производные величины
                mag_data['azimuth'] = calculate_azimuth(
                    mag_values[0], mag_values[1], declination
                )
                mag_data['field_strength'] = calculate_field_strength(*mag_values[:3])
        except ValueError:
            pass
        sentence = nmea_part
    else:
        # Если нет расширения — тоже убираем чексумму
        if '*' in sentence:
            sentence = sentence.split('*')[0]
    
    fields = sentence.split(',')
    
    # Ожидаем формат: $GNRMC,time,status,lat,NS,lon,EW,speed,course,date,...
    if len(fields) < 12 or fields[0] not in ('$GNRMC', '$GPRMC'):
        return None
    
    try:
        # Базовые поля
        result.update({
            'time_utc': fields[1] if len(fields) > 1 else None,
            'status': fields[2] if len(fields) > 2 else None,  # A=active, V=void
            'latitude_dmm': fields[3] if len(fields) > 3 else None,
            'latitude_dir': fields[4] if len(fields) > 4 else None,
            'longitude_dmm': fields[5] if len(fields) > 5 else None,
            'longitude_dir': fields[6] if len(fields) > 6 else None,
            'speed_knots': float(fields[7]) if len(fields) > 7 and fields[7] else None,
            'course_true': float(fields[8]) if len(fields) > 8 and fields[8] else None,
            'date_ddmmyy': fields[9] if len(fields) > 9 else None,
        })
        
        # Конвертация координат
        if result['latitude_dmm'] and result['latitude_dir']:
            result['latitude'] = dmm_to_decimal(
                result['latitude_dmm'], result['latitude_dir']
            )
        if result['longitude_dmm'] and result['longitude_dir']:
            result['longitude'] = dmm_to_decimal(
                result['longitude_dmm'], result['longitude_dir']
            )
        
        # Добавляем магнитные данные если есть
        if mag_data:
            result['magnetometer'] = mag_data
        
        # Флаг валидности: статус 'A' + есть координаты
        result['valid'] = (result['status'] == 'A' and 
                          result.get('latitude') is not None and 
                          result.get('longitude') is not None)
        
        return result
        
    except (IndexError, ValueError, TypeError) as e:
        print(f"⚠️  Parse error: {e}")
        return None


class NMEASniffer:
    """Класс для приёма и парсинга NMEA-данных с последовательного порта."""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.stats = {
            'lines_read': 0,
            'parsed_ok': 0,
            'parse_errors': 0,
            'checksum_errors': 0
        }
    
    def connect(self) -> bool:
        """Открывает последовательный порт."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Даем порту время на инициализацию
            time.sleep(2)
            self.ser.reset_input_buffer()
            print(f"✅ Подключен к {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"❌ Ошибка подключения: {e}")
            return False
    
    def disconnect(self):
        """Закрывает порт."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 Порт {self.port} закрыт")
    
    def read_line(self) -> Optional[bytes]:
        """Читает одну строку из порта."""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            return self.ser.readline()
        except serial.SerialException:
            return None
    
    def parse_stream(self, max_records: Optional[int] = None, 
                     declination: float = 0.0) -> List[Dict]:
        """
        Читает и парсит поток данных.
        
        Args:
            max_records: лимит записей (None = безлимита)
            declination: магнитное склонение в градусах
        
        Returns:
            Список распарсенных записей
        """
        results = []
        print(f"\n📡 Начинаю приём данных (declination={declination}°)...\n")
        print(f"{'─'*20} {'Кадр'} {'─'*20}")
        
        try:
            while max_records is None or len(results) < max_records:
                line = self.read_line()
                if not line:
                    continue
                
                self.stats['lines_read'] += 1
                line_decoded = line.decode('utf-8', errors='ignore').strip()
                
                # Пропускаем пустые и не-NMEA строки
                if not line_decoded.startswith('$'):
                    continue
                
                parsed = parse_nmea_extended(line_decoded, declination)
                
                if parsed:
                    self.stats['parsed_ok'] += 1
                    results.append(parsed)
                    self._print_record(parsed, len(results))
                else:
                    self.stats['parse_errors'] += 1
                    print(f"⚠️  Не распарсено: {line_decoded[:60]}...")
                
                # Небольшая задержка чтобы не перегружать вывод
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n⏹️  Остановлено пользователем")
        
        return results
    
    def _print_record(self, record: Dict, num: int):
        """Красивый вывод распарсенной записи."""
        status_icon = "✅" if record['valid'] else "⚠️"
        print(f"\n{status_icon} Запись #{num} [{record['timestamp']}]")
        print(f"   Статус: {record['status']} | Время: {record['time_utc']} | Дата: {record['date_ddmmyy']}")
        
        if record.get('latitude') and record.get('longitude'):
            print(f"   📍 Координаты: {record['latitude']:.6f}, {record['longitude']:.6f}")
            print(f"      (сырые: {record['latitude_dmm']}{record['latitude_dir']}, "
                  f"{record['longitude_dmm']}{record['longitude_dir']})")
        
        if record.get('speed_knots') is not None:
            print(f"   🚀 Скорость: {record['speed_knots']:.2f} уз. | "
                  f"Курс (GPS): {record['course_true']:.1f}°")
        
        if 'magnetometer' in record:
            mag = record['magnetometer']
            print(f"   🧭 Магнитометр:")
            print(f"      Сырые: X={mag['raw_x']:.0f}, Y={mag['raw_y']:.0f}, Z={mag['raw_z']:.0f}")
            print(f"      Азимут: {mag['azimuth']:.2f}° | Модуль поля: {mag['field_strength']:.2f}")
        
        if not record['valid']:
            print(f"   ⚠️  Запись помечена как НЕВАЛИДНАЯ (проверьте статус и координаты)")


def parse_args():
    parser = argparse.ArgumentParser(
        description='Парсер NMEA-данных с магнитометром (устройство 2)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Примеры:
  %(prog)s --port /dev/ttyACM0
  %(prog)s --port COM3 --baud 9600 --max 50
  %(prog)s --port /dev/ttyACM0 --declination 10.5  # для Калининграда
        '''
    )
    parser.add_argument('--port', '-p', type=str, default='/dev/ttyACM0',
                        help='Последовательный порт (по умолчанию: /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help='Скорость порта (по умолчанию: 115200)')
    parser.add_argument('--max', '-m', type=int, default=None,
                        help='Максимальное число записей для приёма (по умолчанию: безлимита)')
    parser.add_argument('--declination', '-d', type=float, default=0.0,
                        help='Магнитное склонение в градусах (по умолчанию: 0.0)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Файл для сохранения результатов (JSON)')
    return parser.parse_args()


def main():
    args = parse_args()
    
    print(f"🔍 Sniffer NMEA v1.0 — устройство 2 (геопозиционер + 3-осевой компас)")
    print(f"   Порт: {args.port} | Скорость: {args.baud} | Declination: {args.declination}°\n")
    
    sniffer = NMEASniffer(args.port, args.baud)
    
    if not sniffer.connect():
        sys.exit(1)
    
    try:
        records = sniffer.parse_stream(max_records=args.max, declination=args.declination)
        
        # Статистика
        print(f"\n{'='*60}")
        print(f"📊 Статистика:")
        print(f"   Прочитано строк: {sniffer.stats['lines_read']}")
        print(f"   Успешно распарсено: {sniffer.stats['parsed_ok']}")
        print(f"   Ошибки парсинга: {sniffer.stats['parse_errors']}")
        print(f"   Валидных записей: {sum(1 for r in records if r['valid'])}")
        
        # Сохранение в файл если указано
        if args.output and records:
            import json
            with open(args.output, 'w', encoding='utf-8') as f:
                json.dump(records, f, ensure_ascii=False, indent=2)
            print(f"💾 Данные сохранены в {args.output}")
        
    finally:
        sniffer.disconnect()


if __name__ == '__main__':
    main()