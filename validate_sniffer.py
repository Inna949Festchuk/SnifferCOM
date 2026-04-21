#!/usr/bin/env python3
"""
🔍 SERIAL TRAFFIC SNIFFER & VALIDATOR
Универсальный скрипт для захвата, очистки и парсинга бинарного потока с COM-порта.
Структура: [CSV_DATA] \x00 [CSV_DATA] \x00 ... \xFF\xF0\x0F [RSSI] [CSV_DATA] ...
"""
import serial
import time
import logging
import argparse
from typing import Dict, Optional

# Настройка логирования
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

class TrafficSniffer:
    # Константы протокола
    SYNC_HEADER = b'\xff\xf0\x0f'  # Синхробайты
    DELIMITER = b'\x00'           # Разделитель записей (NULL)

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, max_records=10):
        self.port = port
        self.baudrate = baudrate
        self.max_records = max_records
        self.buffer = bytearray()  # Буфер для накопления сырых байт
        self.records_parsed = 0
        self.ser = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            self.ser.reset_input_buffer()
            logger.info(f"✅ Подключено к {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            logger.error(f"❌ Ошибка порта: {e}")
            return False

    def run(self):
        if not self.connect(): return
        
        logger.info(f"🔍 Ожидание записей (лимит: {self.max_records}). Нажмите Ctrl+C для выхода.")
        print("\n" + "="*80)
        
        try:
            while self.records_parsed < self.max_records:
                if self.ser.in_waiting > 0:
                    # Читаем все доступные байты
                    raw_data = self.ser.read(self.ser.in_waiting)
                    self._process_stream(raw_data)
                time.sleep(0.01)
        except KeyboardInterrupt:
            logger.info("\n⛔ Остановлено пользователем")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            print("\n" + "="*80)
            logger.info(f"📊 СЕАНСОВАЯ СТАТИСТИКА: Успешно распарсено {self.records_parsed} записей.")

    def _process_stream(self, data: bytes):
        """
        Накапливает байты в буфер и извлекает полные записи по разделителю 0x00.
        """
        # Добавляем новые данные в буфер
        self.buffer.extend(data)
        
        # Разделяем буфер по разделителю 0x00
        # Последний элемент списка - это "хвост", который еще не завершен (сохраняем его)
        packets = self.buffer.split(self.DELIMITER)
        self.buffer = packets[-1]
        
        # Обрабатываем все полные пакеты
        for packet in packets[:-1]:
            self._parse_packet(packet)

    def _parse_packet(self, packet: bytearray):
        """
        Очищает пакет от синхробайтов и парсит CSV строку.
        """
        if not packet: return

        # 1. Очистка от синхро-пакетов [FF F0 0F] + [RSSI Byte]
        # Ищем синхро-заголовок и вырезаем его + 1 байт RSSI
        while True:
            idx = packet.find(self.SYNC_HEADER)
            if idx == -1:
                break  # Синхро больше нет
            
            # Вырезаем 4 байта (3 байта заголовка + 1 байт RSSI)
            # Если хвост короче, вырезаем сколько есть
            cut_len = min(4, len(packet) - idx)
            del packet[idx : idx + cut_len]

        # 2. Декодирование в строку
        try:
            text = packet.decode('utf-8', errors='ignore').strip()
        except Exception:
            return

        if not text or ',' not in text:
            return # Не похоже на CSV

        # 3. Парсинг полей
        record = self._csv_to_dict(text)
        if record:
            self.records_parsed += 1
            print(f"📦 ЗАПИСЬ #{self.records_parsed}")
            for key, val in record.items():
                print(f"   🔹 {key:20}: {val}")
            print("-" * 80)

    def _csv_to_dict(self, line: str) -> Optional[Dict]:
        """Преобразует строку CSV в словарь, совместимый с importer.py"""
        fields = [f.strip() for f in line.split(',')]
        if len(fields) < 13:
            return None
        
        try:
            return {
                'sensor_id': int(fields[0]) if fields[0].isdigit() else fields[0],
                'utc_time': float(fields[1].replace(',', '.')) if fields[1] else None,
                'state': fields[2],
                'latitude': float(fields[3]) if fields[3] else None,
                'n_s_indica': fields[4],
                'longitude': float(fields[5]) if fields[5] else None,
                'e_w_indica': fields[6],
                'ground_spe': float(fields[7]) if fields[7] else None,
                'position': float(fields[8]) if fields[8] else None,
                'date': float(fields[9]) if fields[9] else None,
                'f2': float(fields[10]) if fields[10] else None,
                'f3': float(fields[11]) if fields[11] else None,
                'alarm': bool(int(fields[12])) if fields[12].isdigit() else False
            }
        except ValueError:
            return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='🔍 Сниффер и валидатор COM-порта')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Порт устройства')
    parser.add_argument('--baud', type=int, default=115200, help='Скорость (бод)')
    parser.add_argument('--max', type=int, default=10, help='Лимит записей для теста')
    args = parser.parse_args()

    sniffer = TrafficSniffer(
        port=args.port, 
        baudrate=args.baud, 
        max_records=args.max
    )
    sniffer.run()
