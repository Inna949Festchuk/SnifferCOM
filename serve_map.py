#!/usr/bin/env python3
"""
Простой HTTP-сервер для карты с поддержкой CORS и авто-освобождением порта
Исправленная версия: корректная обработка отсутствующего файла и ошибок логирования
"""

import http.server
import socketserver
import json
from pathlib import Path
import os
import sys

PORT = 8000
DIRECTORY = Path(__file__).parent

class CORSHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIRECTORY), **kwargs)
    
    def end_headers(self):
        """Добавляет CORS заголовки к ответу"""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()
    
    def do_OPTIONS(self):
        """Обработка preflight CORS запросов"""
        self.send_response(200)
        self.end_headers()
    
    def do_GET(self):
        """
        Обработка GET запросов.
        Специальная обработка для .jsonl файлов (отправка как есть).
        """
        # Проверка на favicon.ico - отправляем 204 (No Content) вместо 404
        if self.path == '/favicon.ico':
            self.send_response(204)  # No Content
            self.end_headers()
            return
        
        if self.path.endswith('.jsonl'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/x-jsonlines')
            self.end_headers()
            filepath = DIRECTORY / self.path.lstrip('/')
            if filepath.exists():
                try:
                    with open(filepath, 'rb') as f:
                        self.wfile.write(f.read())
                except Exception as e:
                    # Тихая обработка ошибок чтения файла
                    pass
            else:
                # Файл не существует - возвращаем пустой массив (клиент сам обработает)
                self.wfile.write(b'')
        else:
            # Для всех остальных файлов используем стандартную обработку
            super().do_GET()
    
    def log_message(self, format, *args):
        """
        Безопасное логирование запросов.
        Исправлена проблема с индексом args при разных форматах сообщений.
        """
        try:
            # Проверяем, что args содержит достаточно элементов для формата
            if format == "code %d, message %s":
                # Это сообщение об ошибке (404 и т.д.)
                if len(args) >= 2:
                    print(f"⚠️ {args[0]} {args[1]}")
                else:
                    print(f"⚠️ Ошибка: {args}")
            elif format == "%s \"%s\" %s \"%s\"":
                # Стандартный формат лога Apache
                if len(args) >= 4:
                    print(f"🌐 {args[0]} {args[1]} {args[2]} {args[3]}")
                else:
                    print(f"🌐 {args}")
            else:
                # Любой другой формат - выводим как есть
                if args:
                    print(f"📝 {args}")
                else:
                    print(f"📝 {format}")
        except Exception as e:
            # Если что-то пошло не так, выводим минимум информации
            print(f"📝 Запрос обработан")


def run_server():
    """Запуск HTTP сервера с автоматическим освобождением порта"""
    os.chdir(DIRECTORY)
    
    # Разрешаем повторное использование адреса (решает "Address already in use")
    socketserver.TCPServer.allow_reuse_address = True
    
    try:
        with socketserver.TCPServer(("", PORT), CORSHandler) as httpd:
            print(f"🚀 Сервер запущен: http://localhost:{PORT}")
            print(f"🗺️  Откройте: http://localhost:{PORT}/map.html")
            print(f"📁 Каталог: {DIRECTORY.resolve()}")
            print(f"📄 Ожидание файла: field_data.jsonl (создаётся скриптом importercom.py)")
            print(f"⚠️  Остановка: Ctrl+C")
            print()
            sys.stdout.flush()
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Сервер остановлен")
    except OSError as e:
        if e.errno == 98 or e.errno == 48:  # Linux: 98, macOS: 48
            print(f"❌ Порт {PORT} занят. Завершите другой процесс или выполните:")
            print(f"   Linux/macOS: sudo lsof -ti:{PORT} | xargs kill -9")
            print(f"   Windows:     netstat -aon | findstr :{PORT} → taskkill /F /PID <PID>")
        else:
            raise


if __name__ == "__main__":
    run_server()