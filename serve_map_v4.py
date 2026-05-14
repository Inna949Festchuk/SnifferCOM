#!/usr/bin/env python3
"""
Простой HTTP-сервер для карты с поддержкой CORS и авто-освобождением порта
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
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()
    
    def do_GET(self):
        if self.path.endswith('.jsonl'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/x-jsonlines')
            self.end_headers()
            filepath = DIRECTORY / self.path.lstrip('/')
            if filepath.exists():
                with open(filepath, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_error(404, 'File not found')
        else:
            super().do_GET()
    
    def log_message(self, format, *args):
        # Более читаемый лог
        print(f"🌐 {args[0]} {args[1]} {args[2]}")

def run_server():
    os.chdir(DIRECTORY)
    
    # 🔑 Разрешаем повторное использование адреса (решает "Address already in use")
    socketserver.TCPServer.allow_reuse_address = True
    
    try:
        with socketserver.TCPServer(("", PORT), CORSHandler) as httpd:
            print(f"🚀 Сервер запущен: http://localhost:{PORT}")
            print(f"🗺️  Откройте: http://localhost:{PORT}/map.html")
            print(f"📁 Каталог: {DIRECTORY.resolve()}")
            print(f"⚠️  Остановка: Ctrl+C")
            sys.stdout.flush()
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Сервер остановлен")
    except OSError as e:
        if e.errno == 98:
            print(f"❌ Порт {PORT} занят. Завершите другой процесс или выполните:")
            print(f"   Linux:   sudo fuser -k {PORT}/tcp")
            print(f"   Windows: netstat -aon | findstr :{PORT} → taskkill /F /PID <PID>")
        else:
            raise

if __name__ == "__main__":
    run_server()