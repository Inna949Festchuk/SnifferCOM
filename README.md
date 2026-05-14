Использование (более подробно см. README.md):
#  Терминал 1 — сбор данных (пример для Windows)
python importercom.py --sensor-port COM4 --gps-port COM5 --num-sensors 4 --output field_data.jsonl

#  Терминал 1 — сбор данных (пример для Linux)
python importercom.py --sensor-port /dev/ttyUSB0 --gps-port /dev/ttyACM0 --num-sensors 4 --output field_data.jsonl

# Терминал 2 — сервер карты
python3 serve_map.py

# Открыть Браузер
# Ввести http://localhost:8000/map.html