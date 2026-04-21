# sniffer.py
import serial
import time

print("🔍 Слушаю порт 5 секунд...")
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()  # сбросить буфер на старте
    time.sleep(0.5)           # подождать первый пакет

    if ser.in_waiting > 0:
        data = ser.read(min(ser.in_waiting, 500))

        print(f"\n📦 Получено {len(data)} байт:\n")
        print("🔹 HEX  :", data[:120].hex(' '))
        print("🔹 ASCII:", repr(data[:120]))
        print()

        # Авто-анализ
        if b'\x00\x75\x00\xff\xf0\x0f' in data:
            print("✅ Найдена бинарная преамбула! Всё как в доке.")
        elif b',' in data:
            print("✅ Данные похожи на чистый CSV/текст (без бинарной обёртки).")
            print("💡 В этом случае импортер будет работать через ser.readline()")
        else:
            print("⚠️  Формат не определён. Пришлите вывод HEX.")
    else:
        print("⚠️  Данные не пришли. Убедитесь, что устройство активно шлёт пакеты.")

except Exception as e:
    print(f"❌ Ошибка: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
