# Script Python para comunicarse con Arduino
# pip install pyserial
import serial, json, time
arduino = serial.Serial('COM11', 115200, timeout=1) # Cambiar puerto
time.sleep(2) # Esperar inicialización Arduino
def enviar(cmd):
    arduino.write((cmd + '\n').encode())
    linea = arduino.readline().decode().strip()
    return json.loads(linea) if linea else None
print('Conectado:', enviar('ALL'))

# Ciclo de monitoreo
for _ in range(5):
    resp = enviar('ALL')
    if resp:
        d = resp['data']
        print(f"Temp: {d['temp']}°C Hum: {d['hum']}% Luz: {d['luz']}")
    time.sleep(3)

enviar('LED:ON')
time.sleep(1)
enviar('LED:OFF')