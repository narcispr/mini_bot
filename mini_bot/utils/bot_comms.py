import serial
import time
import threading

START_BYTE = 0xAA
END_BYTE = 0x55
ID_CMD_RPM = 0x10
ID_SENSOR_RANGE = 0x01
ID_SENSOR_ENCODERS = 0x02
ID_SENSOR_COMPASS = 0x03

serial_lock = threading.Lock()  # mutex pel port sèrie

def to_motor_bytes(speed):
    direction = 1 if speed >= 0 else 0
    pwm = min(abs(speed), 255)
    return (direction, pwm)

def build_pwm_message(left_speed, right_speed):
    l_dir, l_pwm = to_motor_bytes(left_speed)
    r_dir, r_pwm = to_motor_bytes(right_speed)
    data = bytearray([l_dir, l_pwm, r_dir, r_pwm])
    length = len(data)
    msg = bytearray([START_BYTE, ID_CMD_RPM, length]) + data
    checksum = 0
    for b in msg[1:]:
        checksum ^= b
    msg += bytearray([checksum, END_BYTE])
    return msg

def read_message(ser):
    state = 0
    msg = []
    while ser.in_waiting:
        with serial_lock:
            byte = ser.read(1)
        if not byte:
            return None
        byte = byte[0]
        if state == 0:
            if byte == START_BYTE:
                msg = [byte]
                state = 1
        elif state == 1:
            msg.append(byte)
            state = 2
        elif state == 2:
            msg.append(byte)
            length = byte
            state = 3
            data_bytes = []
        elif state == 3:
            data_bytes.append(byte)
            if len(data_bytes) == length:
                msg.extend(data_bytes)
                state = 4
        elif state == 4:
            checksum = byte
            msg.append(byte)
            state = 5
        elif state == 5:
            if byte == END_BYTE:
                msg.append(byte)
                computed = 0
                for b in msg[1:-2]:
                    computed ^= b
                if checksum == computed:
                    return msg[1], msg[3:-2]
            state = 0
    return None

# ⬅️ THREAD 1: Lectura sèrie
def read_thread(ser):
    while True:
        data = read_message(ser)
        if data is not None:
            msg_id, payload = data
            if msg_id == ID_SENSOR_RANGE and len(payload) == 2:
                value = payload[0] | (payload[1] << 8)
                print(f"[Sensor] Rang: {value} cm")
            else:
                print(f"[Altres dades] ID: {msg_id}, Data: {payload}")
        time.sleep(0.04)

# ⬅️ THREAD 2: Entrada d’usuari i enviament de comandes
def write_thread(ser):
    while True:
        try:
            left = int(input("RPM esquerra (-255 a 255): "))
            right = int(input("RPM dreta (-255 a 255): "))
            msg, _, _ = build_pwm_message(left, right)
            with serial_lock:
                ser.write(msg)
            print(f"[Enviat] {list(msg)}\n")
        except ValueError:
            print("Introdueix valors vàlids entre -255 i 255.")
        except KeyboardInterrupt:
            print("\nSortint...")
            break

# ⬅️ MAIN
def main():
    # port = input("Port serial (ex: /dev/ttyUSB0 o COM3): ")
    port = "/dev/ttyACM0"  # o "COM6" a Windows
    ser = serial.Serial(port, 9600, timeout=1)

    reader = threading.Thread(target=read_thread, args=(ser,), daemon=True)
    writer = threading.Thread(target=write_thread, args=(ser,))

    reader.start()
    writer.start()

    writer.join()  # espera que l’usuari acabi

# if __name__ == "__main__":
#    main()

