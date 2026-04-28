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
    data = bytearray([r_dir, r_pwm,l_dir, l_pwm]) # TODO: SWAP here and in ARDUINO M4->left and M3->Right!!
    length = len(data)
    msg = bytearray([START_BYTE, ID_CMD_RPM, length]) + data
    checksum = 0
    for b in msg[1:]:
        checksum ^= b
    msg += bytearray([checksum, END_BYTE])
    return msg

def read_message(ser, lock=None):
    """Read and decode one framed message from serial if available.

    Parser state is persisted across calls so partial frames are not lost.
    """
    if not hasattr(read_message, '_state'):
        read_message._state = 0
        read_message._msg_id = 0
        read_message._length = 0
        read_message._payload = []
        read_message._checksum = 0

    lock_to_use = lock if lock is not None else serial_lock
    max_payload_length = 64

    while ser.in_waiting:
        with lock_to_use:
            raw = ser.read(1)

        if not raw:
            return None

        byte = raw[0]

        if read_message._state == 0:
            if byte == START_BYTE:
                read_message._state = 1

        elif read_message._state == 1:
            read_message._msg_id = byte
            read_message._checksum = byte
            read_message._state = 2

        elif read_message._state == 2:
            read_message._length = byte
            read_message._checksum ^= byte
            read_message._payload = []
            if read_message._length > max_payload_length:
                read_message._state = 0
            else:
                read_message._state = 3

        elif read_message._state == 3:
            read_message._payload.append(byte)
            read_message._checksum ^= byte
            if len(read_message._payload) >= read_message._length:
                read_message._state = 4

        elif read_message._state == 4:
            received_checksum = byte
            read_message._state = 5
            read_message._received_checksum = received_checksum

        elif read_message._state == 5:
            if byte == END_BYTE and read_message._received_checksum == read_message._checksum:
                msg_id = read_message._msg_id
                payload = read_message._payload[:]
                read_message._state = 0
                return msg_id, payload

            read_message._state = 0

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

