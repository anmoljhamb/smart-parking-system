import time

import serial

arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.1)

# Define a dynamic size for the list
list_size = 3  # Change this value to set the list size
# Tupl
slots = ["" for _ in range(list_size)]
recieved_cars = []


def get_first_empty_slot():
    for i in range(list_size):
        if len(slots[i]) == 0:
            return i
    return -1


def get_el(uid: str):
    for car in recieved_cars:
        if car[0] == uid:
            return car
    return None


def handle_input(data):
    global recieved_cars
    print(slots)

    if data.startswith("vehicle-"):
        el = get_el(data[8:])
        if el:
            # Provided string is in the list, remove it
            el = get_el(data[8:])
            recieved_cars.remove(el)
            slot = slots.index(data[8:])
            slots[slot] = ""
            return "r"
        if len(recieved_cars) >= list_size:
            return "n"
        slot = get_first_empty_slot()
        slots[slot] = data[8:]
        recieved_cars.append(
            (data[8:], time.time(), slot),
        )
        return "e" + str(slot + 1)
    else:
        # Print the string that was sent
        print("Received:", data)
        return -1


def read_from_arduino():
    data = arduino.readline().decode().strip()  # Read string data from Arduino
    return data


def send_to_arduino(x):
    arduino.write(bytes(x, "utf-8"))  # Send string data to Arduino
    time.sleep(0.05)


while True:
    value = read_from_arduino()  # Read data from Arduino

    if value:  # If data is received from Arduino
        result = handle_input(value)

        # Sending appropriate response to Arduino
        if result != -1:
            send_to_arduino(result)
        # if result == 0:
        #     send_to_arduino("0")  # Sending '0' if list is full
        # elif result == 1:
        #     send_to_arduino("1")  # Sending '1' if string is added
        # elif result == 2:
        #     send_to_arduino("2")  # Sending '2' if string is removed
