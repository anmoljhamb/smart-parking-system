import time

import serial

arduino = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.1)

# Define a dynamic size for the list
list_size = 3  # Change this value to set the list size
received_strings = []


def handle_input(data):
    global received_strings
    print(received_strings)

    if data.startswith("vehicle-"):
        if data[8:] in received_strings:
            # Provided string is in the list, remove it
            received_strings.remove(data[8:])
            return 2
        if len(received_strings) >= list_size:
            return 0
        received_strings.append(data[8:])
        return 1
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
        if result == 0:
            send_to_arduino("0")  # Sending '0' if list is full
        elif result == 1:
            send_to_arduino("1")  # Sending '1' if string is added
        elif result == 2:
            send_to_arduino("2")  # Sending '2' if string is removed
