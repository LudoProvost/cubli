import serial

# PARAMETER DEFINITIONS
port = 'COM24'          # check device manager
baudrate = 38400        # 38400 or 115200
lsb_sens = 131

serial_port = serial.Serial(port, baudrate)
print("reading port")

while True:
    try:
        raw_serial_data = serial_port.read(6)

        unsigned_data = []

        unsigned_data.append((raw_serial_data[0] << 8) | raw_serial_data[1])
        unsigned_data.append((raw_serial_data[2] << 8) | raw_serial_data[3])
        unsigned_data.append((raw_serial_data[4] << 8) | raw_serial_data[5])

        signed_data = [
            round(unsigned_val/lsb_sens, 2) if unsigned_val < 32768 else round((unsigned_val - 65536)/lsb_sens,2)
            for unsigned_val in unsigned_data
        ]

        print(signed_data)
    except KeyboardInterrupt:
        # serial_port.write("does this work?")
        serial_port.close()
        print("port closed")