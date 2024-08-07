import serial

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# PARAMETER DEFINITIONS
port = 'COM24'          # check device manager
baudrate = 38400        # 38400 or 115200
lsb_sens = 131


def update(i, data_list_x, data_list_y, data_list_z, serial_port):

    try:
        # READ GYRO values:
        raw_serial_data = serial_port.read(6)

        unsigned_data = []

        unsigned_data.append((raw_serial_data[0] << 8) | raw_serial_data[1])
        unsigned_data.append((raw_serial_data[2] << 8) | raw_serial_data[3])
        unsigned_data.append((raw_serial_data[4] << 8) | raw_serial_data[5])

        signed_data = [
        round(unsigned_val/lsb_sens, 2) if unsigned_val < 32768 else round((unsigned_val - 65536)/lsb_sens,2)
        for unsigned_val in unsigned_data
        ]

        data_list_x.append(signed_data[0])
        data_list_y.append(signed_data[1])  
        data_list_z.append(signed_data[2])

        data_list_x = data_list_x[-50:]                           # Fix the list size so that the animation plot 'window' is x number of points
        data_list_y = data_list_y[-50:]                         
        data_list_z = data_list_z[-50:]                          
        
        # PLOT GYRO GRAPH
        ax_gyro.clear()                                          # Clear last data frame
        
        ax_gyro.plot(data_list_x, color='blue', label='x-tilt')                                   # Plot new data frame
        ax_gyro.plot(data_list_y, color='green', label='y-tilt')
        ax_gyro.plot(data_list_z, color='red', label='z-tilt')

        ax_gyro.set_ylim([-50,50])                              # Set Y axis limit of plot
        ax_gyro.set_title("Gyro Data")                        # Set title of figure
        ax_gyro.set_ylabel("Value") 

        # Add legend
        ax_gyro.legend(loc='upper right')

    except:                                             # Pass if data point is bad                               
        pass





data_list_x = []
data_list_y = []
data_list_z = []
                                
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window


ax_gyro = fig.add_subplot(111)                               # Add subplot to main fig window



serial_port = serial.Serial(port, baudrate)
print("reading port")

ani = animation.FuncAnimation(fig, update, frames=100, fargs=(data_list_x, data_list_y, data_list_z, serial_port), interval=100) 
plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
