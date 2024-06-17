import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

# Set up the serial port connection
ser = serial.Serial('COM5', 115200)  # Replace 'COM5' with your Arduino's serial port

# Initialize lists to store time, angle, and PID values
times = []
angles = []
errors = []
outputs = []
Kp_values = []
Ki_values = []
Kd_values = []

# Set up the figure and three subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Set up the first subplot (angle)
line1, = ax1.plot([], [], lw=2, label='Angle')
ax1.set_ylim(-10, 10)  # You can adjust the y-axis limit based on your requirements
ax1.set_ylabel('Angle (degrees)')
ax1.set_title('Real-Time Angle Plot')
ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
ax1.grid(True)
ax1.legend(loc='upper right')
angle_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes)

# Set up the second subplot (output)
line2, = ax2.plot([], [], lw=2, color='r', label='Output')
ax2.set_ylim(-300, 300)  # You can adjust the y-axis limit based on your requirements
ax2.set_ylabel('Output')
ax2.set_title('Real-Time Output Plot')
ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
ax2.grid(True)
ax2.legend(loc='upper right')
output_text = ax2.text(0.02, 0.95, '', transform=ax2.transAxes)
Kp_text = ax2.text(0.02, 0.85, '', transform=ax2.transAxes)
Ki_text = ax2.text(0.02, 0.75, '', transform=ax2.transAxes)
Kd_text = ax2.text(0.02, 0.65, '', transform=ax2.transAxes)

# Set up the third subplot (error)
line3, = ax3.plot([], [], lw=2, color='g', label='Error')
ax3.set_ylim(-60, 60)  # You can adjust the y-axis limit based on your requirements
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error')
ax3.set_title('Real-Time Error Plot')
ax3.yaxis.set_major_locator(MaxNLocator(integer=True))
ax3.grid(True)
ax3.legend(loc='upper right')
error_text = ax3.text(0.02, 0.95, '', transform=ax3.transAxes)

# Adjust layout
plt.tight_layout()

start_time = time.time()

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    angle_text.set_text('')
    output_text.set_text('')
    error_text.set_text('')
    Kp_text.set_text('')
    Ki_text.set_text('')
    Kd_text.set_text('')
    return line1, line2, line3, angle_text, output_text, error_text, Kp_text, Ki_text, Kd_text

def update(frame):
    global times, angles, errors, outputs, Kp_values, Ki_values, Kd_values

    # Read data from the serial port
    while ser.in_waiting:
        try:
            data = ser.readline().decode().strip()
            if data.startswith('Angle: '):
                parts = data.split()
                if len(parts) >= 16:  # Ensure all values are received
                    angle = float(parts[1])
                    error = float(parts[3])
                    integral = float(parts[5])
                    derivative = float(parts[7])
                    output = float(parts[9])
                    Kp = float(parts[11])
                    Ki = float(parts[13])
                    Kd = float(parts[15])
                    current_time = time.time() - start_time

                    times.append(current_time)
                    angles.append(angle)
                    errors.append(error)
                    outputs.append(output)
                    Kp_values.append(Kp)
                    Ki_values.append(Ki)
                    Kd_values.append(Kd)

                    # Update line data
                    line1.set_data(times, angles)
                    line2.set_data(times, outputs)
                    line3.set_data(times, errors)
                    
                    # Update text annotation with the latest values
                    angle_text.set_text(f'Angle: {angle:.2f}°')
                    output_text.set_text(f'Output: {output:.2f}')
                    error_text.set_text(f'Error: {error:.2f}')
                    Kp_text.set_text(f'Kp: {Kp:.5f}')
                    Ki_text.set_text(f'Ki: {Ki:.5f}')
                    Kd_text.set_text(f'Kd: {Kd:.5f}')

                    # Dynamically update x-axis limit
                    ax1.set_xlim(0, current_time)
                    ax2.set_xlim(0, current_time)
                    ax3.set_xlim(0, current_time)
        except Exception as e:
            print(f"Error: {e}")
            pass

    return line1, line2, line3, angle_text, output_text, error_text, Kp_text, Ki_text, Kd_text

ani = animation.FuncAnimation(fig, update, init_func=init, interval=5)

plt.show()

# Close the serial port when the plot window is closed
ser.close()
