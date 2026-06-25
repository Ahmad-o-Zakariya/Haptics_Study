import serial
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'  # <<< CHANGE THIS to match your Arduino UNO R4 port!
BAUD_RATE = 115200
DATA_POINTS = 200     # Number of points shown on screen at one time

# History buffers
time_data = collections.deque([0]*DATA_POINTS, maxlen=DATA_POINTS)
pos_data = collections.deque([0]*DATA_POINTS, maxlen=DATA_POINTS)
deriv_data = collections.deque([0]*DATA_POINTS, maxlen=DATA_POINTS)
force_data = collections.deque([0]*DATA_POINTS, maxlen=DATA_POINTS)

# Initialize Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected successfully to {SERIAL_PORT}")
except Exception as e:
    print(f"Error connecting to port {SERIAL_PORT}: {e}")
    print("Double check your COM port number and ensure the Arduino Serial Monitor is CLOSED.")
    sys.exit()

# Setup the Matplotlib Figure with 3 Subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title('1-DOF Fractional Haptic Rig Telemetry')

frame_count = 0

def update_plot(frame):
    global frame_count
    
    # Read all available data from serial buffer to keep it fresh
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            # Split the Arduino stream: "Position,Fractional_Derivative,Force"
            data = line.split(',')
            
            if len(data) == 3:
                pos = float(data[0])
                deriv = float(data[1])
                force = float(data[2])
                
                frame_count += 1
                time_data.append(frame_count)
                pos_data.append(pos)
                deriv_data.append(deriv)
                force_data.append(force)
        except (ValueError, IndexError):
            pass # Ignore corrupted or partial serial frames
        except Exception as e:
            print(f"Read error: {e}")

    # Clear and redraw Subplot 1: Position
    ax1.clear()
    ax1.plot(time_data, pos_data, label='Encoder Position (Counts)', color='b', linewidth=2)
    ax1.axhline(y=400, color='r', linestyle='--', label='Virtual Wall Boundary')
    ax1.set_ylabel('Position')
    ax1.legend(loc='upper left')
    ax1.grid(True)

    # Clear and redraw Subplot 2: Fractional Derivative
    ax2.clear()
    ax2.plot(time_data, deriv_data, label='Fractional Derivative ($D^{0.5}x$)', color='orange', linewidth=2)
    ax2.set_ylabel('Derivative')
    ax2.legend(loc='upper left')
    ax2.grid(True)

    # Clear and redraw Subplot 3: Output Force
    ax3.clear()
    ax3.plot(time_data, force_data, label='Motor Force Command', color='g', linewidth=2)
    ax3.set_ylabel('Force / PWM')
    ax3.set_xlabel('Sample Frames')
    ax3.legend(loc='upper left')
    ax3.grid(True)

# Run the animation loop (interval=25ms matches the Arduino's 40Hz telemetry output)
ani = animation.FuncAnimation(fig, update_plot, interval=25, cache_frame_data=False)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial port safely closed.")