import serial
import matplotlib.pyplot as plt
from collections import deque

# === Setup Serial ===
ser = serial.Serial('/dev/ttyUSB0', 115200)  # STM32 USB virtual COM port on Linux

MAX_SAMPLES = 500
angle_vals = deque(maxlen=MAX_SAMPLES)
pwm_vals   = deque(maxlen=MAX_SAMPLES)
avg_vals   = deque(maxlen=MAX_SAMPLES)
time_vals  = deque(maxlen=MAX_SAMPLES)
cnt = 0

plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('STM32 Self-Balancing Robot Monitor')

# === Main Loop ===
while True:
    try:
        line = ser.readline().decode().strip()
        # Expected: "angle:1.23 pwm:456 L:12.3mm R:12.3mm avg:12.3mm"

        if 'angle:' not in line:
            continue

        parts = line.split()
        # parts = ['angle:1.23', 'pwm:456', 'L:12.3mm', 'R:12.3mm', 'avg:12.3mm']

        angle = float(parts[0].split(':')[1])
        pwm   = int(parts[1].split(':')[1])
        avg   = float(parts[4].split(':')[1].replace('mm', ''))

        angle_vals.append(angle)
        pwm_vals.append(pwm)
        avg_vals.append(avg)
        time_vals.append(cnt)
        cnt += 1

        # === Plot ===
        ax1.cla()
        ax1.plot(time_vals, angle_vals, 'b.-', linewidth=0.8)
        ax1.axhline(0, color='r', linestyle='--', linewidth=0.8)
        ax1.set_ylabel('Angle (deg)')
        ax1.set_title('Tilt Angle')
        ax1.grid(True)

        ax2.cla()
        ax2.plot(time_vals, pwm_vals, 'g.-', linewidth=0.8)
        ax2.set_ylabel('PWM')
        ax2.set_title('Motor PWM Output')
        ax2.grid(True)

        ax3.cla()
        ax3.plot(time_vals, avg_vals, 'r.-', linewidth=0.8)
        ax3.set_ylabel('Distance (mm)')
        ax3.set_xlabel('Sample')
        ax3.set_title('Average Encoder Distance')
        ax3.grid(True)

        plt.tight_layout()
        plt.pause(0.001)

    except KeyboardInterrupt:
        print("Stopped.")
        ser.close()
        break
    except Exception as e:
        print("Parse error:", e)
        continue