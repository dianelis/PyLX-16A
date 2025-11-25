from math import sin, cos
from pylx16a.lx16a import *
import time

# Try to initialize with the correct port
try:
    LX16A.initialize("/dev/tty.usbserial-210", 0.1)
    print("✓ Connected to servo controller")
except Exception as e:
    print(f"✗ Failed to connect: {e}")
    exit()

# Scan for available servos first
print("Scanning for servos...")
available_servos = []
for i in range(1, 6):
    try:
        servo = LX16A(i)
        print(f"✓ Found servo with ID {i}")
        available_servos.append(i)
    except ServoTimeoutError:
        print(f"✗ Servo ID {i}: Not found")
    except Exception as e:
        print(f"✗ Servo ID {i}: Error - {e}")

if len(available_servos) == 0:
    print("\n❌ No servos found!")
    print("Check:")
    print("- Power supply (6-12V)")
    print("- Wiring connections")
    print("- Servo IDs")
    print("- Try connecting one servo at a time")
    exit()

print(f"\n✓ Found {len(available_servos)} servo(s): {available_servos}")

# Try to use the available servos
servos = []
for servo_id in available_servos:
    try:
        servo = LX16A(servo_id)
        servo.set_angle_limits(0, 240)
        servos.append(servo)
        print(f"✓ Servo {servo_id} ready")
    except Exception as e:
        print(f"✗ Failed to setup servo {servo_id}: {e}")

if len(servos) == 0:
    print("❌ No servos could be initialized")
    exit()

print(f"\n🎯 Starting movement with {len(servos)} servo(s)...")
print("Press Ctrl+C to stop")

t = 0
try:
    while True:
        # Move servos in a sine wave pattern
        for i, servo in enumerate(servos):
            angle = sin(t + i * 1.57) * 60 + 120  # Offset each servo
            servo.move(angle)
        
        time.sleep(0.05)
        t += 0.1
        
except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    # Move servos to center position
    for servo in servos:
        try:
            servo.move(120)
        except:
            pass
    print("✓ Done!")

