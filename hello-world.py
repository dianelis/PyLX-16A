from math import sin, cos
from pylx16a.lx16a import *
import time
import serial.serialutil

# Initialize servos with error handling
def initialize_servos():
    try:
        LX16A.initialize("/dev/tty.usbserial-210", 0.1)
        print("✓ Connected to servo controller")
        
        servo1 = LX16A(10)
        servo2 = LX16A(20)
        servo1.set_angle_limits(0, 240)
        servo2.set_angle_limits(0, 240)
        
        print("✓ Both servos initialized successfully")
        return servo1, servo2
        
    except ServoTimeoutError as e:
        print(f"❌ Servo {e.id_} is not responding. Check connections and power.")
        return None, None
    except serial.serialutil.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print("Check USB connection and port availability")
        return None, None
    except Exception as e:
        print(f"❌ Unexpected error during initialization: {e}")
        return None, None

# Safe servo movement with error handling
def safe_move(servo, angle, servo_name):
    try:
        servo.move(angle)
        return True
    except ServoTimeoutError:
        print(f"⚠️  {servo_name} disconnected! Attempting to reconnect...")
        return False
    except serial.serialutil.SerialException:
        print(f"⚠️  Serial communication lost with {servo_name}")
        return False
    except Exception as e:
        print(f"⚠️  Error moving {servo_name}: {e}")
        return False

# Reconnect servos if needed
def reconnect_servos():
    print("🔄 Attempting to reconnect servos...")
    try:
        # Reinitialize the connection
        LX16A.initialize("/dev/tty.usbserial-210", 0.1)
        
        servo1 = LX16A(10)
        servo2 = LX16A(20)
        servo1.set_angle_limits(0, 240)
        servo2.set_angle_limits(0, 240)
        
        print("✓ Servos reconnected successfully")
        return servo1, servo2
        
    except Exception as e:
        print(f"❌ Reconnection failed: {e}")
        return None, None

# Main execution
servo1, servo2 = initialize_servos()

if servo1 is None or servo2 is None:
    print("❌ Failed to initialize servos. Exiting...")
    exit()

print("🎯 Starting synchronized movement...")
print("Press Ctrl+C to stop")

t = 0
disconnect_count = 0
max_disconnect_attempts = 3

try:
    while True:
        # Calculate angles
        angle1 = sin(t) * 60 + 120
        angle2 = cos(t) * 60 + 120
        
        # Move servos with error handling
        servo1_ok = safe_move(servo1, angle1, "Servo 10")
        servo2_ok = safe_move(servo2, angle2, "Servo 20")
        
        # Check if any servo disconnected
        if not servo1_ok or not servo2_ok:
            disconnect_count += 1
            print(f"⚠️  Disconnect detected (attempt {disconnect_count}/{max_disconnect_attempts})")
            
            if disconnect_count >= max_disconnect_attempts:
                print("❌ Too many disconnections. Stopping...")
                break
            
            # Try to reconnect
            servo1, servo2 = reconnect_servos()
            if servo1 is None or servo2 is None:
                print("❌ Failed to reconnect. Stopping...")
                break
            
            disconnect_count = 0  # Reset counter on successful reconnection
            print("✓ Continuing movement...")
        
        time.sleep(0.05)
        t += 0.1
        
except KeyboardInterrupt:
    print("\n🛑 Stopping movement...")
    
    # Safely stop servos
    try:
        servo1.move(120)  # Center position
        servo2.move(120)  # Center position
        print("✓ Servos moved to center position")
    except:
        print("⚠️  Could not move servos to center (may be disconnected)")
    
    print("✓ Program stopped safely")
