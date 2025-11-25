#!/usr/bin/env python3
"""
Simple servo test - Connect ONE servo and run this
"""

from pylx16a.lx16a import *
import time

print("🔧 Single Servo Test")
print("=" * 40)
print("Make sure:")
print("1. Only ONE servo is connected")
print("2. Servo has 6-12V power")
print("3. All wires are connected properly")
print("=" * 40)

try:
    LX16A.initialize("/dev/tty.usbserial-210", 0.5)
    print("✓ Connected to controller")
    
    # Try different IDs
    for servo_id in range(1, 6):
        try:
            print(f"\nTesting Servo ID {servo_id}...")
            servo = LX16A(servo_id)
            
            # Get servo info
            angle = servo.get_physical_angle()
            voltage = servo.get_vin()
            temp = servo.get_temp()
            
            print(f"🎯 SUCCESS! Servo ID {servo_id} found!")
            print(f"   Current angle: {angle:.2f}°")
            print(f"   Voltage: {voltage/1000:.2f}V")
            print(f"   Temperature: {temp}°C")
            
            # Test movement
            print("   Testing movement...")
            servo.move(0)
            time.sleep(1)
            servo.move(120)
            time.sleep(1)
            servo.move(240)
            time.sleep(1)
            servo.move(120)
            print("   ✓ Movement test passed!")
            
            print(f"\n🎉 Servo {servo_id} is working! You can now:")
            print("1. Connect the second servo")
            print("2. Assign it a different ID")
            print("3. Run your hello-world script")
            
            break
            
        except ServoTimeoutError:
            print(f"   ✗ Servo ID {servo_id}: Not responding")
        except Exception as e:
            print(f"   ✗ Servo ID {servo_id}: {e}")
    
    else:
        print("\n❌ No servos found!")
        print("\nTroubleshooting:")
        print("1. Check power supply (6-12V)")
        print("2. Verify all 4 wires are connected")
        print("3. Try a different servo")
        print("4. Check USB connection")
        print("5. Try different servo IDs")

except Exception as e:
    print(f"❌ Connection error: {e}")
    print("Check USB connection and port")

