#!/usr/bin/env python3
"""
Single Servo Test - Connect one servo at a time to isolate issues
"""

from pylx16a.lx16a import *
import time

def test_single_servo():
    print("Single Servo Test")
    print("=" * 40)
    print("INSTRUCTIONS:")
    print("1. Connect ONLY ONE servo to the bus")
    print("2. Make sure it has proper power (6-12V)")
    print("3. Check all wiring connections")
    print("4. Run this test")
    print("=" * 40)
    
    try:
        LX16A.initialize('/dev/tty.usbserial-210', 0.5)
        print("✓ Connected to port")
        
        # Try different IDs
        for servo_id in range(1, 6):
            try:
                print(f"\nTesting Servo ID {servo_id}...")
                servo = LX16A(servo_id)
                
                # Get basic info
                angle = servo.get_physical_angle()
                voltage = servo.get_vin()
                temp = servo.get_temp()
                
                print(f"✓ Servo ID {servo_id} found!")
                print(f"  - Current angle: {angle:.2f}°")
                print(f"  - Voltage: {voltage/1000:.2f}V")
                print(f"  - Temperature: {temp}°C")
                
                # Test movement
                print("  - Testing movement...")
                servo.move(0)
                time.sleep(0.5)
                servo.move(120)
                time.sleep(0.5)
                servo.move(240)
                time.sleep(0.5)
                servo.move(120)
                print("  ✓ Movement test passed!")
                
                return servo_id
                
            except ServoTimeoutError:
                print(f"✗ Servo ID {servo_id}: Not responding")
            except Exception as e:
                print(f"✗ Servo ID {servo_id}: {e}")
        
        print("\nNo servos found. Check:")
        print("- Power supply (6-12V)")
        print("- Wiring connections")
        print("- USB connection")
        print("- Try a different servo")
        
    except Exception as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    test_single_servo()

