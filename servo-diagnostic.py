#!/usr/bin/env python3
"""
LX-16A Servo Diagnostic Tool
This script helps diagnose issues with servo connections
"""

from pylx16a.lx16a import *
import time

def scan_servos(port, timeout=0.1):
    """Scan for connected servos on the given port"""
    print(f"Scanning for servos on {port}...")
    print("=" * 50)
    
    try:
        LX16A.initialize(port, timeout)
        print(f"✓ Successfully connected to {port}")
    except Exception as e:
        print(f"✗ Failed to connect to {port}: {e}")
        return []
    
    connected_servos = []
    
    # Scan IDs 1-10
    for i in range(1, 11):
        try:
            servo = LX16A(i)
            # Try to get basic info to confirm it's responsive
            angle = servo.get_physical_angle()
            voltage = servo.get_vin()
            temp = servo.get_temp()
            
            print(f"✓ Servo ID {i}: Connected")
            print(f"  - Current angle: {angle:.2f}°")
            print(f"  - Voltage: {voltage/1000:.2f}V")
            print(f"  - Temperature: {temp}°C")
            print()
            
            connected_servos.append(i)
            
        except ServoTimeoutError:
            print(f"✗ Servo ID {i}: Not responding")
        except Exception as e:
            print(f"✗ Servo ID {i}: Error - {e}")
    
    print("=" * 50)
    print(f"Total servos found: {len(connected_servos)}")
    print(f"Connected servo IDs: {connected_servos}")
    
    return connected_servos

def test_servo_movement(servo_id, port):
    """Test basic servo movement"""
    print(f"\nTesting movement for Servo ID {servo_id}...")
    
    try:
        servo = LX16A(servo_id)
        
        # Test different positions
        positions = [0, 60, 120, 180, 240]
        
        for pos in positions:
            print(f"Moving to {pos}°...")
            servo.move(pos)
            time.sleep(1)
            
            # Read back position
            actual_pos = servo.get_physical_angle()
            print(f"  Actual position: {actual_pos:.2f}°")
            
    except Exception as e:
        print(f"Error testing servo {servo_id}: {e}")

if __name__ == "__main__":
    # Try different common ports
    ports_to_try = [
        "/dev/tty.usbserial-210",  # Your current port
        "/dev/tty.usbserial-*",   # Generic USB serial
        "/dev/tty.usbmodem*",     # USB modem
        "/dev/ttyUSB0",           # Linux style
    ]
    
    print("LX-16A Servo Diagnostic Tool")
    print("=" * 50)
    
    # First, let's try your specific port
    port = "/dev/tty.usbserial-210"
    connected_servos = scan_servos(port)
    
    if len(connected_servos) == 0:
        print("\nNo servos found. Troubleshooting tips:")
        print("1. Check USB connection")
        print("2. Verify servo power supply")
        print("3. Check servo IDs (default is usually 1)")
        print("4. Try different USB port")
        print("5. Check if another program is using the port")
    elif len(connected_servos) == 1:
        print(f"\nOnly one servo found (ID: {connected_servos[0]})")
        print("To add a second servo:")
        print("1. Connect the second servo to the bus")
        print("2. Assign it a unique ID (different from the first)")
        print("3. Ensure proper power supply for both servos")
        print("4. Check wiring connections")
        
        # Test the single servo
        test_servo_movement(connected_servos[0], port)
    else:
        print(f"\nMultiple servos found! Testing movement...")
        for servo_id in connected_servos:
            test_servo_movement(servo_id, port)
            time.sleep(2)

