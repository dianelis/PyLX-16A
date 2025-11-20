#!/usr/bin/env python3
"""
Dance Script - Simple script to make the robot dance
Usage: python dance.py or ./dance.py
"""

import sys
import os

# Add the current directory to the path to import from moving_left_and_right_legs
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from moving_left_and_right_legs import *

def main():
    """Main function to run dancing."""
    print("=" * 60)
    print("Robot Dance Control")
    print("=" * 60)
    
    # Initialize serial connection
    try:
        print(f"Connecting to {PORT}...")
        LX16A.initialize(PORT, BAUD_RATE)
        print("✓ Connected to servo bus")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        print("Check:")
        print(f"  - Serial port: {PORT}")
        print("  - USB connection")
        print("  - Servo power supply")
        return
    
    # Initialize all servos
    try:
        initialize_servos()
    except Exception as e:
        print(f"❌ Failed to initialize servos: {e}")
        return
    
    # Use try/finally to ensure we always return to neutral
    try:
        # Dance
        dance(duration_sec=15, t_ms=400)
        
        print("\n" + "=" * 60)
        print("Dancing complete!")
        print("=" * 60)
    
    finally:
        # Always return to neutral standing position
        print("\n🔄 Returning to neutral standing position...")
        try:
            set_pose(NEUTRAL, 800)
            time.sleep(0.5)
            print("✓ Returned to neutral standing position")
        except Exception as e:
            print(f"⚠️  Warning: Could not return to neutral: {e}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")
        # Try to return to neutral
        try:
            set_pose(NEUTRAL, 800)
        except:
            pass
        print("Exiting...")
    except Exception as e:
        print(f"\n\n❌ Unexpected error: {e}")
        # Try to return to neutral if possible
        try:
            set_pose(NEUTRAL, 800)
            print("✓ Returned to neutral after error")
        except:
            pass
        print("Exiting...")

