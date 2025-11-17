#!/usr/bin/env python3
"""
Left Leg Stepping Control for 3-Servo Robotic Leg
Uses LewanSoul LX-16A servos with forward kinematics and smooth stepping motion.
"""

from pylx16a.lx16a import *
import time
import math

# ============================================================================
# CONFIGURATION - Modify these values as needed
# ============================================================================

# Serial port configuration
SERIAL_PORT = "/dev/tty.usbserial-210"  # Change to your port (e.g., "/dev/tty.usbserial-210" on Mac)
BAUD_RATE = 0.1  # Timeout in seconds

# Servo IDs - Left Leg
LEFT_HIP = 30
LEFT_KNEE = 10  # Knee servo with big angle swing
LEFT_ANKLE = 20

# Servo IDs - Right Leg
RIGHT_HIP = 40
RIGHT_KNEE = 50
RIGHT_ANKLE = 60

# Neutral standing pose angles (degrees) - Separate for each leg
# Left Leg neutral angles (original values)
LEFT_HIP_NEUTRAL = 135.0
LEFT_KNEE_NEUTRAL = 135.0
LEFT_ANKLE_NEUTRAL = 135.0

# Right Leg neutral angles (current straight position)
RIGHT_HIP_NEUTRAL = 135.0  # Will be updated from right leg's current position
RIGHT_KNEE_NEUTRAL = 135.0  # Will be updated from right leg's current position
RIGHT_ANKLE_NEUTRAL = 100.0  # Right leg ankle (servo 60) straight position

# Legacy variables for backward compatibility (will use left leg values)
HIP_NEUTRAL = LEFT_HIP_NEUTRAL
KNEE_NEUTRAL = LEFT_KNEE_NEUTRAL
ANKLE_NEUTRAL = LEFT_ANKLE_NEUTRAL

# Hip orientation offset (degrees) - compensates for sideways mounting
HIP_OFFSET = 0.0  # Adjust if hip rotation direction is inverted

# Segment lengths (mm)
L1_HIP_TO_KNEE = 70.0
L2_KNEE_TO_ANKLE = 70.0
L3_ANKLE_TO_FOOT = 30.0

# Joint limits relative to neutral (degrees)
HIP_LIMIT_MIN = -35.0  # hip_neutral - 35 (slightly wider for natural swing)
HIP_LIMIT_MAX = 35.0   # hip_neutral + 35
KNEE_LIMIT_MIN = -60.0  # knee_neutral - 60 (increased for big angle swing)
KNEE_LIMIT_MAX = 60.0   # knee_neutral + 60 (increased for big angle swing)
ANKLE_LIMIT_MIN = -50.0  # ankle_neutral - 50 (increased for bigger angle swing)
ANKLE_LIMIT_MAX = 50.0   # ankle_neutral + 50 (increased for bigger angle swing)

# Default stepping parameters
DEFAULT_STEP_HEIGHT_MM = 45.0  # Increased for larger steps
DEFAULT_STEP_LENGTH_MM = 70.0  # Increased for larger steps
DEFAULT_STEP_DURATION_SEC = 1.0  # Twice as fast (was 2.0)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def clamp_angle(angle, min_angle, max_angle):
    """Clamp an angle to be within specified limits."""
    return max(min_angle, min(max_angle, angle))


def cosine_ease(t):
    """
    Cosine easing function for smooth interpolation.
    Provides smooth acceleration and deceleration.
    
    Args:
        t: Interpolation parameter from 0.0 to 1.0
    
    Returns:
        Eased value from 0.0 to 1.0 with smooth cosine curve
    """
    return 0.5 * (1.0 - math.cos(math.pi * t))


def set_servo_angle(servo_id, angle_deg, speed=None):
    """
    Set a servo to a specific angle.
    
    Args:
        servo_id: The servo ID (10, 20, or 30)
        angle_deg: Target angle in degrees (0-240)
        speed: Optional movement speed (not used in LX-16A, but kept for API consistency)
    """
    try:
        servo = LX16A(servo_id)
        # Clamp to valid servo range
        angle_deg = max(0.0, min(240.0, angle_deg))
        servo.move(angle_deg)
        return True
    except ServoTimeoutError:
        print(f"Warning: Servo {servo_id} not responding")
        return False
    except Exception as e:
        print(f"Error setting servo {servo_id}: {e}")
        return False


def move_to_pose(hip_id, knee_id, ankle_id, hip_angle, knee_angle, ankle_angle, duration_sec=1.0, 
                 hip_neutral=None, knee_neutral=None, ankle_neutral=None):
    """
    Move all three servos to specified angles simultaneously.
    
    Args:
        hip_id: Hip servo ID
        knee_id: Knee servo ID
        ankle_id: Ankle servo ID
        hip_angle: Hip angle in degrees (relative to neutral)
        knee_angle: Knee angle in degrees (relative to neutral)
        ankle_angle: Ankle angle in degrees (relative to neutral)
        duration_sec: Time to complete the movement
        hip_neutral: Neutral angle for hip (defaults to HIP_NEUTRAL)
        knee_neutral: Neutral angle for knee (defaults to KNEE_NEUTRAL)
        ankle_neutral: Neutral angle for ankle (defaults to ANKLE_NEUTRAL)
    """
    # Use provided neutral angles or defaults
    if hip_neutral is None:
        hip_neutral = HIP_NEUTRAL
    if knee_neutral is None:
        knee_neutral = KNEE_NEUTRAL
    if ankle_neutral is None:
        ankle_neutral = ANKLE_NEUTRAL
    
    # Apply neutral offsets and hip offset
    hip_actual = hip_neutral + hip_angle + HIP_OFFSET
    knee_actual = knee_neutral + knee_angle
    ankle_actual = ankle_neutral + ankle_angle
    
    # Apply joint limits (using the provided neutral angles)
    hip_actual = clamp_angle(hip_actual, 
                            hip_neutral + HIP_LIMIT_MIN, 
                            hip_neutral + HIP_LIMIT_MAX)
    knee_actual = clamp_angle(knee_actual, 
                              knee_neutral + KNEE_LIMIT_MIN, 
                              knee_neutral + KNEE_LIMIT_MAX)
    ankle_actual = clamp_angle(ankle_actual, 
                              ankle_neutral + ANKLE_LIMIT_MIN, 
                              ankle_neutral + ANKLE_LIMIT_MAX)
    
    # Move servos
    set_servo_angle(hip_id, hip_actual)
    set_servo_angle(knee_id, knee_actual)
    set_servo_angle(ankle_id, ankle_actual)
    
    # Wait for movement to complete
    time.sleep(duration_sec)


# ============================================================================
# FORWARD KINEMATICS
# ============================================================================

def forward_kinematics(hip_angle_deg, knee_angle_deg, ankle_angle_deg):
    """
    Compute foot position (x, z) relative to hip using forward kinematics.
    
    Args:
        hip_angle_deg: Hip angle in degrees (relative to vertical, positive = forward)
        knee_angle_deg: Knee angle in degrees (relative to straight, positive = bent)
        ankle_angle_deg: Ankle angle in degrees (relative to straight, positive = flexed)
    
    Returns:
        (x, z) tuple: Foot position relative to hip in mm
            x: Forward distance (positive = forward)
            z: Vertical distance (positive = down)
    """
    # Convert to radians
    hip_rad = math.radians(hip_angle_deg)
    knee_rad = math.radians(knee_angle_deg)
    ankle_rad = math.radians(ankle_angle_deg)
    
    # Compute joint positions
    # Hip is at origin (0, 0)
    # Knee position relative to hip
    knee_x = L1_HIP_TO_KNEE * math.sin(hip_rad)
    knee_z = L1_HIP_TO_KNEE * math.cos(hip_rad)
    
    # Ankle position relative to knee
    # Total angle from vertical at knee = hip_angle + knee_angle
    ankle_angle_from_vertical = hip_rad + knee_rad
    ankle_x = knee_x + L2_KNEE_TO_ANKLE * math.sin(ankle_angle_from_vertical)
    ankle_z = knee_z + L2_KNEE_TO_ANKLE * math.cos(ankle_angle_from_vertical)
    
    # Foot position relative to ankle
    # Total angle from vertical at ankle = hip_angle + knee_angle + ankle_angle
    foot_angle_from_vertical = ankle_angle_from_vertical + ankle_rad
    foot_x = ankle_x + L3_ANKLE_TO_FOOT * math.sin(foot_angle_from_vertical)
    foot_z = ankle_z + L3_ANKLE_TO_FOOT * math.cos(foot_angle_from_vertical)
    
    return (foot_x, foot_z)


# ============================================================================
# LEFT LEG CLASS
# ============================================================================

class Leg:
    """Controls a 3-servo robotic leg with forward kinematics and stepping."""
    
    def __init__(self, hip_id, knee_id, ankle_id, leg_name="Leg", hip_neutral=None, knee_neutral=None, ankle_neutral=None):
        """Initialize the leg and connect to servos.
        
        Args:
            hip_id: Hip servo ID
            knee_id: Knee servo ID
            ankle_id: Ankle servo ID
            leg_name: Name of the leg (e.g., "Left Leg" or "Right Leg")
            hip_neutral: Neutral angle for hip (defaults to LEFT_HIP_NEUTRAL or RIGHT_HIP_NEUTRAL based on leg)
            knee_neutral: Neutral angle for knee (defaults to LEFT_KNEE_NEUTRAL or RIGHT_KNEE_NEUTRAL based on leg)
            ankle_neutral: Neutral angle for ankle (defaults to LEFT_ANKLE_NEUTRAL or RIGHT_ANKLE_NEUTRAL based on leg)
        """
        self.hip_id = hip_id
        self.knee_id = knee_id
        self.ankle_id = ankle_id
        self.leg_name = leg_name
        
        # Set neutral angles based on leg type or use provided values
        if hip_neutral is None:
            self.hip_neutral = RIGHT_HIP_NEUTRAL if "Right" in leg_name else LEFT_HIP_NEUTRAL
        else:
            self.hip_neutral = hip_neutral
            
        if knee_neutral is None:
            self.knee_neutral = RIGHT_KNEE_NEUTRAL if "Right" in leg_name else LEFT_KNEE_NEUTRAL
        else:
            self.knee_neutral = knee_neutral
            
        if ankle_neutral is None:
            self.ankle_neutral = RIGHT_ANKLE_NEUTRAL if "Right" in leg_name else LEFT_ANKLE_NEUTRAL
        else:
            self.ankle_neutral = ankle_neutral
        
        self.hip_servo = None
        self.knee_servo = None
        self.ankle_servo = None
        self._initialize_servos()
    
    def _initialize_servos(self):
        """Initialize servo connections and set angle limits."""
        try:
            # Initialize knee and ankle (required)
            self.knee_servo = LX16A(self.knee_id)
            self.ankle_servo = LX16A(self.ankle_id)
            
            # Set angle limits for knee and ankle
            self.knee_servo.set_angle_limits(0, 240)
            self.ankle_servo.set_angle_limits(0, 240)
            
            # Try to initialize hip (optional - may be disabled)
            try:
                self.hip_servo = LX16A(self.hip_id)
                self.hip_servo.set_angle_limits(0, 240)
                print(f"✓ {self.leg_name} servos initialized (hip, knee, ankle)")
            except ServoTimeoutError:
                self.hip_servo = None
                print(f"✓ {self.leg_name} knee and ankle initialized (hip not responding - will stay neutral)")
            
            # Read current servo positions to use as neutral
            self._read_current_positions()
            
            return True
        except ServoTimeoutError as e:
            print(f"❌ {self.leg_name} required servo {e.id_} not responding. Check connections.")
            return False
        except Exception as e:
            print(f"❌ Error initializing {self.leg_name} servos: {e}")
            return False
    
    def _read_current_positions(self):
        """Read current servo positions and display them."""
        print(f"\n📊 {self.leg_name} Current Servo Positions:")
        try:
            if self.knee_servo is not None:
                knee_pos = self.knee_servo.get_physical_angle()
                print(f"  Servo {self.knee_id} (Knee): {knee_pos:.2f}°")
            if self.ankle_servo is not None:
                ankle_pos = self.ankle_servo.get_physical_angle()
                print(f"  Servo {self.ankle_id} (Ankle): {ankle_pos:.2f}°")
            if self.hip_servo is not None:
                hip_pos = self.hip_servo.get_physical_angle()
                print(f"  Servo {self.hip_id} (Hip): {hip_pos:.2f}°")
        except Exception as e:
            print(f"  ⚠️  Could not read current positions: {e}")
    
    def go_to_neutral(self, duration_sec=1.0):
        """
        Move leg to neutral standing pose (true vertical 90-degree position).
        
        Args:
            duration_sec: Time to complete the movement
        """
        print(f"Moving {self.leg_name} to neutral pose...")
        # Ensure all joints are at exactly 0.0 relative to neutral
        move_to_pose(self.hip_id, self.knee_id, self.ankle_id, 0.0, 0.0, 0.0, duration_sec,
                     self.hip_neutral, self.knee_neutral, self.ankle_neutral)
        # Double-check by directly setting servos to neutral angles for precision
        time.sleep(0.1)  # Small delay to ensure previous command completes
        try:
            if self.knee_servo is not None:
                self.knee_servo.move(self.knee_neutral)
            if self.ankle_servo is not None:
                self.ankle_servo.move(self.ankle_neutral)
            if self.hip_servo is not None:
                self.hip_servo.move(self.hip_neutral)
        except:
            pass
        print(f"✓ {self.leg_name} neutral pose reached (vertical standing position)")
    
    def step_forward_once(self, step_height_mm=DEFAULT_STEP_HEIGHT_MM, 
                         step_length_mm=DEFAULT_STEP_LENGTH_MM, 
                         step_duration_sec=DEFAULT_STEP_DURATION_SEC):
        """
        Execute a single stepping motion with sequential forward swings.
        
        The step consists of 3 phases:
        1. Knee Forward Swing: Servo 10 (knee) moves first with big swing forward
        2. Ankle Forward Swing: Servo 20 (ankle) swings forward right after knee
        3. Return to Neutral: All servos return to straight 90-degree position
        
        Motion pattern:
        - Hip servo: Does not move (stays at neutral)
        - Knee servo: First big swing forward (50° relative)
        - Ankle servo: Right after knee, big swing forward (45° relative)
        - Then all servos return to neutral (straight 90° position)
        
        Args:
            step_height_mm: Not used - kept for API compatibility
            step_length_mm: Not used - kept for API compatibility
            step_duration_sec: Total duration of the step (seconds)
        """
        print(f"Stepping with sequential forward swings...")
        
        # Increased number of interpolation steps for smooth motion
        num_steps = 100
        dt = step_duration_sec / num_steps
        
        # Motion pattern:
        # 1. Servo 30 (hip) - do not move (stay at neutral/0)
        # 2. Servo 10 (knee) - first move big swing forward
        # 3. Servo 20 (ankle) - right after knee, also big swing forward
        # 4. Then all servos get straight to 90 degree angle (neutral position)
        
        # Phase 1: Knee swings forward first (big angle swing)
        knee_forward_angle = 50.0  # Big swing forward (relative to neutral, 135+50=185 absolute)
        
        # Phase 2: Ankle swings forward right after knee (big angle swing)
        ankle_forward_angle = 45.0  # Big swing forward (relative to neutral, 135+45=180 absolute)
        
        # Phase 3: Return to neutral (straight 90-degree position)
        # All servos return to exactly 0.0 relative (which is 135° absolute = 90° physical)
        # Ensure we explicitly set to neutral angles
        
        # Define keyframes following the motion pattern
        # Format: (hip_angle, knee_angle, ankle_angle, phase_name, segment_weight)
        # Hip (servo 30) stays at 0.0 (neutral) throughout
        keyframes = [
            # Start from neutral (straight 90-degree position)
            (0.0, 0.0, 0.0, "Neutral Start", 0.10),  # 10% of steps
            
            # Phase 1: Knee (servo 10) swings forward first (big swing)
            (0.0, knee_forward_angle, 0.0, "Knee Forward Swing", 0.30),  # 30% of steps
            
            # Phase 2: Ankle (servo 20) swings forward right after knee (big swing)
            (0.0, knee_forward_angle, ankle_forward_angle, "Ankle Forward Swing", 0.30),  # 30% of steps
            
            # Phase 3: Return to neutral (straight 90-degree position)
            # Explicitly set all to 0.0 to ensure straight position
            (0.0, 0.0, 0.0, "Return to Neutral", 0.30),  # 30% of steps
        ]
        
        # After interpolation, explicitly set servos to exact neutral position
        # This ensures the leg is truly straight
        
        # Calculate step distribution for each segment (between keyframes)
        # Use the weight of the destination keyframe for each segment
        segment_weights = [keyframes[i+1][4] for i in range(len(keyframes) - 1)]
        total_weight = sum(segment_weights)
        segment_steps_list = [int(num_steps * weight / total_weight) for weight in segment_weights]
        
        # Adjust to ensure total steps match (handle rounding)
        current_total = sum(segment_steps_list)
        if current_total < num_steps:
            segment_steps_list[-1] += (num_steps - current_total)
        elif current_total > num_steps:
            segment_steps_list[-1] -= (current_total - num_steps)
        
        # Interpolate between keyframes with cosine easing
        for i in range(len(keyframes) - 1):
            start_frame = keyframes[i]
            end_frame = keyframes[i + 1]
            
            # Number of steps for this segment
            segment_steps = segment_steps_list[i]
            
            for j in range(segment_steps):
                # Linear interpolation parameter (0.0 to 1.0)
                t_linear = j / segment_steps if segment_steps > 0 else 0.0
                
                # Apply cosine easing for smooth acceleration/deceleration
                t_eased = cosine_ease(t_linear)
                
                # Interpolate angles with eased parameter
                hip_angle = start_frame[0] + (end_frame[0] - start_frame[0]) * t_eased
                knee_angle = start_frame[1] + (end_frame[1] - start_frame[1]) * t_eased
                ankle_angle = start_frame[2] + (end_frame[2] - start_frame[2]) * t_eased
                
                # Move to interpolated pose (using this leg's neutral angles)
                move_to_pose(self.hip_id, self.knee_id, self.ankle_id, hip_angle, knee_angle, ankle_angle, dt,
                             self.hip_neutral, self.knee_neutral, self.ankle_neutral)
                
                # Print phase name at start of segment
                if j == 0:
                    print(f"  → {end_frame[3]}")
        
        # After all interpolation, explicitly set all servos to exact neutral position
        # This ensures the leg returns to a perfectly straight 90-degree position
        time.sleep(0.1)  # Small delay to ensure previous commands complete
        try:
            if self.knee_servo is not None:
                self.knee_servo.move(self.knee_neutral)
            if self.ankle_servo is not None:
                self.ankle_servo.move(self.ankle_neutral)
            if self.hip_servo is not None:
                self.hip_servo.move(self.hip_neutral)
            time.sleep(0.2)  # Allow servos to settle into position
        except Exception as e:
            print(f"Warning: Could not set final neutral position: {e}")
        
        print(f"✓ {self.leg_name} step complete - leg returned to straight position")


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """Main function to run the stepping demonstration."""
    print("=" * 60)
    print("Dual Leg Stepping Control")
    print("=" * 60)
    
    # Initialize serial connection
    try:
        print(f"Connecting to {SERIAL_PORT}...")
        LX16A.initialize(SERIAL_PORT, BAUD_RATE)
        print("✓ Connected to servo bus")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        print("Check:")
        print(f"  - Serial port: {SERIAL_PORT}")
        print("  - USB connection")
        print("  - Servo power supply")
        return
    
    # Use try/finally to ensure we always return to neutral
    try:
        # Create leg controllers for both legs
        left_leg = Leg(LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, "Left Leg")
        right_leg = Leg(RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE, "Right Leg")
        
        # Check that required servos (knee and ankle) are initialized for both legs
        if left_leg.knee_servo is None or left_leg.ankle_servo is None:
            print("❌ Failed to initialize left leg servos (knee and ankle). Exiting.")
            return
        if right_leg.knee_servo is None or right_leg.ankle_servo is None:
            print("❌ Failed to initialize right leg servos (knee and ankle). Exiting.")
            return
        
        if left_leg.hip_servo is None:
            print("ℹ️  Left leg hip servo not available - will use knee and ankle only")
        if right_leg.hip_servo is None:
            print("ℹ️  Right leg hip servo not available - will use knee and ankle only")
        
        # Read and display current servo positions first
        print("\n📊 Reading current servo positions...")
        left_leg._read_current_positions()
        right_leg._read_current_positions()
        
        # Read right leg's current positions to use as neutral (straight position)
        print("\n📐 Reading Right Leg's current straight position to use as neutral...")
        # Declare global at the start
        global RIGHT_HIP_NEUTRAL, RIGHT_KNEE_NEUTRAL, RIGHT_ANKLE_NEUTRAL
        try:
            right_hip_pos = right_leg.hip_servo.get_physical_angle() if right_leg.hip_servo is not None else RIGHT_HIP_NEUTRAL
            right_knee_pos = right_leg.knee_servo.get_physical_angle() if right_leg.knee_servo is not None else RIGHT_KNEE_NEUTRAL
            right_ankle_pos = right_leg.ankle_servo.get_physical_angle() if right_leg.ankle_servo is not None else RIGHT_ANKLE_NEUTRAL
            
            print(f"  Right Leg Straight Position:")
            print(f"    Hip (ID {RIGHT_HIP}): {right_hip_pos:.2f}°")
            print(f"    Knee (ID {RIGHT_KNEE}): {right_knee_pos:.2f}°")
            print(f"    Ankle (ID {RIGHT_ANKLE}): {right_ankle_pos:.2f}°")
            print(f"\n  → Using these as neutral angles for right leg")
            print(f"  → Update RIGHT_HIP_NEUTRAL={right_hip_pos:.1f}, RIGHT_KNEE_NEUTRAL={right_knee_pos:.1f}, RIGHT_ANKLE_NEUTRAL={right_ankle_pos:.1f} in code")
            
            # Update right leg neutral values dynamically based on current position
            RIGHT_HIP_NEUTRAL = right_hip_pos
            RIGHT_KNEE_NEUTRAL = right_knee_pos
            RIGHT_ANKLE_NEUTRAL = right_ankle_pos
            
            # Update the right leg object's neutral angles
            right_leg.hip_neutral = right_hip_pos
            right_leg.knee_neutral = right_knee_pos
            right_leg.ankle_neutral = right_ankle_pos
            
            print(f"  ✓ Right leg neutral angles updated to match current straight position")
        except Exception as e:
            print(f"  ⚠️  Could not read right leg positions: {e}")
            print(f"  → Using default neutral angles: HIP={RIGHT_HIP_NEUTRAL}, KNEE={RIGHT_KNEE_NEUTRAL}, ANKLE={RIGHT_ANKLE_NEUTRAL}")
        
        # Move both legs to neutral pose (using the updated neutral values)
        left_leg.go_to_neutral(duration_sec=1.0)
        right_leg.go_to_neutral(duration_sec=1.0)
        time.sleep(0.5)  # Shorter pause
        
        # Read positions again after moving to neutral
        print("\n📊 Servo positions after moving to neutral:")
        left_leg._read_current_positions()
        right_leg._read_current_positions()
        
        # Perform several steps - alternating between legs
        num_steps = 4  # 2 steps per leg
        print(f"\nPerforming {num_steps} steps (alternating legs)...")
        print("-" * 60)
        
        for i in range(num_steps):
            if i % 2 == 0:
                # Left leg steps
                print(f"\nStep {i + 1}/{num_steps} - Left Leg")
                left_leg.step_forward_once(
                    step_height_mm=45.0,
                    step_length_mm=70.0,
                    step_duration_sec=1.0
                )
            else:
                # Right leg steps
                print(f"\nStep {i + 1}/{num_steps} - Right Leg")
                right_leg.step_forward_once(
                    step_height_mm=45.0,
                    step_length_mm=70.0,
                    step_duration_sec=1.0
                )
            time.sleep(0.25)  # Shorter pause between steps
        
        # Return both legs to neutral
        print("\nReturning both legs to neutral...")
        left_leg.go_to_neutral(duration_sec=1.0)
        right_leg.go_to_neutral(duration_sec=1.0)
        
        print("\n" + "=" * 60)
        print("Demo complete!")
        print("=" * 60)
    
    finally:
        # Always return both legs to neutral standing position (true vertical)
        print("\n🔄 Returning both legs to neutral standing position (vertical)...")
        try:
            # Try to return both legs to neutral
            try:
                left_leg.go_to_neutral(duration_sec=1.5)
            except:
                pass
            try:
                right_leg.go_to_neutral(duration_sec=1.5)
            except:
                pass
            
            time.sleep(0.2)  # Allow servos to settle
            
            # Double-check with direct servo commands to ensure exact position
            try:
                if left_leg.knee_servo is not None:
                    left_leg.knee_servo.move(left_leg.knee_neutral)
                if left_leg.ankle_servo is not None:
                    left_leg.ankle_servo.move(left_leg.ankle_neutral)
                if left_leg.hip_servo is not None:
                    left_leg.hip_servo.move(left_leg.hip_neutral)
            except:
                pass
            
            try:
                if right_leg.knee_servo is not None:
                    right_leg.knee_servo.move(right_leg.knee_neutral)
                if right_leg.ankle_servo is not None:
                    right_leg.ankle_servo.move(right_leg.ankle_neutral)
                if right_leg.hip_servo is not None:
                    right_leg.hip_servo.move(right_leg.hip_neutral)
            except:
                pass
            
            time.sleep(0.3)  # Final settle time
            print("✓ Both legs returned to neutral standing position (vertical)")
        except Exception as e:
            print(f"⚠️  Warning: Could not return legs to neutral: {e}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")
        # The finally block in main() will handle returning to neutral
        print("Exiting...")
    except Exception as e:
        print(f"\n\n❌ Unexpected error: {e}")
        # Try to return to neutral if possible
        try:
            LX16A.initialize(SERIAL_PORT, BAUD_RATE)
            left_leg = Leg(LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, "Left Leg")
            right_leg = Leg(RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE, "Right Leg")
            if left_leg.knee_servo is not None and left_leg.ankle_servo is not None:
                left_leg.go_to_neutral(duration_sec=1.0)
            if right_leg.knee_servo is not None and right_leg.ankle_servo is not None:
                right_leg.go_to_neutral(duration_sec=1.0)
            print("✓ Returned to neutral after error")
        except:
            pass
        print("Exiting...")

