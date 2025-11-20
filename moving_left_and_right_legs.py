#!/usr/bin/env python3
"""
Moving Left and Right Legs - Dual Leg Stepping Control
Controls both left and right robotic legs with LewanSoul LX-16A servos.
Uses coordinated pose-based keyframes for smooth bipedal walking.
"""

from pylx16a.lx16a import *
import time
import math

# ============================================================================
# CONFIGURATION - Modify these values as needed
# ============================================================================

# Serial port configuration
PORT = "/dev/tty.usbserial-210"  # Change to your port (e.g., "/dev/tty.usbserial-210" on Mac)
SERIAL_PORT = PORT  # Alias for compatibility
BAUD_RATE = 0.1  # Timeout in seconds

# Servo IDs - Left Leg (from our view, left to right, bottom to top)
LEFT_ANKLE = 40  # Bottom
LEFT_KNEE = 10   # Middle
LEFT_HIP = 20    # Top

# Servo IDs - Right Leg (from our view, left to right, bottom to top)
RIGHT_ANKLE = 30  # Bottom
RIGHT_KNEE = 60   # Middle
RIGHT_HIP = 50    # Top

# Neutral standing pose angles (degrees) - Saved from current position
# These are the actual angles where the robot stands straight
NEUTRAL = {
    LEFT_HIP:   215.3,
    LEFT_KNEE:  123.6,
    LEFT_ANKLE: 123.1,
    RIGHT_HIP:   234.0,
    RIGHT_KNEE:  107.3,
    RIGHT_ANKLE: 177.4,
}

# Legacy variables for backward compatibility
LEFT_HIP_NEUTRAL = NEUTRAL[LEFT_HIP]
LEFT_KNEE_NEUTRAL = NEUTRAL[LEFT_KNEE]
LEFT_ANKLE_NEUTRAL = NEUTRAL[LEFT_ANKLE]
RIGHT_HIP_NEUTRAL = NEUTRAL[RIGHT_HIP]
RIGHT_KNEE_NEUTRAL = NEUTRAL[RIGHT_KNEE]
RIGHT_ANKLE_NEUTRAL = NEUTRAL[RIGHT_ANKLE]
HIP_NEUTRAL = LEFT_HIP_NEUTRAL
KNEE_NEUTRAL = LEFT_KNEE_NEUTRAL
ANKLE_NEUTRAL = LEFT_ANKLE_NEUTRAL

# ============================================================================
# INITIALIZE SERVOS
# ============================================================================

# Initialize bus and servo objects
SERVOS = {}

def initialize_servos():
    """Initialize all servo objects."""
    global SERVOS
    SERVOS = {
        LEFT_ANKLE: LX16A(LEFT_ANKLE),
        LEFT_KNEE:  LX16A(LEFT_KNEE),
        LEFT_HIP:   LX16A(LEFT_HIP),
        RIGHT_ANKLE: LX16A(RIGHT_ANKLE),
        RIGHT_KNEE:  LX16A(RIGHT_KNEE),
        RIGHT_HIP:   LX16A(RIGHT_HIP),
    }
    print("✓ All servos initialized")

# ============================================================================
# POSE FUNCTIONS
# ============================================================================

def set_pose(pose, t_ms=600):
    """
    Move all servos to specified pose simultaneously.
    
    Args:
        pose: dict {servo_id: angle_deg}
        t_ms: move time in milliseconds
    """
    for sid, angle in pose.items():
        try:
            # Clamp angle to valid servo range (0-240 degrees)
            angle = max(0.0, min(240.0, angle))
            # Use move() method with time parameter for coordinated movement
            SERVOS[sid].move(angle, time=t_ms)
        except Exception as e:
            print(f"Warning: Could not move servo {sid}: {e}")
    # Give it time to reach the target
    time.sleep(t_ms / 1000.0 + 0.05)

def pose_from_neutral(deltas_dict=None, **deltas):
    """
    Create a pose as "neutral plus small deltas".
    
    Args:
        deltas_dict: Dictionary with servo_id: delta_angle pairs (can pass as dict)
        **deltas: Keyword arguments with servo_id: delta_angle pairs (alternative)
    
    Returns:
        dict: Complete pose with all servos at neutral + deltas
    """
    pose = dict(NEUTRAL)
    # Handle both dict argument and keyword arguments
    if deltas_dict is not None:
        for sid, delta in deltas_dict.items():
            if sid in pose:
                pose[sid] = NEUTRAL[sid] + delta
    else:
        for sid, delta in deltas.items():
            if sid in pose:
                pose[sid] = NEUTRAL[sid] + delta
    return pose

# ============================================================================
# WALKING SEQUENCE
# ============================================================================

# Left leg step sequence - Big knee and ankle movements, minimal hip
# Swapped: Left leg now uses what was the right leg sequence
LEFT_LEG_STEP_SEQUENCE = [
    # 1) Lift left leg: Bend knee and flex ankle to clear ground
    pose_from_neutral({
        # Right leg stays in neutral (supporting leg)
        RIGHT_HIP: 0,
        RIGHT_KNEE: 0,
        RIGHT_ANKLE: 0,
        LEFT_HIP: 0,      # Hip stays neutral
        LEFT_KNEE: +50,   # Big knee bend to lift leg
        LEFT_ANKLE: -40,  # Ankle flexes up to clear ground
    }),
    
    # 2) Swing left leg forward: Extend knee and ankle forward
    pose_from_neutral({
        # Right leg stays in neutral
        RIGHT_HIP: 0,
        RIGHT_KNEE: 0,
        RIGHT_ANKLE: 0,
        LEFT_HIP: 0,      # Hip stays neutral
        LEFT_KNEE: +30,   # Partially extend knee for forward swing
        LEFT_ANKLE: -20,  # Ankle extends forward
    }),
    
    # 3) Place left foot down: Extend knee and ankle to contact ground
    pose_from_neutral({
        # Right leg stays in neutral
        RIGHT_HIP: 0,
        RIGHT_KNEE: 0,
        RIGHT_ANKLE: 0,
        LEFT_HIP: 0,      # Hip stays neutral
        LEFT_KNEE: +10,   # Slight knee bend for landing
        LEFT_ANKLE: 0,    # Ankle neutral for ground contact
    }),
    
    # 4) Return to neutral
    NEUTRAL,
]

# Right leg step sequence - Big knee and ankle movements, minimal hip
# Swapped: Right leg now uses what was the left leg sequence
RIGHT_LEG_STEP_SEQUENCE = [
    # 1) Lift right leg: Bend knee and flex ankle to clear ground
    pose_from_neutral({
        RIGHT_HIP: 0,     # Hip stays neutral
        RIGHT_KNEE: +50,  # Big knee bend to lift leg
        RIGHT_ANKLE: -40, # Ankle flexes up to clear ground
        # Left leg stays in neutral (supporting leg)
        LEFT_HIP: 0,
        LEFT_KNEE: 0,
        LEFT_ANKLE: 0,
    }),
    
    # 2) Swing right leg forward: Extend knee and ankle forward
    pose_from_neutral({
        RIGHT_HIP: 0,     # Hip stays neutral
        RIGHT_KNEE: +30,  # Partially extend knee for forward swing
        RIGHT_ANKLE: -20, # Ankle extends forward
        # Left leg stays in neutral
        LEFT_HIP: 0,
        LEFT_KNEE: 0,
        LEFT_ANKLE: 0,
    }),
    
    # 3) Place right foot down: Extend knee and ankle to contact ground
    pose_from_neutral({
        RIGHT_HIP: 0,     # Hip stays neutral
        RIGHT_KNEE: +10,  # Slight knee bend for landing
        RIGHT_ANKLE: 0,  # Ankle neutral for ground contact
        # Left leg stays in neutral
        LEFT_HIP: 0,
        LEFT_KNEE: 0,
        LEFT_ANKLE: 0,
    }),
    
    # 4) Return to neutral
    NEUTRAL,
]

def walk(steps=6, t_ms=500):
    """
    Walk forward using alternating leg steps with big knee/ankle movements.
    
    Args:
        steps: Total number of steps (will alternate between legs)
        t_ms: Time per keyframe in milliseconds
    """
    # Go to neutral first
    print("Moving to neutral position...")
    set_pose(NEUTRAL, 800)
    time.sleep(0.5)
    
    print(f"Walking {steps} steps (alternating legs)...")
    for step_num in range(steps):
        # Alternate between left and right legs
        if step_num % 2 == 0:
            # Left leg step
            leg_name = "Left"
            sequence = LEFT_LEG_STEP_SEQUENCE
        else:
            # Right leg step
            leg_name = "Right"
            sequence = RIGHT_LEG_STEP_SEQUENCE
        
        print(f"  Step {step_num + 1}/{steps} - {leg_name} Leg")
        for keyframe_num, keyframe in enumerate(sequence):
            set_pose(keyframe, t_ms)
    
    # Return to neutral
    print("Returning to neutral...")
    set_pose(NEUTRAL, 800)
    print("✓ Walking complete")

# Alias for backward compatibility
walk_forward = walk

def dance(duration_sec=10, t_ms=400):
    """
    Dance with intuitive hip movements and rhythmic patterns.
    
    Args:
        duration_sec: Total dance duration in seconds
        t_ms: Time per keyframe in milliseconds
    """
    print("Moving to neutral position...")
    set_pose(NEUTRAL, 800)
    time.sleep(0.5)
    
    print(f"Dancing for {duration_sec} seconds...")
    
    # Dance sequences with hip movements
    # Note: Hip angles are clamped to stay within 0-240 degree servo limits
    # Sequence 1: Hip sway side to side
    HIP_SWAY_SEQUENCE = [
        # Sway left
        pose_from_neutral({
            LEFT_HIP: -10,   # Left hip moves left (reduced to stay in range)
            RIGHT_HIP: -10,  # Right hip moves left (reduced to stay in range)
            LEFT_KNEE: +10,  # Slight knee bend
            RIGHT_KNEE: +10,
            LEFT_ANKLE: 0,
            RIGHT_ANKLE: 0,
        }),
        # Center
        NEUTRAL,
        # Sway right
        pose_from_neutral({
            LEFT_HIP: +10,   # Left hip moves right (reduced to stay in range)
            RIGHT_HIP: +5,   # Right hip moves right (reduced - right hip neutral is already high at 234)
            LEFT_KNEE: +10,
            RIGHT_KNEE: +10,
            LEFT_ANKLE: 0,
            RIGHT_ANKLE: 0,
        }),
        # Center
        NEUTRAL,
    ]
    
    # Sequence 2: Hip circles/rotations
    HIP_CIRCLE_SEQUENCE = [
        # Forward tilt
        pose_from_neutral({
            LEFT_HIP: +5,    # Reduced to stay in range
            RIGHT_HIP: +5,   # Reduced to stay in range
            LEFT_KNEE: +15,
            RIGHT_KNEE: +15,
            LEFT_ANKLE: -5,
            RIGHT_ANKLE: -5,
        }),
        # Right tilt
        pose_from_neutral({
            LEFT_HIP: +10,   # Reduced
            RIGHT_HIP: -5,   # Right hip down
            LEFT_KNEE: +10,
            RIGHT_KNEE: +10,
            LEFT_ANKLE: 0,
            RIGHT_ANKLE: 0,
        }),
        # Back tilt
        pose_from_neutral({
            LEFT_HIP: -10,
            RIGHT_HIP: -10,
            LEFT_KNEE: +5,
            RIGHT_KNEE: +5,
            LEFT_ANKLE: +5,
            RIGHT_ANKLE: +5,
        }),
        # Left tilt
        pose_from_neutral({
            LEFT_HIP: -5,    # Left hip down
            RIGHT_HIP: +5,   # Reduced to stay in range
            LEFT_KNEE: +10,
            RIGHT_KNEE: +10,
            LEFT_ANKLE: 0,
            RIGHT_ANKLE: 0,
        }),
        # Center
        NEUTRAL,
    ]
    
    # Sequence 3: Alternating hip lifts with knee bends
    HIP_LIFT_SEQUENCE = [
        # Lift left hip
        pose_from_neutral({
            LEFT_HIP: +15,   # Left hip up (reduced)
            RIGHT_HIP: -10,  # Right hip down
            LEFT_KNEE: +30,  # Left knee bends
            RIGHT_KNEE: +5,  # Right knee straightens
            LEFT_ANKLE: -15,
            RIGHT_ANKLE: 0,
        }),
        # Center
        NEUTRAL,
        # Lift right hip
        pose_from_neutral({
            LEFT_HIP: -10,   # Left hip down
            RIGHT_HIP: +5,   # Right hip up (reduced - right hip neutral is already high)
            LEFT_KNEE: +5,   # Left knee straightens
            RIGHT_KNEE: +30, # Right knee bends
            LEFT_ANKLE: 0,
            RIGHT_ANKLE: -15,
        }),
        # Center
        NEUTRAL,
    ]
    
    # Sequence 4: Deep hip movements with ankle coordination
    DEEP_HIP_SEQUENCE = [
        # Deep left
        pose_from_neutral({
            LEFT_HIP: -20,   # Reduced to stay in range
            RIGHT_HIP: -10,
            LEFT_KNEE: +20,
            RIGHT_KNEE: +15,
            LEFT_ANKLE: -10,
            RIGHT_ANKLE: +5,
        }),
        # Center
        NEUTRAL,
        # Deep right
        pose_from_neutral({
            LEFT_HIP: -10,
            RIGHT_HIP: -20,  # Reduced to stay in range
            LEFT_KNEE: +15,
            RIGHT_KNEE: +20,
            LEFT_ANKLE: +5,
            RIGHT_ANKLE: -10,
        }),
        # Center
        NEUTRAL,
    ]
    
    start_time = time.time()
    sequence_count = 0
    
    while (time.time() - start_time) < duration_sec:
        sequence_count += 1
        sequence_num = sequence_count % 4
        
        if sequence_num == 1:
            print(f"  Dance move: Hip Sway (sequence {sequence_count})")
            for keyframe in HIP_SWAY_SEQUENCE:
                set_pose(keyframe, t_ms)
        elif sequence_num == 2:
            print(f"  Dance move: Hip Circle (sequence {sequence_count})")
            for keyframe in HIP_CIRCLE_SEQUENCE:
                set_pose(keyframe, t_ms)
        elif sequence_num == 3:
            print(f"  Dance move: Hip Lifts (sequence {sequence_count})")
            for keyframe in HIP_LIFT_SEQUENCE:
                set_pose(keyframe, t_ms)
        else:
            print(f"  Dance move: Deep Hips (sequence {sequence_count})")
            for keyframe in DEEP_HIP_SEQUENCE:
                set_pose(keyframe, t_ms)
        
        # Check if we should continue
        if (time.time() - start_time) >= duration_sec:
            break
    
    # Return to neutral
    print("Returning to neutral...")
    set_pose(NEUTRAL, 800)
    print("✓ Dancing complete")


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """Main function to run the walking demonstration."""
    print("=" * 60)
    print("Dual Leg Walking Control - Pose-Based Keyframes")
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
        # You can call either walk() or dance() here
        # Uncomment the one you want to use:
        
        # Walk forward
        # walk(steps=10, t_ms=300)
        
        # Or dance
        dance(duration_sec=15, t_ms=400)
        
        print("\n" + "=" * 60)
        print("Demo complete!")
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

