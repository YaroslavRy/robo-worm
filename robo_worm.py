# %%
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import time

# %%
p.connect(p.GUI)
p.resetSimulation()

# %%
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Configure physics for better stability
p.setPhysicsEngineParameter(numSolverIterations=100)
p.setPhysicsEngineParameter(numSubSteps=10)


# %%
planeId = p.loadURDF(
    "models/plane.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True
)  # Load a plane at z = 0 and no rotation (fixed in place)


# Load the worm horizontally (rotated 90 degrees around Y axis) and higher up
worm_orientation = p.getQuaternionFromEuler(
    [0, 1.57, 0]
)  # Rotate 90 degrees around Y axis
target_id = p.loadURDF(
    "models/worm.urdf", [0, 0, 2.0], worm_orientation
)  # Load the robot above the plane

# Get joint information and set initial positions
num_joints = p.getNumJoints(target_id)
print(f"Number of joints: {num_joints}")

joint_ids = []
for i in range(num_joints):
    joint_info = p.getJointInfo(target_id, i)
    print(f"Joint {i}: {joint_info[1].decode()}, type: {joint_info[2]}")
    joint_ids.append(i)

# Set initial joint positions for a more natural pose
rest_angles = []  # Store rest positions for spring behavior
for i, joint_id in enumerate(joint_ids):
    # Alternate slight bending to create a wavy shape
    angle = 0.1 * (-1) ** i  # Alternate between +0.1 and -0.1 radians
    p.resetJointState(target_id, joint_id, angle)
    rest_angles.append(angle)  # Store as rest position

# Spring and muscle simulation parameters
muscle_force = 2.0  # Base muscle force
muscle_frequency = 2.0  # How fast muscles contract (Hz)
muscle_phase_offset = np.pi / 2  # Phase difference between muscles
spring_stiffness = 20.0  # Spring constant (higher = stiffer springs)
spring_damping = 0.5  # Spring damping (higher = less oscillation)


def create_spring_constraints(body_id, joint_ids):
    """Create spring constraints between segments (alternative to URDF joints)"""
    spring_constraints = []

    for i, joint_id in enumerate(joint_ids):
        joint_info = p.getJointInfo(body_id, joint_id)
        parent_link_id = joint_info[16]  # Parent link index
        child_link_id = joint_id  # Child link index

        # Create a spring constraint between segments
        constraint_id = p.createConstraint(
            body_id,
            parent_link_id,
            body_id,
            child_link_id,
            p.JOINT_POINT2POINT,
            [0, 0, 0],  # Joint axis (not used for point2point)
            [0, 0, -0.1],  # Parent frame position
            [0, 0, 0.1],  # Child frame position
        )

        # Configure spring properties
        p.changeConstraint(
            constraint_id,
            maxForce=50,  # Maximum spring force
        )

        spring_constraints.append(constraint_id)

    return spring_constraints


def apply_spring_and_muscle_forces(body_id, joint_ids, rest_angles, time_step):
    """Apply spring restoring forces and muscle activation for natural movement"""
    for i, joint_id in enumerate(joint_ids):
        # Get current joint state
        joint_state = p.getJointState(body_id, joint_id)
        current_angle = joint_state[0]
        current_velocity = joint_state[1]
        
        # Calculate spring restoring force (Hooke's law: F = -kx)
        displacement = current_angle - rest_angles[i]
        spring_force = -spring_stiffness * displacement
        
        # Add spring damping force (F = -cv)
        damping_force = -spring_damping * current_velocity
        
        # Calculate muscle activation with phase offset for wave-like motion
        muscle_activation = np.sin(time_step * muscle_frequency + i * muscle_phase_offset)
        muscle_torque = muscle_force * muscle_activation
        
        # Combine all forces
        total_torque = spring_force + damping_force + muscle_torque
        
        # Apply combined torque
        p.setJointMotorControl2(
            body_id, 
            joint_id, 
            p.TORQUE_CONTROL, 
            force=total_torque
        )


# %%
# Run simulation for longer and check if worm exists
for i in range(10000):  # Longer simulation
    # Check if the worm still exists
    try:
        pos, orn = p.getBasePositionAndOrientation(target_id)
        current_time = i * 0.001  # Simulation time

        # Print position every 100 steps to reduce spam
        if i % 100 == 0:
            print(f"Step {i}: Worm position: {pos}")

        # Reset camera to follow the worm (only initially, then allow manual control)
        if i == 0:
            p.resetDebugVisualizerCamera(5, 45, -30, pos)

        # Apply spring and muscle forces for natural locomotion
        apply_spring_and_muscle_forces(target_id, joint_ids, rest_angles, current_time)

        # If worm falls below ground, reset its position
        if pos[2] < -0.5:
            print("Worm fell through ground, resetting position")
            worm_orientation = p.getQuaternionFromEuler([0, 1.57, 0])
            p.resetBasePositionAndOrientation(target_id, [0, 0, 2.0], worm_orientation)

    except Exception as e:
        print(f"Worm disappeared at step {i}: {e}")
        break

    p.stepSimulation()
    time.sleep(0.001)  # Faster simulation


# %%
p.disconnect()
