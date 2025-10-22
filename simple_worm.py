# %%
import numpy as np
import pybullet as p
import pybullet_data
import time

# %%
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# %%
# Load plane and worm
planeId = p.loadURDF("models/plane.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)

# Load worm horizontally
worm_orientation = p.getQuaternionFromEuler([0, 1.57, 0])
worm_id = p.loadURDF("models/worm_simple.urdf", [0, 0, 2.0], worm_orientation)

# Get joint info
num_joints = p.getNumJoints(worm_id)
print(f"Joints: {num_joints}")

# %%
# Spring parameters
spring_stiffness = 30.0  # Lateral spring strength
spring_damping = 2.0  # Spring damping


def get_link_position(body_id, link_id):
    """Get world position of a link"""
    if link_id == -1:  # Base link (head)
        pos, _ = p.getBasePositionAndOrientation(body_id)
        return np.array(pos)
    else:
        link_state = p.getLinkState(body_id, link_id)
        return np.array(link_state[0])


def apply_lateral_springs(body_id):
    """Apply lateral spring forces between adjacent segments"""

    # Get positions of all segments
    head_pos = get_link_position(body_id, -1)
    seg1_pos = get_link_position(body_id, 0)
    seg2_pos = get_link_position(body_id, 1)
    tail_pos = get_link_position(body_id, 2)

    positions = [head_pos, seg1_pos, seg2_pos, tail_pos]

    # Apply spring forces between adjacent segments
    for i in range(len(positions) - 1):
        current_pos = positions[i]
        next_pos = positions[i + 1]

        # Calculate spring vector
        spring_vector = next_pos - current_pos
        distance = np.linalg.norm(spring_vector)

        # Desired spring length (natural segment spacing)
        natural_length = 0.35

        # Spring force (Hooke's law)
        if distance > 0:
            spring_direction = spring_vector / distance
            spring_force_magnitude = spring_stiffness * (distance - natural_length)
            spring_force = spring_force_magnitude * spring_direction

            # Apply forces to both segments (Newton's 3rd law)
            if i == 0:  # Head segment
                p.applyExternalForce(
                    body_id, -1, spring_force, current_pos, p.WORLD_FRAME
                )
                p.applyExternalForce(body_id, 0, -spring_force, next_pos, p.WORLD_FRAME)
            else:  # Other segments
                p.applyExternalForce(
                    body_id, i - 1, spring_force, current_pos, p.WORLD_FRAME
                )
                p.applyExternalForce(body_id, i, -spring_force, next_pos, p.WORLD_FRAME)


def apply_muscle_waves(body_id, time_step):
    """Apply simple muscle wave for locomotion"""
    wave_frequency = 1.0
    wave_amplitude = 5.0

    for joint_id in range(num_joints):
        # Create traveling wave
        phase = joint_id * np.pi / 2
        muscle_force = wave_amplitude * np.sin(wave_frequency * time_step + phase)

        p.setJointMotorControl2(body_id, joint_id, p.TORQUE_CONTROL, force=muscle_force)


# %%
# Main simulation loop
for step in range(10000):
    current_time = step * 0.01

    # Apply lateral springs between segments
    apply_lateral_springs(worm_id)

    # Apply muscle waves for locomotion
    apply_muscle_waves(worm_id, current_time)

    # Camera setup (once)
    if step == 0:
        pos, _ = p.getBasePositionAndOrientation(worm_id)
        p.resetDebugVisualizerCamera(3, 30, -20, pos)

    # Status
    if step % 200 == 0:
        pos, _ = p.getBasePositionAndOrientation(worm_id)
        print(f"Step {step}: Worm at {pos}")

    p.stepSimulation()
    time.sleep(0.01)

# %%
p.disconnect()
