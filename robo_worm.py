# %%
import numpy as np
import pybullet as p
import pybullet_data
import time

# Connect and initialize
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Physics tuning
p.setPhysicsEngineParameter(numSolverIterations=100)
p.setPhysicsEngineParameter(numSubSteps=10)

# Load environment and robot
planeId = p.loadURDF("models/plane.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)

# Load worm (adjust path/orientation as needed)
worm_orientation = p.getQuaternionFromEuler([0, 1.57, 0])
robot_id = p.loadURDF("models/worm.urdf", [0, 0, 0.5], worm_orientation)

# get joints and create sliders to control them
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints: {num_joints}")

# Create sliders only for non-fixed joints and map slider -> joint index
slider_to_joint = {}
joint_default_targets = {}
for ji in range(num_joints):
    info = p.getJointInfo(robot_id, ji)
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    lower_limit = info[8]
    upper_limit = info[9]

    # skip fixed joints
    if joint_type == p.JOINT_FIXED:
        print(f"Skipping fixed joint {ji} ({joint_name})")
        continue

    # sanitize limits (some URDFs use huge values)
    if lower_limit < -3.1416 or lower_limit < -1e3:
        lower_limit = -3.1416
    if upper_limit > 3.1416 or upper_limit > 1e3:
        upper_limit = 3.1416
    if lower_limit >= upper_limit:
        # fallback range
        lower_limit, upper_limit = -1.57, 1.57

    # default start at midpoint
    default = float((lower_limit + upper_limit) / 2.0)
    slider = p.addUserDebugParameter(joint_name, lower_limit, upper_limit, default)
    slider_to_joint[slider] = ji
    joint_default_targets[ji] = default
    print(f"Created slider for joint {ji} '{joint_name}' range [{lower_limit:.2f}, {upper_limit:.2f}]")

# Main loop: read sliders and drive joints with POSITION_CONTROL
try:
    step = 0
    while True:
        # read each slider and command the matching joint
        for slider_handle, joint_index in slider_to_joint.items():
            try:
                target = p.readUserDebugParameter(slider_handle)
            except Exception:
                # fallback to previous/default if read fails
                target = joint_default_targets.get(joint_index, 0.0)
            # send position command (adjust maxForce if needed)
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(target),
                force=10.0,
                maxVelocity=10,
            )

        # optional: step, sleep, periodic status
        p.stepSimulation()
        if step % 200 == 0:
            base_pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"Step {step}: base position at {base_pos}")
        step += 1
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Simulation stopped by user")

finally:
    p.disconnect()
