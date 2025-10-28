# RoboWorm — PyBullet Worm Robot Simulation

Overview
- RoboWorm is a small PyBullet-based simulation of a worm-like robot (head, body segments, tail) driven by revolute joints and simulated muscle forces.
- The repository includes a runnable Python simulator, a slider demo, URDF robot and plane models, and a notebook for quick experiments.

Files
- [robo_worm.py](robo_worm.py) — main simulation script. Uses a camera, user sliders for joint control and POSITION_CONTROL to command joints. See symbols: [`robo_worm.get_link_index_by_name`](robo_worm.py), [`robo_worm.slider_to_joint`](robo_worm.py) and [`robo_worm.robot_id`](robo_worm.py).
- [robo_worm.ipynb](robo_worm.ipynb) — notebook with quick experiments and diagnostics.
- [requirements.txt](requirements.txt) — pinned Python dependencies for the project.
- [models/worm.urdf](models/worm.urdf) — URDF describing the worm (head, anchors, segments, revolute joints).
- [models/plane.urdf](models/plane.urdf) — environment plane with obstacles.

Quick start (virtualenv recommended)
1. Create and activate a venv (example):
   - python -m venv venv
   - source venv/bin/activate
2. Install dependencies:
   - pip install -r requirements.txt
3. Run the main simulator:
   - `python robo_worm.py`
   - The script creates debug sliders for non-fixed joints and opens a PyBullet GUI camera view.
4. Open the notebook for interactive experiments:
   - (TBC) jupyter lab robo_worm.ipynb

Controls & behavior
- Sliders created in [robo_worm.py](robo_worm.py) map to joints via [`robo_worm.slider_to_joint`](robo_worm.py). Adjust sliders to set joint target positions.
- The camera is computed relative to the head and first segment; see the head/segment selection logic in [robo_worm.py](robo_worm.py).
- URDF joints are revolute and have limits; the scripts sanitize and use sensible defaults when limits are missing.

Development tips
- Use `p.setPhysicsEngineParameter` to tune solver/substeps for stability.
- Use `p.addUserDebugParameter` for rapid control surfaces during debugging.
- For muscle simulation, implement a small PD or spring-damper wrapper that calls `setJointMotorControl2` with torque control or computes target positions/forces each step.

Troubleshooting
- If URDF can't be found, ensure `p.setAdditionalSearchPath(pybullet_data.getDataPath())` is set and the `models/` path is correct.
- If a link name lookup fails, [`robo_worm.get_link_index_by_name`](robo_worm.py) falls back to safe indices — adjust as needed.

Acknowledgements
- This project uses PyBullet for physics and OpenCV for optional camera display.

License
- [ ] add licence