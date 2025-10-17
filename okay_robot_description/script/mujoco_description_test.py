import time

import mujoco
import mujoco.viewer  # type: ignore

# SOURCE:
# https://mujoco.readthedocs.io/en/stable/python.html

# TUTORIALS:
# https://github.com/google-deepmind/mujoco

robot_name = "okay_robot"

# https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjmodel
m = mujoco.MjModel.from_xml_path("okay_robot_description/urdf/" + robot_name + "/scene.xml")

# https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata
d = mujoco.MjData(m)

num_actuators = m.nu
num_dof = m.nv

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
