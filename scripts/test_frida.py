import numpy as np
import time
import _core_ext as vamp
import pybullet as p
import pybullet_data


# [joint1, joint2, joint3, joint4, joint5, joint6, rightfinger, leftfinger]
pose_a = [0.0, -2.1, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0]
pose_b = [0.0, -2.1, -1.1, -1.0, 0.0, 0.0, 0.0, 0.0]

obstaculo_centro = [0, -0.2, 1.0, 0.1]
problem = [obstaculo_centro]

def run_demo():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    robot_path = "/home/dominguez/roborregos/home_ws/src/robot_description/frida_description/urdf/TMR2025/frida_real.urdf"

    robot_id = p.loadURDF(robot_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    for i in range(p.getNumJoints(robot_id)):
        p.changeVisualShape(robot_id, i, rgbaColor=[0, 0, 0, 0.6])

    env = vamp.Environment()
    for obst in problem:
        s = vamp.Sphere(obst[:3], obst[3])
        env.add_sphere(s)

        v_shape = p.createVisualShape(p.GEOM_SPHERE, radius=obst[3], rgbaColor=[1, 0, 0, 0.5])
        p.createMultiBody(baseVisualShapeIndex=v_shape, basePosition=obst[:3])

    movable_joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            movable_joints.append(i)

    print("Validating poses...")
    if not vamp.frida_real.validate(pose_a, env) or not vamp.frida_real.validate(pose_b, env):
        print("VAMP detected collision at start or goal. Check the collision model in C++.")

    print("Planning with RRTC...")
    settings = vamp.RRTCSettings()
    settings.max_iterations = 2000
    rng = vamp.frida_real.xorshift()
    result = vamp.frida_real.rrtc(pose_a, pose_b, env, settings, rng)

    if result and len(result.path) >= 2:

        opt_result = vamp.frida_real.simplify(result.path, env, vamp.SimplifySettings(), rng)
        path = opt_result.path
        print(f"Route found: {len(path)} waypoints.")

        while True:
            for i in range(len(path) - 1):
                start_p, end_p = np.array(path[i]), np.array(path[i+1])
                steps = 50
                for t in range(steps):
                    interp_pose = start_p + (end_p - start_p) * (t / steps)
                    
                   
                    for idx, j_id in enumerate(movable_joints[:8]):
                        p.resetJointState(robot_id, j_id, interp_pose[idx])
                    time.sleep(0.01)
            time.sleep(1)
    else:
        print("RRTC could not find a safe solution.")

if __name__ == "__main__":
    run_demo()