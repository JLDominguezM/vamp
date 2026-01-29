import numpy as np
import time
import _core_ext as vamp
import pybullet as p
import pybullet_data

# 1. Pose Configuration
# Pose A: Start (Extended forward)
pose_a = [0.0, -1.5, 3.1, 0.0, 0.0, 0.0]
# Pose B: Goal (Inclined forward)
pose_b = [-1.0, -0.6, 3.1, 0.0, 0.0, 0.0]

# 2. Environment Definition (Obstacles)

obstaculo_centro = [-0.4, 0.4, 0.5, 0.15]
problem = [obstaculo_centro]

def run_demo():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    robot_path = "/home/dominguez/roborregos/home_ws/src/robot_description/frida_description/urdf/xarm/spherized_xarm/xarm6.urdf"
    robot_id = p.loadURDF(robot_path, useFixedBase=True)

    # Vamp Environment Setup
    env = vamp.Environment()
    for obst in problem:
        s = vamp.Sphere(obst[:3], obst[3])
        env.add_sphere(s)
        
        # Draw in PyBullet
        v_shape = p.createVisualShape(p.GEOM_SPHERE, radius=obst[3], rgbaColor=[1, 0, 0, 0.7])
        p.createMultiBody(baseVisualShapeIndex=v_shape, basePosition=obst[:3])

    print(f"VAMP Environment configured.")

    # 3. Validate Poses (Use env, not problem)
    print("Validating poses...")
    if not vamp.frida_real.validate(pose_a, env):
        print("Error: Start Pose in collision.")
        return
    if not vamp.frida_real.validate(pose_b, env):
        print("Error: Goal Pose in collision.")
        return

    # 4. Planning with RRTC
    settings = vamp.RRTCSettings()
    # Optional settings for TMR (you can play with these later)
    settings.max_iterations = 2000


    rng = vamp.frida_real.xorshift()
    print("Planning path with RRTC...")
    start_time = time.time()

    # IMPORTANT: rrtc returns an object, we need to extract the .path
    result = vamp.frida_real.rrtc(pose_a, pose_b, env, settings, rng)
    
    if result and len(result.path) >= 2:
        simplify_settings = vamp.SimplifySettings()
        opt_result = vamp.frida_real.simplify(result.path, env, simplify_settings, rng)
        path = opt_result.path
        print(f"Obtained path with {len(path)} waypoints after simplification.")

        # Generate 50 intermediate steps between each planner point
        steps = 50

        while True:
            for i in range(len(path) - 1):
                start_p = np.array(path[i])
                end_p = np.array(path[i+1])
                
                # Simple linear interpolation
                for t in range(steps):
                    fraction = t / steps
                    interp_pose = start_p + (end_p - start_p) * fraction

                    for joint in range(6):
                        p.resetJointState(robot_id, joint, interp_pose[joint])
                    time.sleep(0.01)
            time.sleep(1)

if __name__ == "__main__":
    run_demo()