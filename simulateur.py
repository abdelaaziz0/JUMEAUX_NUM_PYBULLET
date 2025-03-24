
"""
simulateur.py
----------------------------------------------
Simulation PyBullet d'un Kuka IIWA 7 DOF (sans pince),
qui effectue un pick-and-place virtuel de plusieurs cubes,
en 'téléportant' l'objet quand il descend pour le saisir.
Les données (positions, cubes, phase, etc.) sont envoyées par UDP.
"""

import pybullet as p
import pybullet_data
import math
import time
import random
import socket
import json

# Paramètres
NUM_CUBES = 3        
TABLE_POS = [0.7, 0.0, -0.2] 
PICK_HEIGHT = 0.025 
ROBOT_URDF = "kuka_iiwa/model.urdf" 
CUBE_URDF  = "cube_small.urdf"

def interpolate_points(pointA, pointB, steps=50):
    """ Génère une liste de points (linéairement interpolés) entre pointA et pointB. """
    path = []
    for i in range(steps):
        alpha = i / (steps - 1)
        x = pointA[0] + alpha * (pointB[0] - pointA[0])
        y = pointA[1] + alpha * (pointB[1] - pointA[1])
        z = pointA[2] + alpha * (pointB[2] - pointA[2])
        path.append([x, y, z])
    return path

def calculate_ik(robot_id, end_effector_index, target_pos, target_orn):
    """ Calcule l'IK (inverse kinematics) pour atteindre target_pos/target_orn. """
    joint_poses = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        target_pos,
        target_orn,
        lowerLimits=[-2*math.pi]*7,
        upperLimits=[2*math.pi]*7,
        jointRanges=[4*math.pi]*7,
        restPoses=[0]*7,
        maxNumIterations=200
    )
    return joint_poses

def set_arm_joints(robot_id, joint_poses, max_force=300):
    """ Applique un contrôle de position sur chaque articulation du bras (0-6). """
    for i in range(7):
        p.setJointMotorControl2(
            robot_id, 
            i, 
            p.POSITION_CONTROL, 
            targetPosition=joint_poses[i],
            force=max_force
        )

def run_sim(
    udp_ip="127.0.0.1",
    udp_port=5005,
    step_time=1/240,
    total_steps=20000
):
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    table_id = p.loadURDF("table/table.urdf", TABLE_POS, p.getQuaternionFromEuler([0, 0, 0]))

    robot_id = p.loadURDF(
        ROBOT_URDF, 
        [0.0, 0.0, 0.0], 
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )
    end_effector_index = 6

    cubes_data = []
    for i in range(NUM_CUBES):
        cx = TABLE_POS[0] + random.uniform(-0.1, 0.1)
        cy = TABLE_POS[1] + random.uniform(-0.2, 0.2)
        cz = TABLE_POS[2] + 0.02
        cube_id = p.loadURDF(CUBE_URDF, [cx, cy, cz], p.getQuaternionFromEuler([0, 0, 0]))
        cubes_data.append({
            "id": cube_id,
            "picked": False,
            "placed": False
        })

    place_pos = [TABLE_POS[0], TABLE_POS[1] + 0.3, TABLE_POS[2] + 0.02]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[Simulateur] Lancement, envoi vers {udp_ip}:{udp_port}")

    step_count = 0
    current_cube_index = 0
    phase = "go_pick"
    teleporting_object = False 

    while step_count < total_steps:
        p.stepSimulation()

        collision_info = p.getContactPoints()
        collisions_list = []
        for c in collision_info:
            bodyA = c[1]
            bodyB = c[2]
            if bodyA != plane_id and bodyB != plane_id:
                collisions_list.append({"A": bodyA, "B": bodyB})

        eff_state = p.getLinkState(robot_id, end_effector_index)
        eff_pos = eff_state[0]
        eff_orn = eff_state[1]

        if current_cube_index >= NUM_CUBES:
            phase = "done"

        if phase != "done":
            cube_info = cubes_data[current_cube_index]
            cpos, corn = p.getBasePositionAndOrientation(cube_info["id"])
            above_cube = [cpos[0], cpos[1], cpos[2] + 0.15]
            pick_cube  = [cpos[0], cpos[1], TABLE_POS[2] + PICK_HEIGHT]

            if phase == "go_pick":
                path = interpolate_points([0.5, 0, 0.3], above_cube, steps=50)
                for pt in path:
                    joint_poses = calculate_ik(robot_id, end_effector_index, pt, p.getQuaternionFromEuler([math.pi, 0, 0]))
                    set_arm_joints(robot_id, joint_poses)
                    p.stepSimulation()
                    time.sleep(step_time)
                phase = "descend"

            elif phase == "descend":
                path = interpolate_points(above_cube, pick_cube, steps=50)
                for pt in path:
                    joint_poses = calculate_ik(robot_id, end_effector_index, pt, p.getQuaternionFromEuler([math.pi, 0, 0]))
                    set_arm_joints(robot_id, joint_poses)
                    p.stepSimulation()
                    time.sleep(step_time)
                teleporting_object = True
                phase = "lift"

            elif phase == "lift":
                lift_target = [pick_cube[0], pick_cube[1], pick_cube[2] + 0.15]
                path = interpolate_points(pick_cube, lift_target, steps=50)
                for pt in path:
                    joint_poses = calculate_ik(robot_id, end_effector_index, pt, p.getQuaternionFromEuler([math.pi, 0, 0]))
                    set_arm_joints(robot_id, joint_poses)
                    p.stepSimulation()
                    time.sleep(step_time)
                phase = "go_place"

            elif phase == "go_place":
                current_eff = eff_pos
                path = interpolate_points(current_eff, [place_pos[0], place_pos[1], current_eff[2]], steps=60)
                for pt in path:
                    joint_poses = calculate_ik(robot_id, end_effector_index, pt, p.getQuaternionFromEuler([math.pi, 0, 0]))
                    set_arm_joints(robot_id, joint_poses)
                    p.stepSimulation()
                    time.sleep(step_time)
                path = interpolate_points([place_pos[0], place_pos[1], current_eff[2]], [place_pos[0], place_pos[1], TABLE_POS[2] + PICK_HEIGHT], steps=50)
                for pt in path:
                    joint_poses = calculate_ik(robot_id, end_effector_index, pt, p.getQuaternionFromEuler([math.pi, 0, 0]))
                    set_arm_joints(robot_id, joint_poses)
                    p.stepSimulation()
                    time.sleep(step_time)
                teleporting_object = False
                phase = "next_cube"

            elif phase == "next_cube":
                current_cube_index += 1
                if current_cube_index < NUM_CUBES:
                    phase = "go_pick"
                else:
                    phase = "done"

        if teleporting_object:
            cube_info = cubes_data[current_cube_index]
            p.resetBasePositionAndOrientation(cube_info["id"], eff_pos, eff_orn)

        joint_positions = []
        for j in range(7):
            st = p.getJointState(robot_id, j)
            joint_positions.append(st[0])

        cubes_positions = []
        for cd in cubes_data:
            pos, orn = p.getBasePositionAndOrientation(cd["id"])
            cubes_positions.append({
                "id": cd["id"],
                "pos": pos,
                "orn": orn
            })

        data_dict = {
            "timestamp": time.time(),
            "phase": phase,
            "current_cube_index": current_cube_index,
            "joint_positions": joint_positions,
            "teleporting": teleporting_object,
            "cubes": cubes_positions,
            "collisions": collisions_list,
        }
        msg_json = json.dumps(data_dict)
        sock.sendto(msg_json.encode('utf-8'), (udp_ip, udp_port))

        step_count += 1
        time.sleep(step_time)

    p.disconnect()
    print("[Simulateur] Simulation terminée")


if __name__ == "__main__":
    run_sim()
