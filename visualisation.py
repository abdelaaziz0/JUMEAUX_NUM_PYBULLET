#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
visualisation_client.py (Option B - sans gripper)
------------------------------------------------
Client PyBullet qui reçoit les données du simulateur_complex.py
et met à jour la scène (robot Kuka IIWA 7 DOF, cubes).
Calcule la latence, affiche collisions, etc.
"""

import pybullet as p
import pybullet_data
import socket
import json
import time
import statistics

ROBOT_URDF = "kuka_iiwa/model.urdf"
NUM_CUBES   = 3

def visualize_advanced(
    udp_ip="127.0.0.1",
    udp_port=5005,
    max_messages=20000
):
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)

    # Charger le sol et la table
    plane_id = p.loadURDF("plane.urdf")
    table_id = p.loadURDF("table/table.urdf",[0.7,0,-0.2],p.getQuaternionFromEuler([0,0,0]))

    # Charger le robot Kuka IIWA standard (7 DOF)
    robot_id = p.loadURDF(
        ROBOT_URDF,
        [0,0,0],
        p.getQuaternionFromEuler([0,0,0]),
        useFixedBase=True
    )

    # Créer le même nombre de cubes
    cube_ids = []
    for i in range(NUM_CUBES):
        c_id = p.loadURDF("cube_small.urdf",[0.7,0,0.02+i*0.05],p.getQuaternionFromEuler([0,0,0]))
        cube_ids.append(c_id)

    # Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    sock.settimeout(5.0)

    latencies = []
    msg_count = 0

    print(f"[Client Avancé - No Gripper] Démarrage, écoute sur {udp_ip}:{udp_port}")

    while msg_count < max_messages:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            print("[Client Avancé - No Gripper] Timeout, pas de data reçue.")
            break

        recv_time = time.time()
        data_dict = json.loads(data.decode('utf-8'))

        send_time = data_dict["timestamp"]
        phase = data_dict["phase"]
        current_cube_index = data_dict["current_cube_index"]
        joint_positions = data_dict["joint_positions"]
        collisions = data_dict["collisions"]
        cubes_info = data_dict["cubes"]
        teleporting = data_dict["teleporting"]

        # Latence
        lat_ms = (recv_time - send_time)*1000
        latencies.append(lat_ms)

        # Mettre à jour le bras (7 articulations)
        for j in range(7):
            p.setJointMotorControl2(
                robot_id,
                j,
                p.POSITION_CONTROL,
                targetPosition=joint_positions[j],
                force=300
            )

        # Mettre à jour cubes
        # On suppose un ordre stable: cubes_info[i] correspond à cube_ids[i]
        for i, cinfo in enumerate(cubes_info):
            pos = cinfo["pos"]
            orn = cinfo["orn"]
            p.resetBasePositionAndOrientation(cube_ids[i], pos, orn)

        # Collisions
        if collisions and msg_count % 50 == 0:
            print(f"[Client Avancé - No Gripper] Collisions détectées: {collisions}")

        # Step
        p.stepSimulation()

        if msg_count % 50 == 0:
            print(f"[Client Avancé - No Gripper] Msg={msg_count}, phase={phase}, cube={current_cube_index}, lat={lat_ms:.2f} ms, teleport={teleporting}")

        msg_count += 1

    # Stats latences
    if latencies:
        avg = statistics.mean(latencies)
        mn = min(latencies)
        mx = max(latencies)
        print(f"\n[Client Avancé - No Gripper] Latences (ms): Moy={avg:.2f}, Min={mn:.2f}, Max={mx:.2f}")
    else:
        print("[Client Avancé - No Gripper] Aucune data reçue.")

    print("[Client Avancé - No Gripper] Fin, on laisse la fenêtre PyBullet ouverte.")
    while True:
        time.sleep(0.1)
        p.stepSimulation()

if __name__ == "__main__":
    visualize_advanced()
