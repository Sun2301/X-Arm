#!/usr/bin/env python
# coding: utf-8
# sun_motion_plan.py
# Planification MoveIt sans contrainte d'orientation

import rospy
from time import sleep
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    rospy.init_node("sun_motion_plan")

    dofbot = MoveGroupCommander("dofbot")
    dofbot.allow_replanning(True)
    dofbot.set_planning_time(20)
    dofbot.set_num_planning_attempts(50)
    dofbot.set_goal_position_tolerance(0.01)
    dofbot.set_goal_tolerance(0.01)
    dofbot.set_max_velocity_scaling_factor(1.0)
    dofbot.set_max_acceleration_scaling_factor(1.0)

    # Aller en position "down" d'abord
    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)

    # --- Cible : position seulement, pas d'orientation ---
    x = 0.059
    y = 0.2269
    z = 0.168

    dofbot.set_position_target([x, y, z])

    for i in range(15):
        plan = dofbot.plan()
        if len(plan.joint_trajectory.points) != 0:
            print("Plan trouvé — exécution")
            dofbot.execute(plan)
            break
        else:
            print(f"Tentative {i+1}/15 — échec")

    sleep(1)
    print("Pose finale :", dofbot.get_current_pose().pose)
