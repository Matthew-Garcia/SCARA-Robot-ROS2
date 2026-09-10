#!/usr/bin/env python3
"""Run both complete stacks in isolated ROS domains, preserving diagnostic logs."""
import os
import signal
import subprocess
import time
from pathlib import Path
for index,mode in enumerate(['mock','gazebo']):
    env={**os.environ,'ROS_DOMAIN_ID':str(201+index),'LIBGL_ALWAYS_SOFTWARE':'1','GAZEBO_MODEL_DATABASE_URI':''}
    path=Path('/tmp')/f'scara-{mode}.log'
    with path.open('w') as log:
        launch=subprocess.Popen(['ros2','launch','scara_bringup','sim.launch.py',f'mode:={mode}','gui:=false','rviz:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            # Humble controller_manager can abort if diagnostics call its
            # services while the initial spawner is still configuring it.
            time.sleep(15 if mode == 'gazebo' else 8)
            subprocess.run(['ros2','run','scara_bringup','check_stack.py'],env=env,check=True,timeout=180)
            if mode == 'gazebo':
                subprocess.run(['ros2','run','scara_bringup','pick_lift_demo.py','--object','cube'],env=env,check=True,timeout=90)
        except Exception:
            print(path.read_text());raise
        finally:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGKILL);launch.wait()

# Exercise the separate OpenCV conveyor world with one bounded red-cube cycle.
env={**os.environ,'ROS_DOMAIN_ID':'203','LIBGL_ALWAYS_SOFTWARE':'1','GAZEBO_MODEL_DATABASE_URI':''}
path=Path('/tmp')/'scara-conveyor.log'
with path.open('w') as log:
    launch=subprocess.Popen(
        ['ros2','launch','scara_bringup','conveyor_demo.launch.py',
         'gui:=false','rviz:=false','autorun:=false'],
        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(18)
        subprocess.run(
            ['ros2','topic','echo','/conveyor/vision/detection',
             'std_msgs/msg/String','--once'],env=env,check=True,timeout=45)
        subprocess.run(
            ['ros2','run','scara_bringup','conveyor_sort_demo.py','--cycles','1'],
            env=env,check=True,timeout=120)
    except Exception:
        print(path.read_text());raise
    finally:
        os.killpg(launch.pid,signal.SIGINT)
        try:launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid,signal.SIGKILL);launch.wait()
