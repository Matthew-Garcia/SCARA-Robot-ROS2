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
