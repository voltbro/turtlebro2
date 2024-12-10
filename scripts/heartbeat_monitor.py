#!/usr/bin/env python3

import threading
import os.path, time
import logging

from pystemd.systemd1 import Unit
from pathlib import Path

logging.basicConfig(level=logging.INFO, 
                        filename="/home/pi/.ros/log/microros_heartbeat.log", 
                        filemode="w",
                        format="%(asctime)s %(levelname)s %(message)s")

logging.info("Start microros service monitor")

file = "/home/pi/.ros/.microros_heartbeat"

unit = Unit(b'microros.service')
unit.load()
 
def timer_callback():
 
    if time.time() - os.path.getmtime(file) > 10:
        logging.info("Need restart microros")
        unit.Unit.Stop(b'replace')
        time.sleep(10)
        unit.Unit.Start(b'replace')
        
        Path('/home/pi/.ros/.microros_heartbeat').touch()

    threading.Timer(5.0, timer_callback).start()

timer_callback()


