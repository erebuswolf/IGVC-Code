#This file should hold all global settings.
import logging

USE_WX = True

MB = 1024 * 1024

ports = {
    "PSO":50000,
    "COMMAND":50001,
}

joystick = {
    "max_speed":3.2, #in m/s
    "max_angle":1.0 #in radians
}

crio_ip = "192.168.1.100"

logger = {
    "filename" : "mainCPU.log", 
    "level" : logging.DEBUG, 
    "max_filesize" : 500 * MB, #in bytes
    "numBackups" : 5,
}
