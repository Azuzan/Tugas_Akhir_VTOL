import numpy as np
import cv2
from gpiozero import AngularServo 
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, math
import smbus
from sshkeyboard import listen_keyboard
import RPi.GPIO as GPIO
import csv

bus = smbus.SMBus(1)

address = 0x1E
address2 = 0x10
getLidarDataCmd = [0x5A,0x05,0x00,0x01,0x60]

servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo2 = AngularServo(13, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(12, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo4 = AngularServo(19, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo.angle = 0
servo2.angle = 0
servo3.angle = 0
servo4.angle = 0

def nothing(x):
    pass

def basic_takeoff(altitude):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    vehicle.simple_takeoff(altitude)
    while True:
        log()
        print("Reached Height = ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.4:
            break
        time.sleep(1)

def change_mode(mode):
    log()
    vehicle.mode = VehicleMode(mode)

def send_to(latitude, longitude, altitude):
    if vehicle.mode.name == "GUIDED":
        location = LocationGlobalRelative(latitude, longitude, float(altitude))
        print(f"lat:'{latitude}' -- lon:'{longitude}' -- alt:'{altitude}'")
        vehicle.simple_goto(location, airspeed=0.5, groundspeed=0.2)
        time.sleep(1)

def change_alt(step):
    actual_altitude = float(vehicle.location.global_relative_frame.alt)
    changed_altitude = [(actual_altitude + 0.30), (actual_altitude - 0.30)]
    if step == "INC":
        if changed_altitude[0] <= 2:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[0])
        else:
            print("Vehicle Reached Maximum Altitude!!!")
    if step == "DEC":
        if changed_altitude[1] >= 0.4:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[1])
        else:
            print("Vehicle Reached Minimum Altitude!!!")

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 # yaw relative to direction of travel
    else:
        is_relative = 0 # yaw is an absolute angle
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0, # confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.5)
        
def get_distance_metres(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def destination_location(homeLattitude, homeLongitude, distance, bearing):
    R = 6371e3
    rlat1 = homeLattitude * (math.pi/180.0) 
    rlon1 = homeLongitude * (math.pi/180.0)
    d = distance
    bearing = 1.0*bearing * (math.pi/180.0)
    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)), (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))
    rlat2 = rlat2 * (180.0/math.pi) 
    rlon2 = rlon2 * (180.0/math.pi)
    location = [rlat2, rlon2]
    return location

def misi_outdoor():
    print("------- MASUK KE MISI -------")
    fly_mode = 0
    dly_mode = 0
    loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
    old_loc = loc
    spd = 3
    spdlow = 1.5
    while True:
        log()
        if fly_mode == 0:
            if dly_mode == 0 and int(vehicle.location.global_relative_frame.alt) <= 0.9:
                basic_takeoff(altitude = 3)  
            dly_mode += 1 
            if dly_mode > 2:
                fly_mode = 1
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc
        if fly_mode == 1: # kanan 5 m    
            loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
            print("fly_mode_1 : ",d_n)
            if d_n > 10: # jarak
                send_ned_velocity(0, 0, 0, 1)
                dly_mode += 1
            elif 7 < d_n <= 10:
                dX = math.cos(vehicle.attitude.yaw + (1.5708)) * spdlow 
                dY = math.sin(vehicle.attitude.yaw + (1.5708)) * spdlow                 
                send_ned_velocity(dX, dY, 0, 1)
                print("1.5 m/s")
            else:
                dX = math.cos(vehicle.attitude.yaw + (1.5708)) * spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708)) * spd                 
                send_ned_velocity(dX, dY, 0, 1)  
                print("3 m/s")
            if dly_mode > 2:
                servo.angle = 90
                fly_mode = 2
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc
        elif fly_mode == 2: # kedepan 10 m
            loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
            print("fly_mode_2 : ",d_n)
            if d_n > 15: # jarak
                send_ned_velocity(0, 0, 0, 1)
                dly_mode += 1
            elif 12 < d_n <= 15:
                dX = math.cos(vehicle.attitude.yaw) * spdlow 
                dY = math.sin(vehicle.attitude.yaw) * spdlow                 
                send_ned_velocity(dX, dY, 0, 1)
                print("1.5 m/s")
            else:
                dX = math.cos(vehicle.attitude.yaw) * spd 
                dY = math.sin(vehicle.attitude.yaw) * spd                 
                send_ned_velocity(dX, dY, 0, 1)
                print("3 m/s")
            if dly_mode > 2:
                servo3.angle = 90
                fly_mode = 3
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
        elif fly_mode == 3: # kiri 10 m
            loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
            print("fly_mode_3 : ",d_n)
            if d_n > 21: # jarak
                send_ned_velocity(0, 0, 0, 1)
                dly_mode += 1
            elif 16 < d_n <= 21:
                dX = math.cos(vehicle.attitude.yaw - (1.5708)) * spdlow 
                dY = math.sin(vehicle.attitude.yaw - (1.5708)) * spdlow                 
                send_ned_velocity(dX, dY, 0, 1)
                print("1.5 m/s")
            else:
                dX = math.cos(vehicle.attitude.yaw - (1.5708)) * spd 
                dY = math.sin(vehicle.attitude.yaw - (1.5708)) * spd                 
                send_ned_velocity(dX, dY, 0, 1)  
                print("3 m/s")
            if dly_mode > 2:
                servo2.angle = 90
                fly_mode = 4
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
        elif fly_mode == 4: # mundur 10 m
            loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
            print("fly_mode_4 : ",d_n)
            if d_n > 15: # jarak
                send_ned_velocity(0, 0, 0, 1)
                dly_mode += 1
            elif 12 < d_n <= 15:
                dX = math.cos(vehicle.attitude.yaw + (1.5708) * 2) * spdlow 
                dY = math.sin(vehicle.attitude.yaw + (1.5708) * 2) * spdlow                 
                send_ned_velocity(dX, dY, 0, 1)
                print("1.5 m/s")
            else:
                dX = math.cos(vehicle.attitude.yaw + (1.5708) * 2) * spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708) * 2) * spd                 
                send_ned_velocity(dX, dY, 0, 1)
                print("3 m/s")
            if dly_mode > 2:
                servo4.angle = 90
                fly_mode = 5
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
        elif fly_mode == 5: # kanan 5 m
            loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
            print("fly_mode_5 : ",d_n)
            if d_n > 10: # jarak
                send_ned_velocity(0, 0, 0, 1)
                dly_mode += 1
            elif 6 < d_n <= 10:
                dX = math.cos(vehicle.attitude.yaw + (1.5708)) * spdlow 
                dY = math.sin(vehicle.attitude.yaw + (1.5708)) * spdlow                 
                send_ned_velocity(dX, dY, 0, 1)
                print("1.5 m/s")
            else:     
                dX = math.cos(vehicle.attitude.yaw + (1.5708)) * spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708)) * spd                 
                send_ned_velocity(dX, dY, 0, 1)
                print("3 m/s")
            if dly_mode > 2:
                fly_mode = 6
                dly_mode = 0 
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc
                print("------- LAND -------")
        elif fly_mode == 6:
            change_mode(mode = "LAND")

def control(value):
    allowed_keys = ['q', 'o', 'i', 'p', 'a', 'space', 'n', 'm', 't', 'l', 'g', 'r', ',', '.', 's', 'b']
    if value in allowed_keys:
        if value == 'space':
            change_alt(step="INC")
        if value == 'tab':
            change_alt(step="DEC")
        if value == 't':
            print("TAKEOFF")
            if int(vehicle.location.global_relative_frame.alt) <= 0.5:
                basic_takeoff(altitude=3)
            time.sleep(1)
            print("Ketinggian Tercapai")
            change_mode(mode="AUTO")
            print("auto")
            time.sleep(3)
            change_mode(mode="GUIDED")
            time.sleep(1)
            misi_outdoor()
        if value == 'l':
            change_mode(mode="LAND")
        if value == 'g':
            change_mode(mode="GUIDED")
        if value == 'r':
            change_mode(mode="RTL")
        if value == 'a':
            change_mode(mode="AUTO")
            print("auto")
        if value == 'l' or value == 'p' or value == 'i' or value == 'b':
            navigation(value=value)
    else:
        print("Enter a valid Key!!!")

def navigation(value):
    angle = int(vehicle.heading)
    loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
    global old_loc, nn, dX, dY, npid
    new_loc = loc
    default_distance = 2
    d_y = vehicle.attitude.yaw * (180 / math.pi)
    print("NAVIGASI ")
    nn = 0     
    if value == 'b':       
        misi_outdoor()
    elif value == "l":
        change_mode(mode="LAND")
    else:
        print("Tidak Ada")

def press(key):
    print(f"'{key}' is pressed")
    control(value=key)

def write_data_to_csv(writer):
    global_relative_frame = vehicle.location.global_relative_frame
    local_frame = vehicle.location.local_frame
    writer.writerow([
        time.time(),
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        global_relative_frame.alt,
        local_frame.north,
        local_frame.east,
        local_frame.down
    ])
def log():
    print("save data")
    write_data_to_csv(writer)
    file.flush()  # Ensure data is written to the file


def main():
    global writer
    global file
    global vehicle
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    vehicle = connect('/dev/serial0', baud=57600)
    with open('drone_data.csv', mode='w', newline='') as file:
        print("masuk open")
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Relative Altitude', 'Local X', 'Local Y', 'Local Z'])
        listen_keyboard(on_press=press)
        try:
            while True:
                print("save data")
                write_data_to_csv(writer)
                file.flush()  # Ensure data is written to the file
                time.sleep(1)
        except KeyboardInterrupt:
            print("Data collection stopped by user")
        finally:
            vehicle.close()

relay_ch = 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

if __name__ == "__main__":
    main()
