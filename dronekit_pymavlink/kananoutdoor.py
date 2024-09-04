
 
# Import Necessary Packages
import numpy as np
import cv2
from gpiozero import AngularServo 
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time, math
import smbus
 
 
 
from sshkeyboard import listen_keyboard
import RPi.GPIO as GPIO
 
bus = smbus.SMBus(1) # Change the I2C bus number based on the actual device
 
address = 0x1E # Radar default address 0x10
address2 = 0x10
getLidarDataCmd = [0x5A,0x05,0x00,0x01,0x60] # Gets the distance value instruction


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
 
def deteksiObj():
    global v1,v2,v3,dX1,dY1,dZ1
    v1=0
    v2=0
    tgtX = 320
    tgtY = 240
    Kp = 1.5
    Kd = 1
    Ki = 0
    I=0
    prevError=0
    i=700
    t=0
    error=0
    
   # webcam = cv2.VideoCapture(0)
 
    while(1):       
        _, imageFrame = webcam.read()
        
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        
        # HL = cv2.getTrackbarPos('HL','image')
        # HH = cv2.getTrackbarPos('HH','image')
 
        # SL = cv2.getTrackbarPos('SL','image')
        # SH = cv2.getTrackbarPos('SH','image')
 
        # VL = cv2.getTrackbarPos('VL','image')
        # VH = cv2.getTrackbarPos('VH','image') 
        
        HL = 11
        HH = 23
 
        SL = 91
        SH = 255
 
        VL = 83
        VH = 165 
 
        red_lower = np.array([HL, SL, VL], np.uint8)
        red_upper = np.array([HH, SH, VH], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    
        kernel = np.ones((5, 5), "uint8")
        
        # For red color
        red_mask = cv2.dilate(red_mask, kernel)
        res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask)  
    
        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        x=0
        y=0
        w=0
        h=0
        area_big = 0
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 1000):
                if area_big<area:
                    area_big = area
                    x, y, w, h = cv2.boundingRect(contour)
                # break
        
        if area_big>0:            
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                    (x + w, y + h), 
                                    (0, 0, 255), 2)
            
            #cv2.putText(imageFrame, "Red Colour", (x, y),
            #            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            #            (0, 0, 255))
            imageFrame = cv2.circle(imageFrame,((x+int(w/2)),(y+int(h/2))),1,(0,255,0),3)
            
        # Program Termination
        # cv2.imshow("red", red_mask)
        imageFrame = cv2.circle(imageFrame,(tgtX,tgtY),1,(255,0,0),3)
        #cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        posX = x+(w/2)
        posY = y+(h/2)
 
        #PID obj
        v1=0
        if w!=0:
            dZ1=0
            #print ('ada obj')
            dlX = abs(posX-tgtX)
            dlY = abs(posY-tgtY)
            jrkTgt = math.sqrt(math.pow(dlX,2) + math.pow(dlY,2))
            
            cv2.putText(imageFrame, ("%s"%jrkTgt), (x, y),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            (0, 0, 255))
            
            actual_altitude = float(vehicle.location.global_relative_frame.alt)
            #print ("actual_altitude", actual_altitude)
            #print ("JARAK TARGET", jrkTgt)
            v1=1
            if (actual_altitude-0.10)>0.4:
                if jrkTgt < 110:
                    #print (jrkTgt)
                    #print ('turun')
                    dZ1=actual_altitude-0.10
                    v1=2
                    #altPID("DEC")
            else:
                v1=3
                #print("sudah dapat")
            
            spd = (jrkTgt*0.8)/600
            if v1==2:
               spd=0
                
            #PID
            error = spd
            P = error
            I = I + error
            D = error-prevError
            
            PIDvalue = (Kp*P) + (Ki*I) + (Kd*D)
            prevError = error
 
            px = posX - tgtX
            py = posY - tgtY
            
            if py<0:
                if px>0:
                    dgr = math.atan(px/py)+math.pi
                elif px<0:
                    dgr = math.atan(px/py)-math.pi
            elif py==0:
                dgr = math.atan(px/0.00001)
            else:
                dgr = math.atan(px/py)
            dX1 = math.cos(vehicle.attitude.yaw + dgr) * spd
            dY1 = math.sin(vehicle.attitude.yaw + dgr) * spd
            #print(dgr)
            v2=1
            #send_ned_velocity(dX,dY,dZ,1)
        else:
            v2=2
            #send_ned_velocity(0,0,0,2)
            
 
        if cv2.waitKey(2) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break    
 
def getLidarData(addr, cmd):
    bus.write_i2c_block_data(addr, 0x00, cmd)
    time.sleep(0.01)
    data = bus.read_i2c_block_data(addr, 0x00, 9)
    distance = data[0] | (data[1] << 8)
    strengh = data[2] | (data[3] << 8)
    temperature = (data[4] | (data[5] << 8)) / 100 
    print('distance LIDAR 2 = %5d cm, strengh = %5d, temperature = %5d ℃'%(distance, strengh, temperature))
    return int(distance)
def getLidarData2(addr, cmd):
    bus.write_i2c_block_data(addr, 0x00, cmd)
    time.sleep(0.01)
    data = bus.read_i2c_block_data(addr, 0x00, 9)
    distance = data[0] | (data[1] << 8)
    strengh = data[2] | (data[3] << 8)
    temperature = (data[4] | (data[5] << 8)) / 100 
    print('distance LIDAR1 = %5d cm, strengh = %5d, temperature = %5d ℃'%(distance, strengh, temperature))
    return int(distance)
 
def basic_takeoff(altitude):
 
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    vehicle.simple_takeoff(altitude)
    to=0
    while True:
        print("Reached Height = ", vehicle.location.global_relative_frame.alt)
 
        # if to == 0 and (vehicle.location.global_relative_frame.alt >= (altitude/2.0)):
        #     to=1
        #     change_mode(mode = "AUTO")
        if vehicle.location.global_relative_frame.alt >= 10:
            break
        time.sleep(1)

def indoor_takeoff(altitude):
 
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    vehicle.simple_takeoff(altitude)
    to=0
    while True:
        print("Reached Height = ", vehicle.location.global_relative_frame.alt)
 
        # if to == 0 and (vehicle.location.global_relative_frame.alt >= (altitude/2.0)):
        #     to=1
        #     change_mode(mode = "AUTO")
        if vehicle.location.global_relative_frame.alt >= 0.5:
            break
        time.sleep(1)
 
def outdoor_takeoff(altitude):
 
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    vehicle.simple_takeoff(altitude)
    to=0
    while True:
        print("Reached Height = ", vehicle.location.global_relative_frame.alt)
 
        # if to == 0 and (vehicle.location.global_relative_frame.alt >= (altitude/2.0)):
        #     to=1
        #     change_mode(mode = "AUTO")
        if vehicle.location.global_relative_frame.alt >= (altitude * 14.5):
            break
        time.sleep(1)
 
 
def change_mode(mode):
 
    """
 
    This function will change the mode of the Vehicle.
 
    Inputs:
        1.  mode            -   Vehicle's Mode
 
    """
 
    vehicle.mode = VehicleMode(mode)
 
 
def send_to(latitude, longitude, altitude):
 
    """
    Inputs:
        1.  latitude            -   Destination location's Latitude
        2.  longitude           -   Destination location's Longitude
        3.  altitude            -   Vehicle's flight Altitude
 
    """
 
    if vehicle.mode.name == "GUIDED":
        location = LocationGlobalRelative(latitude, longitude, float(altitude))
        print(f"lat:'{latitude}' -- lon:'{longitude}' -- alt:'{altitude}")
        vehicle.simple_goto(location,airspeed=0.5,groundspeed=0.2)
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
 
def altPID(step):
 
    actual_altitude = float(vehicle.location.global_relative_frame.alt)
    changed_altitude = [(actual_altitude + 0.20), (actual_altitude - 0.10)]
 
    if step == "INC":
        if changed_altitude[0] <= 1:
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
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
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
 
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        
def get_distance_metres(lat1, lon1, lat2, lon2):
 
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
 
def destination_location(homeLattitude, homeLongitude, distance, bearing):
 
    """
    Inputs:
        1.  homeLattitude       -   Home or Current Location's  Latitude
        2.  homeLongitude       -   Home or Current Location's  Latitude
        3.  distance            -   Distance from the home location
        4.  bearing             -   Bearing angle from the home location
 
    """
 
    #Radius of earth in metres
    R = 6371e3
 
    rlat1 = homeLattitude * (math.pi/180.0) 
    rlon1 = homeLongitude * (math.pi/180.0)
 
    d = distance
 
    #Converting bearing to radians
    bearing = 1.0*bearing * (math.pi/180.0)
 
    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))
 
    #Converting to degrees
 
    rlat2 = rlat2 * (180.0/math.pi) 
    rlon2 = rlon2 * (180.0/math.pi)
 
    # Lat and Long as an Array
    location = [rlat2, rlon2]
 
    return location
 
global npid
 
def kendali_PID():
    print("AUTO PID MODE")
    
    Kp = 50
    Kd = 0
    Ki = 0
    
    thp=0
    cnt=0
    I=0
    prevError=0
    i=700
    t=0
    error=0
    while npid==1:
        lidar2 = getLidarData(address, getLidarDataCmd)
        lidar1 = getLidarData2(address2, getLidarDataCmd)
        if thp==0:
            if lidar1 > 0 and lidar1 <= 40:
                error=-10
            elif lidar1 > 40 and lidar1 <= 50:
                error=-8
            elif lidar1 > 50 and lidar1 <= 60:
                error=-6
            elif lidar1 > 60 and lidar1 <= 70:
                error=-4
            elif lidar1 > 70 and lidar1 <= 80:
                error=-2
            elif lidar1 > 80 and lidar1 <= 90:
                error=-1                                                    
            elif lidar1 > 90 and lidar1 <= 100: #tengah
                error=0
            elif lidar1 > 100 and lidar1 <= 110:
                error=1
            elif lidar1 > 110 and lidar1 <= 120:
                error=2
            elif lidar1 > 120 and lidar1 <= 130:
                error=4
            elif lidar1 > 130 and lidar1 <= 140:
                error=6
            elif lidar1 > 140 and lidar1 <= 160:
                error=8
            elif lidar1 > 160 and lidar1 <= 180:
                error=10
            elif lidar1 > 180:
                error=15
 
            #PID
            P = error
            I = I + error
            D = error-prevError
            
            PIDvalue = (Kp*P) + (Ki*I) + (Kd*D)
            prevError = error
            
            print("PID 1 ")
            arahBlk = ((PIDvalue*1.5708)/400)
            print( arahBlk )
 
            if PIDvalue>=-50 and PIDvalue<=50: # lurus
                dX = math.cos(vehicle.attitude.yaw+arahBlk) * 0.2
                dY = math.sin(vehicle.attitude.yaw+arahBlk) * 0.2
                send_ned_velocity(dX,dY,0,1)
            elif PIDvalue>=-500 and PIDvalue<500:  # dekat kiri -> ke kanan
                dX = math.cos(vehicle.attitude.yaw+(arahBlk) ) * 0.1
                dY = math.sin(vehicle.attitude.yaw+(arahBlk) ) * 0.1
                send_ned_velocity(dX,dY,0,1)
            else:
                 print("else")
                 break
 
            if lidar2 < 150:
                send_ned_velocity(0,0,0,2)
                cnt=cnt+1
                if cnt>=3:
                    cnt=0
                    thp = 1
                    I=0
                    prevError=0
                    i=700
                    t=0
                    error=0
                    print("tahap keluar")
            else: cnt=0
            
        elif thp==1:
            if lidar2 > 0 and lidar2 <= 40:
                error=-10
            elif lidar2 > 40 and lidar2 <= 60:
                error=-8
            elif lidar2 > 60 and lidar2 <= 70:
                error=-6
            elif lidar2 > 70 and lidar2 <= 80:
                error=-4
            elif lidar2 > 80 and lidar2 <= 90:
                error=-2
            elif lidar2 > 90 and lidar2 <= 100:
                error=-1                                                    
            elif lidar2 > 100 and lidar2 <= 110: #tengah
                error=0
            elif lidar2 > 110 and lidar2 <= 120:
                error=1
            elif lidar2 > 120 and lidar2 <= 130:
                error=2
            elif lidar2 > 130 and lidar2 <= 140:
                error=4
            elif lidar2 > 140 and lidar2 <= 150:
                error=6
            elif lidar2 > 150 and lidar2 <= 160:
                error=8
            elif lidar2 > 160 and lidar2 <= 180:
                error=10
            elif lidar2 > 180:
                error=15
 
            #PID
            P = error
            I = I + error
            D = error-prevError
            
            PIDvalue = (Kp*P) + (Ki*I) + (Kd*D)
            prevError = error
            active_relay()
            print("PID 2")
            arahBlk = ((PIDvalue*1.5708)/400) - 1.5708
            print( arahBlk )
 
            if PIDvalue>=-50 and PIDvalue<=50: # lurus
                dX = math.cos(vehicle.attitude.yaw+arahBlk) * 0.2
                dY = math.sin(vehicle.attitude.yaw+arahBlk) * 0.2
                send_ned_velocity(dX,dY,0,1)
            elif PIDvalue>=-750 and PIDvalue<=750:  # dekat kiri -> ke kanan
                dX = math.cos(vehicle.attitude.yaw+(arahBlk) ) * 0.1
                dY = math.sin(vehicle.attitude.yaw+(arahBlk) ) * 0.1
                send_ned_velocity(dX,dY,0,1)
            else:
                 print("else")
                 break
            
            if lidar2>200:
                send_ned_velocity(0,0,0,2)
                cnt=cnt+2
                if cnt>=2:
                    cnt=0
                    thp = 2
                    I=0
                    prevError=0
                    i=700
                    t=0
                    error=0
                    print("tahap keluar")
            else: cnt=0
            
        elif thp==2:
            print("PID 3")
            dX = math.cos(vehicle.attitude.yaw - 1.5708) * 0.1
            dY = math.sin(vehicle.attitude.yaw - 1.5708) * 0.1
            send_ned_velocity(dX,dY,0,5)
            break
            
        time.sleep(0.5)
        
    send_ned_velocity(0,0,0,5)
    change_mode(mode = "LAND")
 
def misi_outdoor():
    print("MASUK KE MISI OUTDOOR")
    fly_mode=0
    dly_mode=0
    loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
    old_loc = loc
    spd=3
    spdlow=1
    while True:
        if fly_mode==0:
            if dly_mode == 0 and int(vehicle.location.global_relative_frame.alt) <= 0.9:
                outdoor_takeoff(altitude = 1)  
            
            dly_mode = dly_mode+1 
            
            if dly_mode>3:
                fly_mode = 1
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc
                
        if fly_mode == 1: #kanan 5 m    
            loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
 
            if d_n > 10: #jarak
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1
                #servo.angle = 90
            if 10>=d_n>8:
                dX = math.cos(vehicle.attitude.yaw + (1.5708))*spdlow
                dY = math.sin(vehicle.attitude.yaw + (1.5708))*spdlow
                send_ned_velocity(dX,dY,0,1)
            
            else:
                dX = math.cos(vehicle.attitude.yaw + (1.5708))*spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708))*spd                 
                send_ned_velocity(dX,dY,0,1)  
               
 
            if dly_mode>3:
                servo.angle = 90
                fly_mode = 2
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc
 
        elif fly_mode == 2: #kedepan 10 m
            loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
 
            if d_n > 15: # jarak
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1
                ## servo           

            if 15>=d_n>12:
                dX = math.cos(vehicle.attitude.yaw)*spdlow
                dY = math.sin(vehicle.attitude.yaw)*spdlow
                send_ned_velocity(dX,dY,0,1)
 
            else:
                dX = math.cos(vehicle.attitude.yaw)*spd 
                dY = math.sin(vehicle.attitude.yaw)*spd                 
                send_ned_velocity(dX,dY,0,1)
 
            if dly_mode>3:
                servo2.angle = 90
                fly_mode = 3
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
 
        elif fly_mode == 3: #kiri 10 m
            loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
 
            if d_n > 20: # jarak
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1
            if 20>=d_n>16:
                dX = math.cos(vehicle.attitude.yaw - (1.5708))*spdlow
                dY = math.sin(vehicle.attitude.yaw - (1.5708))*spdlow
                send_ned_velocity(dX,dY,0,1)

            else:
                dY = math.sin(vehicle.attitude.yaw - (1.5708))*spd                 
                dX = math.cos(vehicle.attitude.yaw - (1.5708))*spd 
                send_ned_velocity(dX,dY,0,1)  
                
            if dly_mode>3:
                servo3.angle = 90
                fly_mode = 4
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
 
        elif fly_mode == 4: #mundur 10 m
            loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
 
            if d_n > 15: # jarak
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1

            if 20>=d_n>16:
                dX = math.cos(vehicle.attitude.yaw + (1.5708))*spdlow
                dY = math.sin(vehicle.attitude.yaw + (1.5708))*spdlow
                send_ned_velocity(dX,dY,0,1)   
            
            else:
                dX = math.cos(vehicle.attitude.yaw + (1.5708)*2)*spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708)*2)*spd                 
                send_ned_velocity(dX,dY,0,1)
 
            if dly_mode>3:
                servo4.angle = 90
                fly_mode = 5
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc   
 
        elif fly_mode == 5: #kanan 5 m
            loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0], loc[1])       
 
            if d_n > 10: # jarak
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1

            if 20>=d_n>16:
                dX = math.cos(vehicle.attitude.yaw + (1.5708))*spdlow
                dY = math.sin(vehicle.attitude.yaw + (1.5708))*spdlow
                send_ned_velocity(dX,dY,0,1)
            else:     
                dX = math.cos(vehicle.attitude.yaw + (1.5708))*spd 
                dY = math.sin(vehicle.attitude.yaw + (1.5708))*spd                 
                send_ned_velocity(dX,dY,0,1)  
            if dly_mode>3:
                fly_mode = 6
                dly_mode = 0 
                loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
                old_loc = loc 
 
        elif fly_mode==6:
            change_mode(mode = "LAND")
 
def otoImage():
    flg=0
    fly_mode=1
    dly_mode=0
    while True:
        if fly_mode == 1:
            print ("v1:",v1," v2:",v2)
            if v1==1:
                print ('ada obj')
                time.sleep(1)
            elif v1==2:
                altPID("DEC")
                print ('turun')
                flg=flg+1
                time.sleep(1)
            elif v1==3:
                print ('sdh dapat')
                time.sleep(1)
            
            if flg>0:
                flg=flg+1
                altPID("DEC")
                print ('turun..') 
                time.sleep(1)
                if(flg>=2):
                    flg=0
                
            if v2==1 and flg==0:
                print ('ngepasin')
                send_ned_velocity(dX1,dY1,0,1)
            elif v2==2:
                print ('berhenti')
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1
            
            if dly_mode>20:
                fly_mode = 2
                dly_mode = 0
 
        elif fly_mode ==2: # terbang naik
            if dly_mode == 0 and int(vehicle.location.global_relative_frame.alt) <= 0.67:
                basic_takeoff(altitude = 0.7)  
            
            dly_mode = dly_mode+1 
            
            if dly_mode>3:
                fly_mode = 3
                dly_mode = 0
 
        elif fly_mode == 3:
            navigation('p')
            break
        
def control(value):
 
    """
        t             -       Take-Off
        l             -       Land
        g             -       Guided Mode
        r             -       RTL Mode
 
    """
    allowed_keys = ['q','o','i','p','a','space', 'n', 'm',  't', 'l', 'g', 'r', ',', '.','s','b']
 
    if value in allowed_keys:
 
        if value == 'o':
            otoImage()
            
        if value == 'space':
            change_alt(step = "INC")
 
        if value == 'tab':
            change_alt(step = "DEC")
 
        if value == 't':
            if int(vehicle.location.global_relative_frame.alt) <= 0.95:
                basic_takeoff(altitude = 1)
            time.sleep(2)
            change_mode(mode = "AUTO")
            print("auto")
            time.sleep(5)
            change_mode(mode = "GUIDED")
            time.sleep(2)
            #otoImage()  # ambil obj kemudian tembok
            #navigation('p') # tembok
            #navigation('b') # outdoor
            #indoorS5()
            misi_outdoor()
            
        if value == 'l':
            change_mode(mode = "LAND")
 
        if value == 'g':
            change_mode(mode = "GUIDED")
 
        if value == 'r':
            change_mode(mode = "RTL")
 
        if value == 'a':
            change_mode(mode = "AUTO")
            print("auto")
            
        if value == 'm':
            deactive_relay()
            print("MAGNET ON")
 
        if value == 'n':
            active_relay()
            print("MAGNET OF")
 
        # if value in allowed_keys[-4:]:
        if value == 'l' or value == 'p' or value == 'i' or value == 'b'  :
            navigation(value = value)
            
 
    else:
        print("Enter a valid Key!!!")
 
def navigation(value):

 
    # Vehicle Location
    angle = int(vehicle.heading)
    loc   = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
    global old_loc, nn, dX, dY, npid
    new_loc = loc
    # Default Distance in meters
    default_distance = 2
    d_y = vehicle.attitude.yaw * (180/math.pi)
    print("NAVIGASI ")
 
 
    nn = 0
    if value == 'p':
        npid=1
        kendali_PID()
    elif value == 'i':
        deteksiObj()        
    elif value == 'b':       
        misi_outdoor()
    elif value == "l":
        change_mode (mode = "LAND")
    else:
        print("Tidak Ada")
 
def press(key):
 
    """
    Inputs:
        1.  key         -   Pressed keyboard Key
 
    """
 
    print(f"'{key}' is pressed")
 
    # Sending Control Inputs
    control(value = key)
    
def indoorS5():
    print("CODINGAN S5")
    fly_mode = 0
    dly_mode = 0
    loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)
    old_loc = loc
    spd = 0.3
    
    while True:
        if fly_mode == 0:
            print("MASUK 0")
            dly_mode = dly_mode + 1
            
            if dly_mode > 3:
                fly_mode = 1
                dly_mode = 0
                loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
                old_loc = loc
        
        if fly_mode == 1: #Maju 2 M
            print("MAJU 1 M")
            loc = (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0],loc[1])
            
            if d_n > 2:
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode + 1
            
            else:
                dX = math.cos(vehicle.attitude.yaw)*spd
                dY = math.sin(vehicle.attitude.yaw)*spd
                send_ned_velocity(dX,dY,0,1)
            
            if dly_mode > 3:
                fly_mode = 2
                dly_mode = 0
                loc = (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt)
                old_loc = loc

        elif fly_mode == 2: #BERHENTI
            print("BERHENTI")
            send_ned_velocity(0,0,0,1)
            dly_mode = dly_mode + 1        
            if dly_mode > 3:
                fly_mode = 3
                dly_mode = 0
        
        elif fly_mode == 3: #TURUN
            
            change_alt('DEC')
            print("TURUN")
            send_ned_velocity(0,0,0,1)
            dly_mode = dly_mode + 1        
            if dly_mode > 3:
                fly_mode = 4
                dly_mode = 0
                
        elif fly_mode == 4:
            print("NAIKK")
            change_alt('INC')
            send_ned_velocity(0,0,0,1)
            dly_mode = dly_mode + 1
            
            if dly_mode > 3:
                fly_mode = 5
                dly_mode = 0
        
        elif fly_mode == 5:
            print('MODE 5')
            loc = (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt)
            d_n = get_distance_metres(old_loc[0], old_loc[1], loc[0],loc[1])
            
            if d_n > 2:
                send_ned_velocity(0,0,0,1)
                dly_mode = dly_mode+1
            else:
                dX = math.cos(vehicle.attitude.yaw)*spd
                dY = math.sin(vehicle.attitude.yaw)*spd
                send_ned_velocity(dX,dY,0,1)
            
            if dly_mode > 3:
                fly_mode = 6
                dly_mode = 0
                loc = (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt)
                old_loc = loc
        elif fly_mode == 6:
            change_mode(mode = "LAND")
                
 
def main():
 
    # Declaring Vehicle as global variable
    global vehicle
    global relay_ch
    global webcam
    webcam = cv2.VideoCapture(0)
    relay_ch = 17                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(relay_ch, GPIO.OUT)
 
    # Connecting the Vehicle
    vehicle = connect('/dev/serial0', baud=57600)
 
    # Setting the Heading angle constant throughout flight
    if vehicle.parameters['WP_YAW_BEHAVIOR'] != 0:
        vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")
 
    # Listen Keyboard Keys
    listen_keyboard(on_press=press)

relay_ch = 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay_ch, GPIO.OUT)
def active_relay():
    GPIO.setup(relay_ch, GPIO.HIGH)
    
def deactive_relay():
    GPIO.setup(relay_ch, GPIO.LOW)
 
 
if __name__ == "__main__":
    main()                                                                                                           


