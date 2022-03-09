import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

vehicle = connect(connection_string, wait_ready=True)
# vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# vehicle = connect('/dev/ttyACM0',baud=115200,wait_ready=True)


def send_body_velocity(Cx, Cy, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        Cx, Cy, velocity_z,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if (vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95):
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(1.5)
print("Take off Selesai")

x = 5
y = 10

r = (x**2 + y**2)**0.5
t = r / 2 #m/s

Cx = x/t
Cy = y/t

i = round(t / 0.2) #s
for index in range(0, i):
    print("LAT >>", vehicle.location.global_relative_frame.lat, " || ", "LON >>", vehicle.location.global_relative_frame.lon)
    send_body_velocity(Cx, Cy, 0)
    time.sleep(0.2)

print("Close vehicle object")
vehicle.close()
sitl.stop()