import rospy
from clover import srv
from std_srvs.srv import Trigger
import math


rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

xkoef = 0.000045
ykoef = 0.000045

def navigate_wait(x=0, y=0, z=1.5, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def wait_arrival(tolerance=0.2):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

start = get_telemetry(frame_id='map')

print('Start point global position: lat={}, lon={}'.format(start.lat, start.lon))

print('Take off 2 meters local pos')
navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
wait_arrival()

rospy.sleep(1)

telem = get_telemetry(frame_id='map')

print('increase altitude to 5 meters using GPS')
navigate_global(lat=start.lat, lon=start.lon, z=(telem.z+3), speed=5, frame_id='map') 
wait_arrival()

print('mission starting')

navigate_global(lat=(start.lat+xkoef), lon=start.lon, z=(telem.z+3), speed=5, frame_id='map')
wait_arrival()
print('1 point reached')

navigate_global(lat=start.lat+xkoef, lon=start.lon + ykoef, z=(telem.z+3), speed=5, frame_id='map')
wait_arrival()
print('2 point reached')

navigate_global(lat=start.lat, lon=start.lon + ykoef, z=(telem.z+3),  speed=5, frame_id='map')
wait_arrival()
print('3 point reached')

navigate_global(lat=start.lat, lon=start.lon, z=(telem.z+3), speed=5, frame_id='map')
wait_arrival()
print('4 point reached')

navigate_global(lat=start.lat, lon=start.lon, z=telem.z, speed=5, frame_id='map')
wait_arrival()

print('Land')
land()
