import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from apscheduler.schedulers.blocking import BlockingScheduler
from datetime import datetime
rospy.init_node('flight')

def func_flight():
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    land = rospy.ServiceProxy('land', Trigger)

    def wait_arrival(tolerance=0.2):
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)


    start = get_telemetry(frame_id='map')

    print('Start point global position: lat={}, lon={}'.format(start.lat, start.lon))

    print('Take off 15 meters')

    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    wait_arrival()
    print('BASIC take off is done')
    rospy.sleep(4)
    navigate(frame_id='aruco_55', x=0, y=0, z=5)
    wait_arrival()
    print('ARUCO aruco_55 is done')
    rospy.sleep(4)


    telem = get_telemetry(frame_id='map')
    print('telem.z = ', telem.z)
    navigate_global(lat=start.lat, lon=start.lon, z=telem.z, speed=5, frame_id='map')
    wait_arrival()
    rospy.sleep(4)
    print('GPS take off is done')


    print('flying to the upper left side of the square')
    navigate_global(lat=start.lat+0.000045, lon=start.lon, z=telem.z, speed=5, frame_id='map')
    wait_arrival()

    print('flying to the upper right side of the square')
    navigate_global(lat=start.lat+0.000045, lon=start.lon + 0.000045, z=telem.z, speed=5, frame_id='map')
    wait_arrival()

    print('flying to the lower right side of the square')
    navigate_global(lat=start.lat, lon=start.lon + 0.000045, z=telem.z,  speed=5, frame_id='map')
    wait_arrival()

    print('flying to the starting position')
    navigate_global(lat=start.lat, lon=start.lon, z=telem.z, speed=5, frame_id='map')
    wait_arrival()

    print('Land')
    land()

    pass


scheduler = BlockingScheduler()
scheduler.add_job(func_flight, 'interval', hours=2, start_date=datetime.now())

scheduler.start()
