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


while True:

    start = get_telemetry()
    print('Start point global position: lat={}, lon={}'.format(start.lat, start.lon))
    rospy.sleep(5)
    print('please wait for 25 sec')
    rospy.sleep(15)
    print('please wait for 10 sec')
    rospy.sleep(10)
