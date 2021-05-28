import rospy
import time
import panda_driver_motors
from motors_panda.msg import Motors

def callback(data):
    rospy.loginfo("I heard LEFT: %f", data.data_l)
    rospy.loginfo("I heard RIGHT: %f", data.data_r)
    panda_driver_motors.move_dir_FW (data.data_l,data.data_r)
    #time.sleep(1)
    #panda_driver_motors.stop()
    #time.sleep(1)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Motors, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
