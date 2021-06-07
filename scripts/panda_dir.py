import rospy
import time
import panda_driver_motors as pdm
from motors_panda.msg import Motors

motor_right_front = pdm.motor(pdm.right_front_EN,pdm.right_front_IN_FW,pdm.right_front_IN_BW)
motor_right_back  = pdm.motor(pdm.right_back_EN,pdm.right_back_IN_FW,pdm.right_back_IN_BW)
motor_left_front  = pdm.motor(pdm.left_front_EN,pdm.left_front_IN_FW,pdm.left_front_IN_BW)
motor_left_back   = pdm.motor(pdm.left_back_EN,pdm.left_back_IN_FW,pdm.left_back_IN_BW)

def callback(data):
    rospy.loginfo("I heard LEFT: %f", data.data_l)
    rospy.loginfo("I heard RIGHT: %f", data.data_r)
    if (data.data_l == 0.0 and data.data_r == 0.0):
        motor_right_front.stop()
        motor_right_back.stop()
        motor_left_front.stop()
        motor_left_back.stop()
    else:
        motor_right_front.move_motor(data.data_r)                                                                   
        motor_right_back.move_motor(data.data_r)                                                                    
        motor_left_front.move_motor(data.data_l)                                                                    
        motor_left_back.move_motor(data.data_l)
    #time.sleep(1)
    #panda_driver_motors.stop()
    #time.sleep(1)

def listener():
    motor_right_front.enable()
    motor_right_back.enable()
    motor_left_front.enable()
    motor_left_back.enable()
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tracks_duty_cycle", Motors, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        motor_right_front.shutdown()                                                                        
        motor_right_back.shutdown()                                                                         
        motor_left_front.shutdown()                                                                         
        motor_left_back.shutdown()
