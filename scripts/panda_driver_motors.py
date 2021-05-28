import PCA9685
import time

pca9685 = PCA9685.PCA9685()

PWM_DUTY = 80
PWM_FREQ = 1000

right_front_EN = 0
right_front_IN_BW = 1
right_front_IN_FW = 2

left_front_EN = 7
left_front_IN_BW = 5
left_front_IN_FW = 6

left_back_EN = 10
left_back_IN_BW = 8
left_back_IN_FW = 9

right_back_EN = 12
right_back_IN_BW = 13
right_back_IN_FW = 14

pca9685.set_freq(PWM_FREQ)
pca9685.set_PWM(right_front_EN,0)
pca9685.set_PWM(right_front_IN_BW,0)
pca9685.set_PWM(right_front_IN_FW,0)

pca9685.set_PWM(right_back_EN,0)
pca9685.set_PWM(right_back_IN_BW,0)
pca9685.set_PWM(right_back_IN_FW,0)

pca9685.set_PWM(left_front_EN,0)
pca9685.set_PWM(left_front_IN_BW,0)
pca9685.set_PWM(left_front_IN_FW,0)

pca9685.set_PWM(left_back_EN,0)
pca9685.set_PWM(left_back_IN_BW,0)
pca9685.set_PWM(left_back_IN_FW,0)

MOTORS = (right_front_EN, right_back_EN, left_front_EN, left_back_EN)

NORTH = ((right_front_IN_FW, PWM_DUTY),(right_back_IN_FW, PWM_DUTY),
         (left_front_IN_FW, PWM_DUTY),(left_back_IN_FW,PWM_DUTY))

def enable_motor (enable, dir, duty):
    print("Enable motors")
    pca9685.set_PWM(enable,100)
    pca9685.set_PWM(dir,duty)

def disable_motor (enable, dir):
    print("Disable motors")
    pca9685.set_PWM(enable,0)
    pca9685.set_PWM(dir,0)

def move_motors (motors, direction):
    for m, d in zip(motors, direction):
        print(f'Move motors: {m},{d[0]},{d[1]}')
        enable_motor(m,d[0],d[1])

def move_dir_FW (frac_izq, frac_der):
    duty_izq = int(PWM_DUTY*frac_izq)
    duty_der = int(PWM_DUTY*frac_der)
    direc = ((right_front_IN_FW, duty_der),(right_back_IN_FW, duty_der),
             (left_front_IN_FW, duty_izq),(left_back_IN_FW,duty_izq))
    move_motors(MOTORS,direc)

def stop():
        print(f'Stop')
        pca9685.set_PWM(right_front_EN,0)
        pca9685.set_PWM(right_front_IN_BW,0)
        pca9685.set_PWM(right_front_IN_FW,0)
        pca9685.set_PWM(right_back_EN,0)
        pca9685.set_PWM(right_back_IN_BW,0)
        pca9685.set_PWM(right_back_IN_FW,0)
        pca9685.set_PWM(left_front_EN,0)
        pca9685.set_PWM(left_front_IN_BW,0)
        pca9685.set_PWM(left_front_IN_FW,0)
        pca9685.set_PWM(left_back_EN,0)
        pca9685.set_PWM(left_back_IN_BW,0)
        pca9685.set_PWM(left_back_IN_FW,0)

# if __name__ == "__main__":
#     stop()
#     time.sleep(1)
#     move_dir_FW (0.9,0.4)
#     time.sleep(1)
#     stop()
#     time.sleep(1)
