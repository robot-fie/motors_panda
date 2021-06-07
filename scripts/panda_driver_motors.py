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

def enable_motor (enable):
    print("Enable motors")
    pca9685.set_PWM(enable,100)

def disable_motor (enable):
    print("Disable motor")
    pca9685.set_PWM(enable,0)

def stop_motor (dir):
    print("Stopping Motor")
    pca9685.set_PWM(dir,0)

def move_motor (dir,duty):
    print ("Moving Motor")
    pca9685.set_PWM(dir,duty)

def disable_motor_and_stop (enable, dir):
    print("Disable motor and stop")
    disable_motor(enable)
    stop_motor (dir)

def enable_motor_and_move (enable, dir, duty):
    print("Enable motor and move")
    enable_motor (motor)
    move_motor(dir,duty)

def move_motors (motors, direction):
    for m, d in zip(motors, direction):
        print(f'Move motors: {m},{d[0]},{d[1]}')
        enable_motor_and_move(m,d[0],d[1])

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

STP  = 0
FRW  = 1
BKW  = -1

class motor():
    motor_HW_EN = 3
    motor_HW_FW = 4
    motor_HW_BW = 11

    curr_mov = FRW

    def __init__ (self, EN_pin, FW_pin, BW_pin):
        self.motor_HW_EN = EN_pin
        print("Enable en el pin: ", self.motor_HW_EN)
        self.motor_HW_FW = FW_pin
        print("Forward en el pin: ", self.motor_HW_FW)
        self.motor_HW_BW = BW_pin
        print("Backward en el pin: ",self.motor_HW_BW) 
        self.curr_mov = FRW

    def enable(self):
        enable_motor(self.motor_HW_EN)

    def disable(self):
        disable_motor(self.motor_HW_EN)

    def stop(self):
        stop_motor(self.motor_HW_FW)
        stop_motor(self.motor_HW_BW)
        curr_mov = STP

    def move_motor(self, frac):
        local_duty = abs(int(frac*PWM_DUTY))
        print ("Local duty set to: ",local_duty)
        print ("Frac is set to: ",frac)
        if frac*self.curr_mov < 0.0:
            print ("New direction is note the previous one, STOP")
            print ("Prod of frac*self.curr is: ",frac*self.curr_mov) 
            self.stop()

        if frac > 0.0:
            print ("New direction is FW")
            move_motor(self.motor_HW_FW,local_duty)
            self.curr_mov = FRW

        if frac < 0.0:
            print ("New direction is BW")
            move_motor(self.motor_HW_BW,local_duty)
            self.curr_mov = BKW

        if frac == 0.0:
            self.stop()
            self.curr_mov = STP

    def shutdown(self):
        self.stop()
        self.disable()

if __name__ == "__main__":
    motor_right_front = motor(right_front_EN,right_front_IN_FW,right_front_IN_BW)
    stop()
    time.sleep(1)
    motor_right_front.enable()
    motor_right_front.move_motor(0.5)
    time.sleep(1)
    motor_right_front.move_motor(-0.5)
    time.sleep(1)
    motor_right_front.move_motor(0.5)
    time.sleep(1)
    motor_right_front.move_motor(-0.5)
    time.sleep(1)
    motor_right_front.stop()
    motor_right_front.shutdown()
    stop()
