import RPi.GPIO as GPIO
from components.Wheel import *
from components.Sensor import *
import time


LEFT_WHEEL = {
    'name': 'left_wheel',
    'enable_pin': 22,
    'input1': 5,
    'input2': 6,
    'freq' : 1000,
    'speed' : 90
}

RIGHT_WHEEL = {
    'name': 'right_wheel',
    'enable_pin': 26,
    'input1': 13,
    'input2': 19,
    'freq' : 1000,
    'speed' : 90
}

SENSOR = {
    '1': 12,
    '2': 16,
    '3': 20,
    '4': 21
}


class Charles():

    def __init__(self):

        # initial motor speed
        self.initial_motor_speed = 80

        # turn speed
        self.turn_speed = 80

        self.minimum_speed = 60
        self.maximum_speed = 100

        # define pid constants
        self.Kp = 25
        self.Ki = 1
        self.Kd = 25

        self.error = 0
        self.P = 0
        self.I = 0
        self.D = 0
        self.PID_value = 0
        self.previous_error = 0
        self.previous_I = 0

        self.flag = 0

        self.sensor_obj = Sensor()
        self.left_wheel = Wheel()
        self.right_wheel = Wheel()

    def setup(self):

        # initialize sensors io
        # set sensors ios as digital input
        self.sensor_obj.initialize(SENSOR)
        # initialize motor io
        # set motor input as digital output
        # set ENA and ENB as output (PWM)
        self.left_wheel.initialize_wheel(LEFT_WHEEL)
        self.right_wheel.initialize_wheel(RIGHT_WHEEL) 

        print('CHARLES Initialized')

    def navigate(self):

        self.read_sensor_values()
        #print('Error: {}'.format(error))

        #make left turn until it detects straight path
        if self.error == 100:

            while True:
                
                # analogWrite(ENA, 110) #Left Motor Speed
                # analogWrite(ENB, 90) #Right Motor Speed
                self.left_wheel.changePWM(self.turn_speed)
                self.right_wheel.changePWM(self.turn_speed)

                self.sharpLeftTurn()
                self.read_sensor_values()
                if self.error == 0:
                    break

        elif self.error == 101:
            while True:
                self.left_wheel.changePWM(self.turn_speed)
                self.right_wheel.changePWM(self.turn_speed)

                self.sharpRightTurn()
                self.read_sensor_values()
                if self.error == 0:
                    break
            # analogWrite(ENA, 110) #Left Motor Speed
            # analogWrite(ENB, 90) #Right Motor Speed
            #self.left_wheel.changePWM(self.turn_speed)
            #self.right_wheel.changePWM(self.turn_speed)

            # self.forward()
            # time.sleep(0.2)
            # self.stop_bot()
            # self.read_sensor_values()
            # if self.error == 102:
            #     while True:
            #         # analogWrite(ENA, 110); //Left Motor Speed
            #         # analogWrite(ENB, 90); //Right Motor Speed
            #         self.left_wheel.changePWM(self.turn_speed)
            #         self.right_wheel.changePWM(self.turn_speed)
            #         self.sharpRightTurn()
            #         self.read_sensor_values()
            #         if self.error == 0:
            #             break

        # elif self.error == 102:
        #     while True:

        #         self.left_wheel.changePWM(self.turn_speed)
        #         self.right_wheel.changePWM(self.turn_speed)
        #         # analogWrite(ENA, 110); //Left Motor Speed
        #         # analogWrite(ENB, 90); //Right Motor Speed
        #         self.sharpLeftTurn()
        #         self.read_sensor_values()

        #         if self.error == 0:
        #             self.stop_bot()
        #             time.sleep(0.5)
        #             break

        # elif self.error == 103:
        #     self.forward()
        #     time.sleep(0.2)
        #     self.read_sensor_values()
        #     while self.error == 103:
        #         self.read_sensor_values()
        #         self.stop_bot()


        


        

        # Make right turn in case of it detects only right path 
        # (it will go into forward direction in case of straight and right "|--")
        # until it detects straight path
        # elif error == 101:
            
        #     # analogWrite(ENA, 110) #Left Motor Speed
        #     # analogWrite(ENB, 90) #Right Motor Speed
        #     left_wheel.changePWM(turn_speed)
        #     right_wheel.changePWM(turn_speed)

        #     forward()
        #     time.sleep(0.2)
        #     stop_bot()
        #     read_sensor_values()
        #     if error == 102:
        #         while True:
        #             # analogWrite(ENA, 110); //Left Motor Speed
        #             # analogWrite(ENB, 90); //Right Motor Speed
        #             left_wheel.changePWM(turn_speed)
        #             right_wheel.changePWM(turn_speed)
        #             sharpRightTurn()
        #             read_sensor_values()
        #             if error == 0:
        #                 break
        # elif error == 101:
        
        # # analogWrite(ENA, 110) #Left Motor Speed
        # # analogWrite(ENB, 90) #Right Motor Speed
        #     left_wheel.changePWM(turn_speed)
        #     right_wheel.changePWM(turn_speed)

        #     while True:
        #         read_sensor_values()
        #         # analogWrite(ENA, 110) #Left Motor Speed
        #         # analogWrite(ENB, 90) #Right Motor Speed
        #         left_wheel.changePWM(turn_speed)
        #         right_wheel.changePWM(turn_speed)

        #         sharpRightTurn()
        #         if error == 0:
        #             break

        # # Make left turn untill it detects straight path
        # elif error == 102:
        #     while True:

        #         left_wheel.changePWM(turn_speed)
        #         right_wheel.changePWM(turn_speed)
        #         # analogWrite(ENA, 110); //Left Motor Speed
        #         # analogWrite(ENB, 90); //Right Motor Speed
        #         sharpLeftTurn()
        #         read_sensor_values()

        #         if error == 0:
        #             stop_bot()
        #             time.sleep(0.2)
        #             break
        
        # Make left turn untill it detects straight path or stop if dead end reached.
        # elif error == 103:
        #     if flag == 0:
                
        #         left_wheel.changePWM(turn_speed)
        #         right_wheel.changePWM(turn_speed)
        #         # analogWrite(ENA, 110); //Left Motor Speed
        #         # analogWrite(ENB, 90); //Right Motor Speed
        #         forward()
        #         time.sleep(0.2)
        #         stop_bot()
        #         read_sensor_values()
        #         if error == 103:
        #             stop_bot()
        #             flag = 1
        #             print('Status: {}'.format('Dead End Reached.'))
        #         else:
                    
        #             # analogWrite(ENA, 110); //Left Motor Speed
        #             # analogWrite(ENB, 90); //Right Motor Speed
        #             left_wheel.changePWM(turn_speed)
        #             right_wheel.changePWM(turn_speed)
        #             sharpLeftTurn()
        #             time.sleep(0.2)

        #             while True:
        #                 read_sensor_values()
        #                 # analogWrite(ENA, 110); //Left Motor Speed
        #                 # analogWrite(ENB, 90); //Right Motor Speed
        #                 left_wheel.changePWM(turn_speed)
        #                 right_wheel.changePWM(turn_speed)
        #                 sharpLeftTurn()
        #                 if error == 0:
        #                     break
        # else:
        #     calculate_pid()
        #     motor_control()

        # if self.error == 100:
        #     pass
        # elif self.error == 101:
        #     pass
        # elif self.error == 102:
        #     pass
        # elif self.error == 103:
        #     pass
        else:
            self.calculate_pid()
            self.motor_control()
            print("--------------")

    def read_sensor_values(self):

        sensor = self.sensor_obj.getInputValues()

        if sensor[0] == 1 and sensor[1] == 0 and sensor[2] == 0 and sensor[3] == 0:
            self.error = 3
        elif sensor[0] == 1 and sensor[1] == 1 and sensor[2] == 0 and sensor[3] == 0:
            self.error = 100
        elif sensor[0] == 0 and sensor[1] == 1 and sensor[2] == 0 and sensor[3] == 0:
            self.error = 1
        elif sensor[0] == 0 and sensor[1] == 1 and sensor[2] == 1 and sensor[3] == 0:
            self.error = 0
        elif sensor[0] == 0 and sensor[1] == 0 and sensor[2] == 1 and sensor[3] == 0:
            self.error = -1
        elif sensor[0] == 0 and sensor[1] == 0 and sensor[2] == 1 and sensor[3] == 1:
            self.error = 101
        elif sensor[0] == 0 and sensor[1] == 0 and sensor[2] == 0 and sensor[3] == 1:
            self.error = -3
        elif sensor[0] == 1 and sensor[1] == 1 and sensor[2] == 1 and sensor[3] == 0: #Turn robot left side
            self.error = 100
        elif sensor[0] == 0 and sensor[1] == 1 and sensor[2] == 1 and sensor[3] == 1: #Turn robot right side
            self.error = 101
        elif sensor[0] == 0 and sensor[1] == 0 and sensor[2] == 0 and sensor[3] == 0: #Make U turn
            self.error = 102
        elif sensor[0] == 1 and sensor[1] == 1 and sensor[2] == 1 and sensor[3] == 1: #Turn left side or stop
            self.error = 103

        print('Sensors {}'.format(sensor))
        print('Error {}'.format(self.error))

    def calculate_pid(self):

        self.P = self.error
        self.I = self.I + self.previous_I
        self.D = self.error - self.previous_error

        self.PID_value = (self.Kp * self.P) + (self.Ki * self.I) + (self.Kd * self.D)
        #self.PID_value = (self.Kp * self.P) + (self.Ki * self.I) + (self.Kd * self.D)

        self.previous_I = self.I
        self.previous_error = self.error

        print('P-I-D {}-{}-{}:{}'.format(self.P,self.I,self.D,self.PID_value))

    def motor_control(self):

        # Calculating the effective motor speed
        def normalize_speed(minimum_speed,maximum_speed, input_speed):
            return (input_speed * maximum_speed) / minimum_speed

        def clip_function(minimum_speed,maximum_speed, input_speed):
            return min(maximum_speed,max(minimum_speed,input_speed))

        left_motor_speed = self.initial_motor_speed - self.PID_value
        right_motor_speed = self.initial_motor_speed + self.PID_value

        # print('Raw LMS:{}'.format(left_motor_speed))
        # print('Raw RMS:{}'.format(right_motor_speed))
        # print('PID val:{}'.format(self.PID_value))

        left_motor_speed = clip_function(self.minimum_speed,self.maximum_speed, left_motor_speed)
        right_motor_speed = clip_function(self.minimum_speed,self.maximum_speed, right_motor_speed)

        print('LMS-RMS:{}-{}'.format(left_motor_speed,right_motor_speed))

        self.left_wheel.changePWM(left_motor_speed)
        self.right_wheel.changePWM(right_motor_speed)

        self.forward()

        
    def forward(self):
        self.left_wheel.forward()
        self.right_wheel.forward()

    def reverse(self):
        self.left_wheel.backward()
        self.right_wheel.backward()

    def right(self):
        self.left_wheel.forward()
        self.right_wheel.stop()

    def left(self):
        self.left_wheel.stop()
        self.right_wheel.forward()

    def sharpRightTurn(self):
        self.left_wheel.forward()
        self.right_wheel.backward()

    def sharpLeftTurn(self):
        self.left_wheel.backward()
        self.right_wheel.forward()

    def stop_bot(self):
        self.left_wheel.stop()
        self.right_wheel.stop()


charles = Charles()

charles.setup()

while True:
    charles.navigate()
