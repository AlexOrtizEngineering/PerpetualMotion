# ////////////////////////////////////////////////////////////////
# //                     IMPORT STATEMENTS                      //
# ////////////////////////////////////////////////////////////////
import os
import math
import sys
import time
import threading

os.environ["DISPLAY"] = ":0.0"

from kivy.app import App
from kivy.lang import Builder
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import *
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.uix.behaviors import ButtonBehavior
from kivy.clock import Clock
from kivy.animation import Animation
from functools import partial
from kivy.config import Config
from kivy.core.window import Window
from pidev.kivy import DPEAButton
from pidev.kivy import PauseScreen
from time import sleep
from dpeaDPi.DPiComputer import *
from dpeaDPi.DPiStepper import *

# ////////////////////////////////////////////////////////////////
# //                     HARDWARE SETUP                         //
# ////////////////////////////////////////////////////////////////
"""Stepper Motor goes into MOTOR 0 )
    Limit Switch associated with Stepper Motor goes into HOME 0
    One Sensor goes into IN 0
    Another Sensor goes into IN 1
    Servo Motor associated with the Gate goes into SERVO 1
    Motor Controller for DC Motor associated with the Stairs goes into SERVO 0"""


# ////////////////////////////////////////////////////////////////
# //                      GLOBAL VARIABLES                      //
# //                         CONSTANTS                          //
# ////////////////////////////////////////////////////////////////
ON = False
OFF = True
HOME = True
TOP = False
OPEN = False
CLOSE = True
YELLOW = .180, 0.188, 0.980, 1
BLUE = 0.917, 0.796, 0.380, 1
DEBOUNCE = 0.1
INIT_RAMP_SPEED = 7
RAMP_LENGTH = 725

DC_NUM = 0
SERVO_NUM = 1
SERVO_POS = 1
STEPPER_ENABLED = False
STEPPER_NUM = 0
STAIRCASE_STATUS = 0


# ////////////////////////////////////////////////////////////////
# //            DECLARE APP CLASS AND SCREENMANAGER             //
# //                     LOAD KIVY FILE                         //
# ////////////////////////////////////////////////////////////////
class MyApp(App):
    def build(self):
        self.title = "Perpetual Motion"
        return sm

Builder.load_file('main.kv')
Window.clearcolor = (.1, .1,.1, 1) # (WHITE)


# ////////////////////////////////////////////////////////////////
# //                    SLUSH/HARDWARE SETUP                    //
# ////////////////////////////////////////////////////////////////
sm = ScreenManager()

# ////////////////////////////////////////////////////////////////
# //                       MAIN FUNCTIONS                       //
# //             SHOULD INTERACT DIRECTLY WITH HARDWARE         //
# ////////////////////////////////////////////////////////////////
	
# ////////////////////////////////////////////////////////////////
# //        DEFINE MAINSCREEN CLASS THAT KIVY RECOGNIZES        //
# //                                                            //
# //   KIVY UI CAN INTERACT DIRECTLY W/ THE FUNCTIONS DEFINED   //
# //     CORRESPONDS TO BUTTON/SLIDER/WIDGET "on_release"       //
# //                                                            //
# //   SHOULD REFERENCE MAIN FUNCTIONS WITHIN THESE FUNCTIONS   //
# //      SHOULD NOT INTERACT DIRECTLY WITH THE HARDWARE        //
# ////////////////////////////////////////////////////////////////

# Define motor and motor board
dpiStepper = DPiStepper()
dpiStepper.setBoardNumber(STEPPER_NUM)
if not dpiStepper.initialize():
    print("Communication with the DPiStepper board failed.")
dpiStepper.enableMotors(False)

# Create a DPiComputer object and initialize to default values
dpiComputer = DPiComputer()
if not dpiComputer.initialize():
    print("Communication with the DPiComputer board failed.")

"""
    Precondition: SERVO_POS is correctly declared upon initialization
"""
def openGate (position):
    global SERVO_POS

    dpiComputer.writeServo(SERVO_NUM, position)
    if position == 180:
        SERVO_POS = 0
    elif position == 0:
        SERVO_POS = 1

"""
    Precondition: STEPPER_ENABLED is correctly declared upon initialization
    Updated the motor position and corresponding global constant
"""
def update_motor(boolean):
    global STEPPER_ENABLED
    dpiStepper.enableMotors(boolean)
    STEPPER_ENABLED = boolean

"""
    Precondition: Speed has been properly updated, Stepper motor is connected to Motor STEPPER_NUM
    Runs the stepper motor associated with the ramp
"""
def run_motor(motor_direction, move_back):
    global STEPPER_NUM
    while not isBallAtTopOfRamp(): #can get it to only moev 28 revs instead
        dpiStepper.moveToRelativePositionInRevolutions(STEPPER_NUM, motor_direction * 1, False)
    sleep(2)
    if move_back:
        moveToHome()
    update_motor(False)
    event.cancel()

"""
    Moves ramp back to home for safety reasons
"""
def moveToHome():
    dpiStepper.moveToHomeInRevolutions(STEPPER_NUM, 1, 7, 1000)
    update_ramp_speed(INIT_RAMP_SPEED)

"""
    Updates stepper motor speed
"""
def update_ramp_speed(speed):
    dpiStepper.setSpeedInRevolutionsPerSecond(STEPPER_NUM, speed)
    dpiStepper.setAccelerationInRevolutionsPerSecondPerSecond(STEPPER_NUM, speed)

"""
    Precondition: Sensor is connected to IN 0 on dpi computer
    Checks if the ball is at the bottom of the ramp
"""
def isBallAtBottomOfRamp():
    sensor_val = dpiComputer.readDigitalIn(dpiComputer.IN_CONNECTOR__IN_0)
    if sensor_val == 0:
        sleep(DEBOUNCE)
        if sensor_val == 0:
            return True
    return False

"""
    Precondition: Sensor is connected to IN 1 on dpi computer
    Checks if the ball is at the top of the ramp
"""
def isBallAtTopOfRamp():
    sensor_val = dpiComputer.readDigitalIn(dpiComputer.IN_CONNECTOR__IN_1)
    if sensor_val == 0:
        sleep(DEBOUNCE)
        if sensor_val == 0:
            return True
    return False


class MainScreen(Screen):

    staircaseSpeedText = '0'
    rampSpeed = INIT_RAMP_SPEED
    staircaseSpeed = 40

    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.initialize()

    """
        Precondition: Servo motor is connected to SERVO SERVO_NUM on dpi computer, SERVO_NUM and SERVO_POS are correctly defined
        Changes position of servo motor associated with the gate
    """
    def toggleGate(self):
        global SERVO_NUM
        global SERVO_POS
        if SERVO_POS == 1:
            sleep(DEBOUNCE)
            if SERVO_POS == 1:
                openGate(180)
        elif SERVO_POS == 0:
            openGate(0)
        else:
            print("Error rotating servo motor")

    """
        Precondition: DC Motor is connected to SERVO SERVO_NUM on dpi computer, STAIRCASE_STATUS is correctly defined
        Either turns on or off DC Motor associated with staircase
    """
    def toggleStaircase(self):
        global STAIRCASE_STATUS
        if STAIRCASE_STATUS == 0:
            self.turnOnStaircase()
        elif STAIRCASE_STATUS == 1:
            self.turnOffStaircase()
        else:
            print("Error turning on staircase")

    """
        Precondition: Staircase is not already on, STAIRCASE_STATUS is correctly defined
        Activates the DC Motor associated with staircase
    """
    def turnOnStaircase(self):
        global STAIRCASE_STATUS
        dpiComputer.writeServo(DC_NUM, 180)
        STAIRCASE_STATUS = 1
        sleep(6)
        self.turnOffStaircase()

    """
        Precondition: Staircase is not already on, STAIRCASE_STATUS is correctly defined
        Turns off DC Motor associated to staircase
    """
    def turnOffStaircase(self):
        global STAIRCASE_STATUS
        dpiComputer.writeServo(DC_NUM, 90)
        STAIRCASE_STATUS = 0

    """
        Precondition: Sensor at bottom of ramp is working
        If the ball is detected at the bottom of ramp, enables stepper motor associated with ramp
    """
    def toggleRamp(self):
        if isBallAtBottomOfRamp():
            sleep(DEBOUNCE)
            if isBallAtBottomOfRamp():
                self.moveRamp(-1)
        else:
            moveToHome()

    """
        Enables stepper motor associated with ramp
    """
    def moveRamp(self, motor_direction, move_back = True):
        global event
        update_motor(True)
        event = Clock.schedule_interval(lambda dt: run_motor(motor_direction, move_back), 0.000001)

    """
        Precondition: Ramp is at the top before moveToHome() occurs
        Goes through one cycle of perpetual motion
    """
    def auto(self):
        self.one_round()
        Clock.schedule_once(lambda dt: moveToHome(), 0.01)

    """
        Precondition: Ball is either at the gate or at bottom of the ramp
    """
    def one_round(self):
        openGate(180)  # Opens gate
        while not isBallAtBottomOfRamp():  # While the ball isn't at bottom of ramp, program waits
            print("waiting")
            continue
        self.moveRamp(-1, False)  # Ramp carries ball up
        openGate(0)  # Closes gate for dramatics
        Clock.schedule_once(lambda dt: self.turnOnStaircase(), 0.01)  # Staircase turns on to carry ball

    """
        Precondition: update_ramp_speed correctly changes the stepper motor's velocity
    """
    def setRampSpeed(self, speed):
        update_ramp_speed(speed)
        print("Ramp speed: " + str(speed))
        self.ids.rampSpeedLabel.text = 'Ramp Speed: ' + str(speed)

    """
        Incomplete, not required
    """
    def setStaircaseSpeed(self, speed):
        print("Set the staircase speed and update slider text")

    def initialize(self):
        global STAIRCASE_STATUS
        dpiComputer.writeServo(SERVO_NUM, 0)
        print("Gate is closed")
        moveToHome()
        print("Ramp moved to home")
        dpiComputer.writeServo(DC_NUM, 90)
        STAIRCASE_STATUS = 0
        print("Staircase is off")

    def resetColors(self):
        self.ids.gate.color = YELLOW
        self.ids.staircase.color = YELLOW
        self.ids.ramp.color = YELLOW
        self.ids.auto.color = BLUE
    
    def quit(self):
        print("Exit")
        MyApp().stop()

sm.add_widget(MainScreen(name = 'main'))

# ////////////////////////////////////////////////////////////////
# //                          RUN APP                           //
# ////////////////////////////////////////////////////////////////
if __name__ == "__main__":
    # Window.fullscreen = True
    # Window.maximize()
    MyApp().run()

# Stop staircase and disable motors
dpiComputer.writeServo(DC_NUM, 90)
STAIRCASE_STATUS = 0
update_motor(False)
print("Motors are disabled")