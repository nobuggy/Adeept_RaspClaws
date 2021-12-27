from typing import Set
from dataclasses import dataclass
from enum import IntEnum
import time
import threading

import Adafruit_PCA9685

import logging
logger = logging.getLogger('robot')

#                      LEDS
#               ------------------
#              /                  \
#   L1   ------       Camera       ------    R1
#    ====|    o         /\         o    |====
#        ------         ||         ------
#             |                    |
#   L2   ------                    ------    R2
#    ====|    o                    o    |====
#        ------                    ------
#             |                    |
#   L3   ------                    ------    R3
#    ====|    o                    o    |====
#        ------                    ------
#             |                    |
#              \                  /
#               ------------------
#
#    Ellbow
#        ___       Shoulder (forward + / backward -)     
#       /   \ -------+--
#       | O |        |
#       |   |--------+--
#       \   |
#        \  |
#  OUT +  \ |   IN -
#    <===  \| ===>
#
# Mapping PWM to Legs
#        shoulder  ellbow
#   L1     0         1
#
#
# Adeept AD002 Servos:
#  110ms / 60°  aka 330ms/180°
#  180° range
#  PWM Period 50Hz (20ms)
#
#  https://www.adeept.com/learn/tutorial-162.html
#

AD002_DC_MIN = 100   # min duty cycle for PWM
AD002_DC_MAX = 560   # max value
AD002_ANGLE_RANGE = 180   # angular range between min/max duty cycle
AD002_ADJUST_SPEED = 560
AD002_PWM_PERIOD_MS = 20   # millis
AD002_PWM_PERIOD_HZ = 1000//AD002_PWM_PERIOD_MS   # Hz
AD002_DCTICKS_PER_1DEGREE = (AD002_DC_MAX - AD002_DC_MIN)/180
AD002_ANGLE_PER_DCTICK = (AD002_DC_MAX - AD002_DC_MIN)/180
AD002_DCTICK_ADJUST_SPEED_MS = 330 / (AD002_DC_MAX - AD002_DC_MIN)

SERVO_MIN_DCSTEP = 10
SERVO_MAX_DCSTEP = AD002_DC_MAX - AD002_DC_MIN



PCA9685 = Adafruit_PCA9685.PCA9685()
PCA9685.set_pwm_freq(AD002_PWM_PERIOD_HZ)  # 50Hz

def setpwm(pwmChannel:int, value:int):
    assert 0 <= pwmChannel <= 11, f'Illegal PWM channel: {pwmChannel}'
    assert AD002_DC_MIN <= value <= AD002_DC_MAX, f'Illegal servo angle value: {value}'
    print(f'PWM{pwmChannel}={value}')
    PCA9685.set_pwm(pwmChannel, 0, value)

    
class Robot:
    pass

@dataclass
class MoveInstr:
    leg:str
    shoulder:int
    ellbow:int

@dataclass
class Servo:
    minDC = AD002_DC_MIN
    maxDC = AD002_DC_MAX
    currDC = (maxDC-minDC)//2
    goalDC = -1
    ttime = 0

    def schedule():
        dcdiff = self.goalDC - self.currDC
        if dcdiff == 0:
            return
        minsteps = int(math.ceil(dcdiff / SERVO_MAX_DCSTEP))
        maxsteps = int(math.floor(dcdiff / SERVO_MIN_DCSTEP))
        now = time.monotonic()
        tdiff = self.ttime - now
        tmin = max( AD002_PWM_PERIOD_MS,
                    dcdiff * AD002_DCTICK_ADJUST_SPEED_MS) # time for servo to perform angle diff
        if tdiff < tmin:
            logger.error('Not enough time to adjust servo')
            tdiff = tmin
        else:
            pass
            

class Leg:
    def __init__(self, pwm:int, fwdDir:int, outDir:int) -> None:
        assert pwm >= 0 and pwm <= 11
        self.pwm = pwm
        self.shoulderAngle = 0
        self.ellbowAngle = 0
        
NEUTRAL = (345, 255)  # Left/right
STANCES = {
    #         Left/right
    'N': (   0,   0 ),
    'F': (  30,  30 ),
    'B': ( -30, -30 ),
}

LEG2SHOULDER_PWM = {
    'L1':  0, 'L2':2, 'L3': 4,
    'R1': 10, 'R2':8, 'R3': 6
}
LEG2ELLBOW_PWM = {
    'L1':  1, 'L2':3, 'L3': 5,
    'R1': 11, 'R2':9, 'R3': 7
}

G1 = {'L1','R2','L3'}
G2 = {'R1','L2','R3'}


def set_pwm_group(grp:Set[str], stance:str):
    for leg in grp:
        lr = int(leg[0]=='R')
        setpwm(LEG2ELLBOW_PWM[leg], NEUTRAL[lr] + STANCES[stance][lr])
        
CRAB_RIGHT = 'NN,FN,FB,NB'


class GaitEngine(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.goFlag = threading.Event()
        self.stoppedFlag = threading.Event()
        self.goFlag.clear()
        self.stoppedFlag.set()
        self.groups = (set(), set())
        self.schedule = []
        self.delay = 1.0

    def stop (self) -> None:
        self.goFlag.clear()
        self.stoppedFlag.wait()
        
    def gait(self, g1, g2, sched):
        self.stop()
        self.schedule = sched.split(',')
        self.groups = (g1, g2)
        self.goFlag.set()

    def run (self) -> None:
        try:
            while True:
                print('Gait engine waiting for GO...')
                self.goFlag.wait()
                print('Gait engine running...')
                self.stoppedFlag.clear()
                while self.goFlag.is_set():
                    for step1,step2 in self.schedule:
                        if not self.goFlag.is_set():
                            break
                        g1, g2 = self.groups
                        print(f'{g1} {step1} / {g2} {step2}')
                        set_pwm_group(g1, step1)
                        set_pwm_group(g2, step2)
                        time.sleep(self.delay)
                self.stoppedFlag.set()
        except Exception as exc:
            logger.error('GaitEngine failed: %s', exc, exc_info=True)

gaitEngine = GaitEngine()
gaitEngine.start()
