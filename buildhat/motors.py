### DO NOT MODIFY CODE IN THIS FILE ###

'''
Lego Motor:
    Description:
        This code can be used to run either type of Spike motor. This includes accessing data 
        from the encoder, such as the motor position and speed, as well as setting the motor to
        run. Functionality includes setting the motor power, the motor speed, or even the motor
        position.

    Hardware:
        These motors can be connected to any of the BuildHAT motor ports. The letter associated 
        with each port can be seen just behind the ports, going from A to D counterclockwise.
        
        More info:
        https://www.raspberrypi.com/documentation/accessories/build-hat.html
    
    Initialization:
        motor_name = Motor('port')

        If you plug your motor into port A and want to name it 'left_motor', your initializaion
        will look like this:

        left_motor = Motor('A')

    Motor.start(speed (optional)):
        This function can be used to begin running a motor. The speed, ranging from -100 to 100,
        can be set here, but if not input is given it will use default_speed.

    Motor.stop()
        This function stops the motor, setting its speed to 0.
        
    Motor.run_PID(Kp, Ki, Kd, windup, speed(optional))
        This function runs the motor using a PID controller to keep the speed at a desired value.
        Without a speed input, the motor will use default_speed. PID values for start are .003, 
        .01, and 0, respectively. The windup defines the maximum value from the integral gain,
        typically 100.
        
    Motor.pwm(speed)
        This function takes an input from -1 to 1 and feeds this to the motor as a square wave with
        constant frequency and duty cycle given as the speed input. This effectively allows for 
        direct control of the motor force.
        
    Motor.set_default_speed(default_speed):
        This function can be used to set the default speed of the motor, ranging from -100 
        to 100 as a proportion of the maximum speed of the motor. This is automatically set 
        to 20.

    Motor.run_for_rotations(rotations, speed (optional), blocking (optional)):
        This function can be used to run the motor for a set number of revolutions. The speed,
        ranging from -100 to 100, can be set here, but if no input is given it will use 
        default_speed. One can also set whether this function has blocking. This is a 
        boolean value which is true if it is not set at the function call. While true,
        blocking is enabled, meaning that the next line of code will not be run until
        the motor has finished running for the required number of rotations. If blocking 
        is false, the next line of code will immediately be run while the motor is 
        turning.

    Motor.run_for_degrees(degrees, speed (optional), blocking (optional)):
        This function can be used to run the motor for a set number of degrees. The speed,
        ranging from -100 to 100, can be set here, but if no input is given it will use 
        default_speed. One can also set whether this function has blocking. This is a 
        boolean value which is true if it is not set at the function call. While true,
        blocking is enabled, meaning that the next line of code will not be run until
        the motor has finished running for the required number of degrees. If blocking 
        is false, the next line of code will immediately be run while the motor is 
        turning.

    Motor.run_to_position(degrees, speed (optional), blocking (optional), direction (optional)):
        This function can be used to run the motor to a set position in degrees, ranging from -180 
        to 180. The speed, ranging from -100 to 100, can be set here, but if no input is given it 
        will use default_speed. The direction taken by the motor can also be set, with 'shortest'
        as the default and 'clockwise' and 'anticlockwise' being other options. One can also set 
        whether this function has blocking. This is a boolean value which is true if it is not 
        set at the function call. While true, blocking is enabled, meaning that the next line of 
        code will not be run until the motor has finished running to the desired position. If 
        blocking is false, the next line of code will immediately be run while the motor is 
        turning.

    Motor.run_for_seconds(seconds, speed(optional), blocking(optional)):
        This function can be used to run the motor for a set length of time. The speed,
        ranging from -100 to 100, can be set here, but if no input is given it will use 
        default_speed. One can also set whether this function has blocking. This is a 
        boolean value which is true if it is not set at the function call. While true,
        blocking is enabled, meaning that the next line of code will not be run until
        the motor has finished running for the required number of seconds. If blocking 
        is false, the next line of code will immediately be run while the motor is 
        turning.

    Motor.get_position():
        Returns the current position of the motor, in degrees, relative to some starting position.
        This can be either positive or negative and is not limited in value. 
    
    Motor.get_apostition():
        Returns the current absolute position of the motors, in degrees, in terms of a single rotation. 
        This can be positive or negative but is restricted to be between -180 and 180.

    Motor.get_speed():
        Returns the current speed of the motor, ranging from -100 to 100.
    
    EncoderExamples.py and Run_Examples.py: 
        Usage of this code is demonstrated in the example files for these motors in the Examples
        folder on your Pi's desktop.

'''

### DO NOT MODIFY CODE BELOW THIS LINE ###
import threading
import time
from collections import deque
from concurrent.futures import Future
from enum import Enum
from threading import Condition

from .devices import Device
from .exc import MotorError


class Motor(Device):
    """Motor device

    :param port: Port of device
    :raises DeviceError: Occurs if there is no motor attached to port
    """

    def __init__(self, port):
        """Initialise motor

        :param port: Port of device
        """
        super().__init__(port)
        self.default_speed = 20
        self._currentspeed = 0
        if self._typeid in {38}:
            self.mode([(1, 0), (2, 0)])
            self._combi = "1 0 2 0"
            self._noapos = True
        else:
            self.mode([(1, 0), (2, 0), (3, 0)])
            self._combi = "1 0 2 0 3 0"
            self._noapos = False
        self.plimit(0.7)
        self.pwmparams(0.65, 0.01)
        self._rpm = False
        self._release = True
        self._bqueue = deque(maxlen=5)
        self._cvqueue = Condition()
        self.when_rotated = None
        self._oldpos = None
        self._runmode = MotorRunmode.NONE

    def run_PID(self, Kp, Ki, Kd, windup, speed=None):
        self._wait_for_nonblocking()
        if self._runmode == MotorRunmode.FREE:
            if self._currentspeed == speed:
                # Already running at this speed, do nothing
                return
        elif self._runmode != MotorRunmode.NONE:
            # Motor is running some other mode, wait for it to stop or stop() it yourself
            return

        if speed is None:
            speed = self.default_speed
        else:
            if not (speed >= -100 and speed <= 100):
                raise MotorError("Invalid Speed")
        speed = self._speed_process(speed)
        cmd = f"port {self.port} ; set {speed}\r"
        if self._runmode == MotorRunmode.NONE:
            self.run_for_degrees(0)
            pid = f"pid {self.port} 0 0 s1 1 0 {Kp} {Ki} {Kd} {windup} 0.01; "
            cmd = (f"port {self.port} ; select 0 ; selrate {self._interval}; "
                   f"{pid}"
                   f"set {speed}\r")
        self._runmode = MotorRunmode.FREE
        self._currentspeed = speed
        self._write(cmd)
    
    def start(self, speed=None):
        """Start motor

        :param speed: Speed ranging from -100 to 100
        :raises MotorError: Occurs when invalid speed specified
        """
        if speed is None:
            speed = self.default_speed
        self.run_PID(.003, .01, 0, 100, speed )

    def stop(self):
        """Stop motor"""
        self._wait_for_nonblocking()
        self._runmode = MotorRunmode.NONE
        self._currentspeed = 0
        self.coast()
    
    def set_default_speed(self, default_speed):
        """Set the default speed of the motor

        :param default_speed: Speed ranging from -100 to 100
        :raises MotorError: Occurs if invalid speed passed
        """
        if not (default_speed >= -100 and default_speed <= 100):
            raise MotorError("Invalid Speed")
        self.default_speed = default_speed
    
    def run_for_rotations(self, rotations, speed=None, blocking=True):
        """Run motor for N rotations

        :param rotations: Number of rotations
        :param speed: Speed ranging from -100 to 100
        :param blocking: Whether call should block till finished
        :raises MotorError: Occurs if invalid speed passed
        """
        self._runmode = MotorRunmode.DEGREES
        if speed is None:
            self.run_for_degrees(int(rotations * 360), self.default_speed, blocking)
        else:
            if not (speed >= -100 and speed <= 100):
                raise MotorError("Invalid Speed")
            self.run_for_degrees(int(rotations * 360), speed, blocking)

    def run_for_degrees(self, degrees, speed=None, blocking=True):
        """Run motor for N degrees

        Speed of 1 means 1 revolution / second

        :param degrees: Number of degrees to rotate
        :param speed: Speed ranging from -100 to 100
        :param blocking: Whether call should block till finished
        :raises MotorError: Occurs if invalid speed passed
        """
        self._runmode = MotorRunmode.DEGREES
        if speed is None:
            speed = self.default_speed
        if not (speed >= -100 and speed <= 100):
            raise MotorError("Invalid Speed")
        if not blocking:
            self._queue((self._run_for_degrees, (degrees, speed)))
        else:
            self._wait_for_nonblocking()
            self._run_for_degrees(degrees, speed)

    def run_to_position(self, degrees, speed=None, blocking=True, direction="shortest"):
        """Run motor to position (in degrees)

        :param degrees: Position in degrees from -180 to 180
        :param speed: Speed ranging from 0 to 100
        :param blocking: Whether call should block till finished
        :param direction: shortest (default)/clockwise/anticlockwise
        :raises MotorError: Occurs if invalid speed or angle passed
        """
        self._runmode = MotorRunmode.DEGREES
        if speed is None:
            speed = self.default_speed
        if not (speed >= 0 and speed <= 100):
            raise MotorError("Invalid Speed")
        if degrees < -180 or degrees > 180:
            raise MotorError("Invalid angle")
        if not blocking:
            self._queue((self._run_to_position, (degrees, speed, direction)))
        else:
            self._wait_for_nonblocking()
            self._run_to_position(degrees, speed, direction)



    def run_for_seconds(self, seconds, speed=None, blocking=True):
        """Run motor for N seconds

        :param seconds: Time in seconds
        :param speed: Speed ranging from -100 to 100
        :param blocking: Whether call should block till finished
        :raises MotorError: Occurs when invalid speed specified
        """
        self._runmode = MotorRunmode.SECONDS
        if speed is None:
            speed = self.default_speed
        if not (speed >= -100 and speed <= 100):
            raise MotorError("Invalid Speed")
        if not blocking:
            self._queue((self._run_for_seconds, (seconds, speed)))
        else:
            self._wait_for_nonblocking()
            self._run_for_seconds(seconds, speed)

    def pwmparams(self, pwmthresh, minpwm):
        """PWM thresholds

        :param pwmthresh: Value 0 to 1, threshold below, will switch from fast to slow, PWM
        :param minpwm: Value 0 to 1, threshold below which it switches off the drive altogether
        :raises MotorError: Occurs if invalid values are passed
        """
        if not (pwmthresh >= 0 and pwmthresh <= 1):
            raise MotorError("pwmthresh should be 0 to 1")
        if not (minpwm >= 0 and minpwm <= 1):
            raise MotorError("minpwm should be 0 to 1")
        self._write(f"port {self.port} ; pwmparams {pwmthresh} {minpwm}\r")

    def pwm(self, pwmv):
        """PWM motor

        :param pwmv: Value -1 to 1
        :raises MotorError: Occurs if invalid pwm value passed
        """
        if not (pwmv >= -1 and pwmv <= 1):
            raise MotorError("pwm should be -1 to 1")
        self._write(f"port {self.port} ; pwm ; set {pwmv}\r")


    def get_position(self):
        """Get position of motor with relation to preset position (can be negative or positive)

        :return: Position of motor in degrees from preset position
        :rtype: int
        """
        return self.get()[1]

    def get_aposition(self):
        """Get absolute position of motor

        :return: Absolute position of motor from -180 to 180
        :rtype: int
        """
        if self._noapos:
            raise MotorError("No absolute position with this motor")
        else:
            return self.get()[2]

    def get_speed(self):
        """Get speed of motor

        :return: Speed of motor
        :rtype: int
        """
        return self.get()[0]  

    def plimit(self, plimit):
        """Limit power

        :param plimit: Value 0 to 1
        :raises MotorError: Occurs if invalid plimit value passed
        """
        if not (plimit >= 0 and plimit <= 1):
            raise MotorError("plimit should be 0 to 1")
        self._write(f"port {self.port} ; port_plimit {plimit}\r")

        
    def coast(self):
        """Coast motor"""
        self._write(f"port {self.port} ; coast\r")

    def float(self):
        """Float motor"""
        self.pwm(0)

    @property
    def release(self):
        """Determine if motor is released after running, so can be turned by hand

        :getter: Returns whether motor is released, so can be turned by hand
        :setter: Sets whether motor is released, so can be turned by hand
        :return: Whether motor is released, so can be turned by hand
        :rtype: bool
        """
        return self._release

    @release.setter
    def release(self, value):
        """Determine if the motor is released after running, so can be turned by hand

        :param value: Whether motor should be released, so can be turned by hand
        :type value: bool
        """
        if not isinstance(value, bool):
            raise MotorError("Must pass boolean")
        self._release = value  

    def _run_for_degrees(self, degrees, speed):
        self._runmode = MotorRunmode.DEGREES
        mul = 1
        if speed < 0:
            speed = abs(speed)
            mul = -1
        pos = self.get_position()
        newpos = ((degrees * mul) + pos) / 360.0
        pos /= 360.0
        self._run_positional_ramp(pos, newpos, speed)
        self._runmode = MotorRunmode.NONE

    def _run_to_position(self, degrees, speed, direction):
        self._runmode = MotorRunmode.DEGREES
        data = self.get()
        pos = data[1]
        if self._noapos:
            apos = pos
        else:
            apos = data[2]
        diff = (degrees - apos + 180) % 360 - 180
        newpos = (pos + diff) / 360
        v1 = (degrees - apos) % 360
        v2 = (apos - degrees) % 360
        mul = 1
        if diff > 0:
            mul = -1
        diff = sorted([diff, mul * (v2 if abs(diff) == v1 else v1)])
        if direction == "shortest":
            pass
        elif direction == "clockwise":
            newpos = (pos + diff[1]) / 360
        elif direction == "anticlockwise":
            newpos = (pos + diff[0]) / 360
        else:
            raise MotorError("Invalid direction, should be: shortest, clockwise or anticlockwise")
        # Convert current motor position to decimal rotations from preset position to match newpos units
        pos /= 360.0
        self._run_positional_ramp(pos, newpos, speed)
        self._runmode = MotorRunmode.NONE

    def _run_positional_ramp(self, pos, newpos, speed):
        """Ramp motor

        :param pos: Current motor position in decimal rotations (from preset position)
        :param newpos: New motor postion in decimal rotations (from preset position)
        :param speed: -100 to 100
        """
        if self._rpm:
            speed = self._speed_process(speed)
        else:
            speed *= 0.05  # Collapse speed range to -5 to 5
        dur = abs((newpos - pos) / speed)
        cmd = (f"port {self.port}; select 0 ; selrate {self._interval}; "
               f"pid {self.port} 0 1 s4 0.0027777778 0 5 0 .1 3 0.01; "
               f"set ramp {pos} {newpos} {dur} 0\r")
        ftr = Future()
        self._hat.rampftr[self.port].append(ftr)
        self._write(cmd)
        ftr.result()
        if self._release:
            time.sleep(0.2)
            self.coast()

    def _run_for_seconds(self, seconds, speed):
        speed = self._speed_process(speed)
        self._runmode = MotorRunmode.SECONDS
        if self._rpm:
            pid = f"pid_diff {self.port} 0 5 s2 0.0027777778 1 0 2.5 0 .4 0.01; "
        else:
            pid = f"pid {self.port} 0 0 s1 1 0 0.003 0.01 0 100 0.01;"
        cmd = (f"port {self.port} ; select 0 ; selrate {self._interval}; "
               f"{pid}"
               f"set pulse {speed} 0.0 {seconds} 0\r")
        ftr = Future()
        self._hat.pulseftr[self.port].append(ftr)
        self._write(cmd)
        ftr.result()
        if self._release:
            self.coast()
        self._runmode = MotorRunmode.NONE


    @property
    def when_rotated(self):
        """
        Handle rotation events

        :getter: Returns function to be called when rotated
        :setter: Sets function to be called when rotated
        :return: Callback function
        """
        return self._when_rotated

    def _intermediate(self, data):
        if self._noapos:
            speed, pos = data
            apos = None
        else:
            speed, pos, apos = data
        if self._oldpos is None:
            self._oldpos = pos
            return
        if abs(pos - self._oldpos) >= 1:
            if self._when_rotated is not None:
                self._when_rotated(speed, pos, apos)
            self._oldpos = pos

    @when_rotated.setter
    def when_rotated(self, value):
        """Call back, when motor has been rotated

        :param value: Callback function
        """
        self._when_rotated = value
        self.callback(self._intermediate)

    def bias(self, bias):
        """Bias motor

        :param bias: Value 0 to 1
        :raises MotorError: Occurs if invalid bias value passed

        .. deprecated:: 0.6.0
        """  # noqa: RST303
        raise MotorError("Bias no longer available")


    def _queue(self, cmd):
        Device._instance.motorqueue[self.port].put(cmd)

    def _wait_for_nonblocking(self):
        """Wait for nonblocking commands to finish"""
        Device._instance.motorqueue[self.port].join()

    def _speed_process(self, speed):
        """Lower speed value"""
        if self._rpm:
            return speed / 60
        else:
            return speed
 
class MotorRunmode(Enum):
    """Current mode motor is in"""

    NONE = 0
    FREE = 1
    DEGREES = 2
    SECONDS = 3