# -*- coding: utf-8 -*-

# ------------------------------------------------------------------------------
# Copyright (c) 2015 Eric Pascual
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

""" This modules provides the Python equivalent the LeJOS chassis.

This implementation is not a one for one copy of the Java code. It retains the
principles from the original code, but adapt the implementation for trying
to benefit as much as possible from Python strengths.
"""

import math
import time
import threading

from ev3dev.motors import RegulatedMotor

__author__ = 'Eric Pascual'


class Wheel(object):
    """ The base model for wheels.

    Its main purpose is to control the motors driving them from the motions
    wanted at the wheel level. This takes in account the connecting gear train
    if any, the wheel type (standard, holonom,...)
    """
    def __init__(self, motor, diameter, gear_ratio=1.0, invert=False):
        """ Units used for dimensions do not matter, as long as they are coherent.
        For instance, if the wheel diameter is provided in millimeters, all the other
        dimensions involved in the robot model must be given in millimeters, and same
        for the motion distances.

        Args:
            motor (RegulatedMotor): the motor driving the wheel
            diameter (float): the wheel diameter
            gear_ratio (float): the gear ratio between the motor and the wheel.
            It is greater that 1 if the motor turns faster that the wheel. Default: 1.0.
            invert(bool): True in the wheel motion is inverted WRT implicit one.
                Default: False
        """
        if not motor:
            raise ValueError('motor must be specified')
        if not diameter or diameter <= 0:
            raise ValueError('diameter must be > 0')
        if not gear_ratio:
            raise ValueError('gear_ratio cannot be null')

        self._motor = motor
        self._diameter = float(diameter)    # be sure to do float arithmetic
        self._radius = self._diameter / 2   # often used
        self._gear_ratio = float(gear_ratio)
        self._invert = invert

        self._motor_pulses_per_rot = int(round(motor.count_per_rot * self._gear_ratio))
        self._motor_pulses_per_rd = int(round(self._motor_pulses_per_rot / math.pi / 2.))
        self._motor_pulses_per_deg = int(round(self._motor_pulses_per_rot / 360.))
        self._dist_per_pulse = diameter * math.pi / self._motor_pulses_per_rot

    @property
    def motor(self):
        """ Read-only access to the wheel motor

        :type: RegulatedMotor
        """
        return self._motor

    @property
    def diameter(self):
        """ Read-only access to the wheel diameter

        :type: float
        """
        return self._diameter

    @property
    def radius(self):
        """ Read-only access to the wheel radius (derived from the diameter
        for speeding up computations)

        :type: float
        """
        return self._radius

    @property
    def gear_ratio(self):
        """ Read-only access to the gear train ratio

        :type: float
        """
        return self._gear_ratio

    @property
    def invert(self):
        """ Read-only access to the inverted motion indicator

        :type: bool
        """
        return self._invert

    def radians(self, pulse_count):
        """ Returns the wheel rotation equivalent to a given amount of motor pulses.

        Args:
            pulse_count (float): the motor pulse count

        Returns:
            float: the resulting wheel rotation, in radians
        """
        return pulse_count / self._motor_pulses_per_rd

    def pulse_count(self, radians):
        """ Returns the motor pulse count equivalent to a given wheel rotation.

        Args:
            radians (float): the rotation angle in radians

        Returns:
            int: the equivalent pulse count
        """
        return int(round(radians * self._motor_pulses_per_rd))

    @property
    def angular_position_set_point(self):
        """ Set point of the wheel angular position (in radians)

        :type: float
        """
        return self.radians(self._motor.position_sp / self._gear_ratio)

    @angular_position_set_point.setter
    def angular_position_set_point(self, value):
        self._motor.position_sp = self.pulse_count(value * self._gear_ratio)

    @property
    def angular_speed_set_point(self):
        """ Set point of the wheel angular speed (in radians/sec)

        :type: float
        """
        return self.radians(self._motor.speed_sp / self._gear_ratio)

    @angular_speed_set_point.setter
    def angular_speed_set_point(self, value):
        self._motor.speed_sp = self.pulse_count(value * self._gear_ratio)

    @property
    def speed_regulation_enabled(self):
        """ Speed regulation enabling

        :type: bool
        """
        return self._motor.speed_regulation_enabled == RegulatedMotor.SPEED_REGULATION_ON

    @speed_regulation_enabled.setter
    def speed_regulation_enabled(self, value):
        self._motor.speed_regulation_enabled = \
            RegulatedMotor.SPEED_REGULATION_ON if value else RegulatedMotor.SPEED_REGULATION_OFF

    @property
    def linear_speed_sp(self):
        """ Wheel linear speed, in wheel unit per sec

        :type: float
        """
        return self.radians(self.angular_speed_set_point) * self._diameter / 2.0

    @linear_speed_sp.setter
    def linear_speed_sp(self, value):
        self.angular_speed_set_point = self.pulse_count(value * 2 / self._diameter)


class StandardWheel(Wheel):
    """ A standard wheel

    In addition to the generic wheel properties, it is characterized by its
    distance from the main axis of the chassis.
    """
    def __init__(self, motor, diameter, offset, gear_ratio=1.0, invert=False):
        """
        Args:
            motor: see :py:class:`Wheel`
            diameter: see :py:class:`Wheel`
            offset (float): distance of the wheel from the robot main axis
            gear_ratio: see :py:class:`Wheel`
            invert: see :py:class:`Wheel`
        """
        if not offset:
            raise ValueError('offset must be not null')

        super(StandardWheel, self).__init__(
            motor=motor, diameter=diameter, gear_ratio=gear_ratio, invert=invert
        )
        self._offset = offset

    @property
    def offset(self):
        """ Read-access to the wheel distance to the robot main axis.

        :type: float
        """
        return self._offset


class HolonomicWheel(StandardWheel):
    """ A holonomic wheel.

    In addition to the standard wheel, its distance is defined with respect to
    the robot center of rotation, and its position is defined by the angle of
    its rotation axis with the robot main axis.
    """
    def __init__(self, motor, diameter, offset, angle, gear_ratio=1.0, invert=False):
        """
        Args:
            motor: see :py:class:`StandardWheel`
            diameter: see :py:class:`StandardWheel`
            offset (float): distance of the wheel from the robot rotatioon center
            angle (float): angle of the wheel axis and the robot main axis
            gear_ratio: see :py:class:`StandardWheel`
            invert: see :py:class:`StandardWheel`
        """
        if angle is None or angle < 0:
            raise ValueError('angle is mandatory and must be >= 0')

        super(HolonomicWheel, self).__init__(
            motor=motor, diameter=diameter, gear_ratio=gear_ratio, offset=offset, invert=invert
        )
        self._angle = angle

    @property
    def angle(self):
        """ Read-access to the wheel axis angle to the robot main axis.

        :type: float
        """
        return self._angle


class WheeledChassis(object):
    """ Represents the chassis of a wheeled robot, whatever configuration is used.
    """
    _travel_speed = 0
    _rotate_speed = 0

    def __init__(self, wheels=None, motors_settings=None):
        """
        Args:
            wheels (iterable[Wheel]): the wheels of the chassis
            motors_settings (dict): optional properties for initializing the motors
        """
        self._check_wheels(wheels)
        self._wheels = wheels

        self.reset_motors()

        # configure the motors, using reasonable settings for not specified ones
        settings = {
            'duty_cycle_sp': 100,
            'speed_regulation': RegulatedMotor.SPEED_REGULATION_ON,
            'stop_command': RegulatedMotor.STOP_COMMAND_HOLD,
            'ramp_up_sp': 500,
            'ramp_down_sp': 500
        }
        if motors_settings:
            settings.update(motors_settings)
        self.setup_motors(**settings)

    @property
    def wheels(self):
        return self._wheels

    @property
    def travel_speed(self):
        """ The default robot travel speed, in wheel diameter units per second.

        .. note::

            The sign of the speed value is discarded. The direction can be controlled
            either by overriding the default value when calling :py:meth:`drive` or
            using methods :py:meth:`forward` and :py:meth:`backward`.

        :type: float
        """
        return self._travel_speed

    @travel_speed.setter
    def travel_speed(self, speed):
        self._travel_speed = abs(speed)

    @property
    def rotate_speed(self):
        """ The robot rotation speed, in degrees per second.

        .. note::

            The sign of the speed value is discarded. The direction can be controlled
            either by overriding the default value when calling :py:meth:`rotate` or
            using methods :py:meth:`rotate_right` and :py:meth:`rotate_left`.

        :type: float
        """
        return self._rotate_speed

    @rotate_speed.setter
    def rotate_speed(self, speed):
        self._rotate_speed = abs(speed)

    def reset_motors(self):
        """ Resets the motors
        """
        for w in self._wheels:
            w.motor.reset()

    def setup_motors(self, **kwargs):
        """ Configure the motors.

        Args:
            \**kwargs: the keyword arguments providing the desired attribute settings
        """
        for attr, value in kwargs.iteritems():
            for w in self._wheels:
                setattr(w.motor, attr, value)

    def _check_wheels(self, wheels):
        if not wheels:
            raise ValueError('a chassis needs wheels')

    def _wheels_sync_start(self, command):
        for w in self._wheels:
            w.motor.command = command


class DifferentialWheeledChassis(WheeledChassis):
    """ A chassis based on two wheels (or groups of wheels, or treads) which
    is controlled like a tank.
    """
    _travel_speed = 0
    _rotation_speed = 0

    def _check_wheels(self, wheels):
        """
        Args:
            wheels (list[StandardWheel]): the wheels of the chassis
        """
        super(DifferentialWheeledChassis, self)._check_wheels(wheels)
        if not all((isinstance(wheel, StandardWheel) for wheel in wheels)):
            raise ValueError('this chassis supports standard wheels only')

    def _monitor_move(self, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        monitor = MotionMonitor(self,
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )
        monitor.start()
        return monitor

    def travel(self, distance, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Travels for the specified distance at a given speed.

        This method is asynchronous, which means that it returns immediately. It returns
        the monitor created for tracking the motion.

        In addition, callable can be passed, which will be called on various events
        (start of the move, end of the move, stalled detection).

        Example::

            >>> from ev3dev.ev3 import LargeMotor
            >>>
            >>> wheel_left = StandardWheel(LargeMotor('out_B'), 43.2, -75)
            >>> wheel_right = StandardWheel(LargeMotor('out_C'), 43.2, 75)
            >>> chassis = DifferentialWheeledChassis((wheel_left, wheel_right))
            >>>
            >>> # synchronous usage with a forever wait
            >>> chassis.travel(250).wait()
            >>>
            >>> # asynchronous usage
            >>> def arrived(chassis):
            >>>     print('just arrived')
            >>>
            >>> mvt = chassis.travel(distance=250, speed=100, on_complete=arrived)
            >>> # ... do something while traveling
            >>>
            >>> # wait at most 10 secs for arrival at destination before doing something else
            >>> mvt.wait(delay=10)
            >>> if mvt.running:
            >>>     print("something wrong happened while traveling")
            >>> else:
            >>>     print("we are at destination")

        Args:
            distance (float): the distance to be traveled, in wheel diameter unit
            speed (float): optional local override of the default speed as set by :py:attr:`travel_speed`
            on_start: optional callback for motion start event handling
            on_complete: optional callback for completion event handling
            on_stalled: optional callback for stalled detection handling
            callback_args (dict): optional dictionary defining the kwargs passed to the callbacks

        Returns:
            MotionMonitor: the motion monitoring object
        """
        speed = speed or self._travel_speed

        if not speed:
            return NullMotionMonitor()

        for w in self._wheels:
            w.angular_position_set_point = w.pulse_count(distance / w.radius)

            w.speed_regulation_enabled = True
            w.angular_speed_set_point = w.pulse_count(speed / w.radius)

        self._wheels_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

        return self._monitor_move(
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )

    def arc(self, radius, angle, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Moves the robot along an arc with a specified radius and angle, after which the robot stops moving.

        If radius is positive, the robot turns left, the center of the circle being on its left side. If it
        is negative, the move takes place on the opposite side. If radius is null, the robot rotates in place.

        The sign of the angle gives the direction of the spin (CCW if positive, CW if negative). Hence the combined
        signs of radius and angle specify the direction of move (forward or backward) along the arc. If both are
        the same, the robot will move forward. If they are different it will move backwards.

        The robot will stop when its heading has changed by the provided angle.

        .. note::

            In case of rotation in place (null radius), the currently set angular speed is used.

            If provided, the sign of the speed will be ignored, since inferred by the ones of radius and angle.

        Args:
            radius (float): radius of the arc, 0 for a rotation in place
            angle (float): the heading change
            speed (float): the speed along the path, if different from default one
            on_start, on_on_complete, on_stalled, callback_args: see :py:meth:`drive`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        speed = abs(speed or self._travel_speed)

        if not speed:
            return NullMotionMonitor()

        for w in self._wheels:
            pulse_count = w.pulse_count((radius + w.offset) * math.radians(angle) / w.radius)
            w.angular_position_set_point = pulse_count

            w.speed_regulation_enabled = True
            w.angular_speed_set_point = pulse_count * speed / float(angle)

        self._wheels_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

        return self._monitor_move(
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )

    def rotate(self, angle, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Rotates for the specified amount of degrees.

        Nothing else but the special case of a null radius arc.

        See :py:meth:`arc` for detailed documentation.
        """
        return self.arc(0, angle, speed, on_start, on_complete, on_stalled, callback_args)

    def rotate_right(self, angle, **kwargs):
        """ Convenience shortcut to avoid taking care of the angle sign.

        The sign if the passed angle value is discarded.

        .. seealso::

            :py:meth:`rotate`
        """
        return self.rotate(-abs(angle), **kwargs)

    def rotate_left(self, angle, **kwargs):
        """ Same as :py:meth:`rotate_right` in the opposite direction.
        """
        return self.rotate(abs(angle), **kwargs)

    def stop(self, stop_command=None):
        """ Immediate stop, using the current motor stop setting.

        Args:
            stop_command (str): optional stop command, among `RegulatedMotor.STOP_COMMAND_xxx`.
                If not provided, use the current motors setting
        """
        for w in self._wheels:
            w.motor.stop(stop_command=stop_command)


class MotionMonitor(threading.Thread):
    """ An instance of this class is returned by pilot motion commands.

    It extends the standard :py:class:`threading.Thread` class by adding
    a couple of convenience methods and properties.
    """
    def __init__(self, chassis, on_start=None, on_complete=None, on_stalled=None, callback_args=None, **kwargs):
        """ All the callbacks receive the pilot as first argument, and can accept
        additional keyword parameters, which will contain the content
        of the `callback_args` dictionary passed here.

        Args:
            chassis (WheeledChassis): the associated pilot
            on_start (callable): an optional callback invoked when starting the motion
            on_complete (callable): an optional callback invoked at the normal completion of the motion.
            on_stalled (callable): an optional callback invoked when a motor stalled situation is detected
            callback_args (dict): an optional dictionary defining the kwargs which will be passed to the callbacks.
            \**kwargs: transmitted to super
        """
        super(MotionMonitor, self).__init__(**kwargs)
        self._chassis = chassis
        self._on_start = on_start
        self._on_complete = on_complete
        self._on_stalled = on_stalled
        self._callback_args = callback_args or {}
        self._stalled = False
        self._stopped = False

    @property
    def stalled(self):
        """ Tells if the motion was interrupted by a motor being stalled.

        :type: bool
        """
        return self._stalled

    @property
    def running(self):
        """ Tells if the motion is still ongoing..

        Can be used to test if it could complete within the wait delay.

        :type: bool
        """
        return self.is_alive()

    def wait(self, delay=60):
        """ Wait for the motion to be complete.

        Extends the inherited :py:meth:`threading.Thread.join` method by
        adding a default delay value. Although discouraged, it is allowed
        to pass `None` for a forever wait.

        The monitoring loop is stopped in case of timeout, to avoid
        callbacks being called at some later moment.

        Args:
            delay (float): the maximum wait time, in seconds.
        Returns:
            the instance, so the call can be chained with the motion command,
            while still returning the monitor to the caller
        """
        self.join(delay)
        if self.is_alive():
            self._stopped = True
        return self

    def stop(self):
        """ Stops the monitor and waits for the thread to end.
        """
        self._stopped = True
        self.join()

    def run(self):
        if self._on_start:
            self._on_start(self._chassis, **self._callback_args)

        motors = tuple((w.motor for w in self._chassis.wheels))
        prev_positions = None

        while not self._stopped:
            s_p = [(_m.state, _m.position) for _m in motors]

            # if both motors are holding their position, it means that they have reached the goal
            if all(('holding' in s for s, _ in s_p)):
                if self._on_complete:
                    self._on_complete(self._chassis, **self._callback_args)
                return

            # check if one of the motors is not stalled, by comparing the current positions
            # and the previous ones (if available)
            # TODO find why the speed cannot be used (always 0)
            if prev_positions and any((pp == sp[1] and 'holding' not in sp[0] for pp, sp in zip(prev_positions, s_p))):
                self._stalled = True
                if self._on_stalled:
                    self._on_stalled(self._chassis, **self._callback_args)
                return

            prev_positions = [p for _, p in s_p]
            time.sleep(0.1)


class NullMotionMonitor(object):
    """ A dummy monitor imitating real ones methods and used for handling special
    cases resulting in null motions.
    """
    def wait(self, *kwargs):
        return self

    def stop(self):
        pass

    @property
    def running(self):
        return False

    @property
    def stalled(self):
        return False
