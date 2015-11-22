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

""" This modules provides the implementation of the concept of *pilots*, which are
high level entities intended to control the motion of a robot in a uniform way,
not depending on its mechanical architecture.

This encompasses the common differential architecture, but also the holonomic,
steering,...

This code is heavily based on LeJOS_ one, its documentation being
reproduced as is when relevant, so that LeJOS users can quickly be productive.

.. note::

    After I started porting the Java model, I noticed that the pilots are now
    deprecated, and have been replaced by the chassis. You'll find the Python
    equivalents to LeJOS chassis in the :py:mod:`.chassis` module.

.. _LeJOS: http://www.lejos.org/ev3/docs/
"""

import threading
import time
from collections import namedtuple
import math

from ev3dev.motors import RegulatedMotor

from .chassis import Chassis


class MovePilot(object):
    """ The Pilot class is a software abstraction of the Pilot mechanism
    of a robot. It contains methods to control robot movements: travel forward or
    backward in a straight line or a circular path or rotate to a new direction.

    This class will work with any chassis. Some types of chassis might not support all the
    movements this pilot support.

    An object of this class assumes that it has exclusive control of
    its motors. If any other object makes calls to its motors, the results are
    unpredictable.

    It automatically publishes :py:class:`Pose` instances while executing the moves.
    """
    def __init__(self, chassis):
        """
        Args:
            chassis (Chassis): the chassis controlled by the pilot
        """
        if not isinstance(chassis, Chassis):
            raise ValueError("missing or invalid chassis argument")

        self._chassis = chassis

    def travel(self, distance, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Travels for the specified distance at a given speed.

        This method is asynchronous, which means that it returns immediately. Its result
        is the monitor created for tracking the motion.

        Callable can be passed, which will be called on various events
        (start of the move, end of the move, stalled detection).

        Args:
            distance (float): the distance to be traveled, in wheel diameter unit
            speed (float): optional local override of the default speed as set by :py:attr:`travel_speed`
            on_start (callable): optional callback for motion start event handling
            on_complete (callable): optional callback for completion event handling
            on_stalled (callable): optional callback for stalled detection handling
            callback_args (dict): optional dictionary defining the kwargs passed to the callbacks

        Returns:
            MotionMonitor: the motion monitoring object

        Example:

        .. code-block:: python

            from ev3dev.ev3 import LargeMotor
            from ev3dev.robotics.chassis import StandardWheel, DifferentialWheeledChassis
            from ev3dev.robotics.navigation import MovePilot

            wheel_left = StandardWheel(LargeMotor('out_B'), 43.2, -75)
            wheel_right = StandardWheel(LargeMotor('out_C'), 43.2, 75)
            chassis = DifferentialWheeledChassis((wheel_left, wheel_right))
            pilot = MovePilot(chassis)

            # synchronous usage with a forever wait
            pilot.travel(250).wait()

            # asynchronous usage
            def arrived(chassis):
                print('just arrived')

            mvt = pilot.travel(distance=250, speed=100, on_complete=arrived)
            # ... do something while traveling

            # wait at most 10 secs for arrival at destination before doing something else
            mvt.wait(delay=10)
            if mvt.running:
                print("something wrong happened while traveling")
            else:
                print("we are at destination")
        """
        self._chassis.travel(distance=distance, speed=speed)
        return MotionMonitor(
            self._chassis,
            on_start=on_start, on_complete=on_complete, on_stalled=on_stalled, callback_args=callback_args
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
            on_start (callable): optional callback for motion start event handling
            on_complete (callable): optional callback for completion event handling
            on_stalled (callable): optional callback for stalled detection handling
            callback_args (dict): optional dictionary defining the kwargs passed to the callbacks

        Returns:
            MotionMonitor: the motion monitoring object
        """
        self._chassis.arc(radius=radius, angle=angle, speed=speed)
        return MotionMonitor(
            self._chassis,
            on_start=on_start, on_complete=on_complete, on_stalled=on_stalled, callback_args=callback_args
        )

    def rotate(self, angle, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Rotates in place.

        This is a special case of the :py:meth:`arc` method, with a null radius.

        Args:
            angle (float): the heading change
            speed (float): the speed along the path, if different from default one
            on_start (callable): optional callback for motion start event handling
            on_complete (callable): optional callback for completion event handling
            on_stalled (callable): optional callback for stalled detection handling
            callback_args (dict): optional dictionary defining the kwargs passed to the callbacks

        Returns:
            MotionMonitor: the motion monitoring object
        """
        self._chassis.rotate(angle=angle, speed=speed)
        return MotionMonitor(
            self._chassis,
            on_start=on_start, on_complete=on_complete, on_stalled=on_stalled, callback_args=callback_args
        )

    def rotate_left(self, angle, **kwargs):
        """ Convenience method for a rotation in place leftward (i.e. CCW)

        The sign of the provided angle will be ignored and forced to the right one.

        Args:
            angle (float): the heading change
            **kwargs: same as :py:meth:`rotate`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        return self.rotate(angle=abs(angle), **kwargs)

    def rotate_right(self, angle, **kwargs):
        """ Convenience method for a rotation in place rightward (i.e. CW)

        The sign of the provided angle will be ignored and forced to the right one.

        Args:
            angle (float): the heading change
            **kwargs: same as :py:meth:`rotate`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        return self.rotate(angle=-abs(angle), **kwargs)

    def stop(self, stop_option=None):
        """ Immediate stop.

        Args:
            stop_option (int): optional stop command, among `WheeledChassis.StopOption` values.
                If not provided, use the current motors setting
        """
        self._chassis.stop(stop_option)


class DifferentialPilot(object):
    """ The DifferentialPilot class is a software abstraction of the Pilot mechanism of a robot.
    It contains methods to control robot movements: travel forward or backward in a straight line
    or a circular path or rotate to a new direction.

    This class will only work with two independently controlled motors to steer differentially,
    so it can rotate within its own footprint (i.e. turn on one spot).

    In all the methods that cause the robot to change its heading (the angle relative
    to the X axis in which the robot is facing) the angle parameter specifies the change
    in heading. A positive angle causes a turn to the left (anti-clockwise) to increase the heading,
    and a negative angle causes a turn to the right (clockwise).

    .. deprecated:: 0.2.0
    """
    _travel_speed = 0
    _rotate_speed = 0

    def __init__(self, wheel_diameter, track_width, left_motor, right_motor, reverse=False, motors_settings=None):
        """
        Args:
            wheel_diameter (float): the diameter of the wheels
            track_width (float): the distance between wheel contact points on the ground
            left_motor (RegulatedMotor): the left motor
            right_motor (RegulatedMotor): the right motor
            reverse (bool): if true, the NXT robot moves forward when the motors are running backward
            motors_settings (dict): optional dictionary with motor settings

        Raise:
            ValueError: if any of the mandatory parameter is missing

        .. note::

            The dimensions units (wheel diameter, track with) can be anything (mm or cm) as long
            as all the properties use the same one.
        """
        if not all((wheel_diameter, track_width, left_motor, right_motor)):
            raise ValueError('all arguments are mandatory')

        self._wheel_diam = wheel_diameter
        self._track_width = track_width
        self._left_motor, self._right_motor = self._motors = (left_motor, right_motor)
        self._reverse = reverse

        self._dist_per_pulse = self._wheel_diam * math.pi / self._left_motor.count_per_rot
        self._rotation_per_pulse = math.degrees(self._dist_per_pulse / self._track_width * 2)

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

    def reset_motors(self):
        """ Resets the motors
        """
        for m in self._motors:
            m.reset()

    def setup_motors(self, **kwargs):
        """ Configure the motors.

        Args:
            \**kwargs: the keyword arguments providing the desired attribute settings
        """
        for attr, value in kwargs.iteritems():
            for m in self._motors:
                setattr(m, attr, value)

    def _pulses_per_sec_linear(self, speed):
        """ Returns the number of pulses per second corresponding to the given linear speed.

        Args:
            speed (float): the speed in wheel diameter units per second

        Returns:
            int: the equivalent pulses per second
        """
        return round(speed / self._dist_per_pulse)

    def _pulses_per_sec_angular(self, speed):
        """ Returns the number of pulses per second corresponding to the given rotational peed.

        Args:
            speed (float): the speed in degrees per second

        Returns:
            int: the equivalent pulses per second
        """
        return round(speed / self._rotation_per_pulse)

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

    def drive(self, speed=None):
        """ Travels straight, at the speed previously set using :py:attr:`travel_speed`
        or locally overridden by the `speed` argument.

        Moves forward if speed is positive, backwards otherwise.

        Args:
            speed (float): optional speed
        """
        pulses_per_sec = self._pulses_per_sec_linear(speed or self._travel_speed)
        for m in self._motors:
            m.speed_regulation_enabled = m.SPEED_REGULATION_ON
            m.speed_sp = pulses_per_sec

        self._motors_sync_start(command=m.COMMAND_RUN_FOREVER)

    def forward(self, speed=None):
        """ Travels forward, no matter is the speed sign.
        """
        self.drive(abs(speed or self._travel_speed))

    def backward(self, speed=None):
        """ Travels backward, no matter is the speed sign.
        """
        self.drive(-abs(speed or self._travel_speed))

    def stop(self, stop_command=None):
        """ Immediate stop, using the current motor stop setting.

        Args:
            stop_command (str): optional stop command, among `RegulatedMotor.STOP_COMMAND_xxx`.
                If not provided, use the current motors setting
        """
        for m in self._motors:
            m.stop(stop_command=stop_command)

    def _motors_sync_start(self, command):
        """ Internal method used to start the motors in sync

        Args:
            command (str): the start command to be used (one of `COMMAND_RUN_xxx`)
        """
        for m in self._motors:
            m.command = command

    def travel(self, distance, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Travels for the specified distance at a given speed.

        This method is asynchronous, which means that it returns immediately. It returns
        the monitor created for tracking the motion.

        In addition, runnables can be passed, which will be called on various events
        (start of the move, end of the move, stalled detection).

        Example::

            >>> pilot = DifferentialPilot(43.2, 150, m_B, m_C)
            >>>
            >>> # synchronous usage with a forever wait
            >>> pilot.travel(250).wait(delay=None)
            >>>
            >>> # asynchronous usage
            >>> def arrived(pilot):
            >>>     print('just arrived')
            >>>
            >>> mvt = pilot.travel(distance=250, speed=100, on_complete=arrived)
            >>> # ... do something while traveling
            >>>
            >>> # wait at most 10 secs for arrival at destination before doing something else
            >>> mvt.wait(delay=10)
            >>> if mvt.not_done:
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
        pulses = round(distance / self._dist_per_pulse)
        pulses_per_sec = self._pulses_per_sec_linear(speed or self._travel_speed)

        for m in self._motors:
            m.position_sp = pulses
            m.speed_regulation_enabled = RegulatedMotor.SPEED_REGULATION_ON
            m.speed_sp = pulses_per_sec

        self._motors_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

        monitor = MotionMonitor(self,
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )
        monitor.start()

        return monitor

    def rotate(self, angle, speed=None, on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Rotates for the specified amount of degrees.

        Positive angles are counted CCW, negative ones CW. Callbacks mechanism is the same as for :py:meth:`travel`.

        Args:
            angle (float): the number of degrees to rotate
            speed (float): the rotation speed, in degrees per second (absolute value considered)
            on_start: see :py:meth:`travel`
            on_complete: see :py:meth:`travel`
            on_stalled: see :py:meth:`travel`
            callback_args: see :py:meth:`travel`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        pulses_per_sec = self._pulses_per_sec_angular(speed or self._rotate_speed)

        for m in self._motors:
            m.speed_regulation_enabled = m.SPEED_REGULATION_ON
            m.speed_sp = pulses_per_sec

        pulses = round(angle / self._rotation_per_pulse)
        self._left_motor.position_sp = -pulses
        self._right_motor.position_sp = pulses

        self._motors_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

        monitor = MotionMonitor(self,
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )
        monitor.start()

        return monitor

    def rotate_forever(self, speed):
        """ Rotates in place forever.

        Args:
            speed (float): the rotation speed in degrees per second. Positive speed turns CCW, negative CW.
        """
        pulses_per_sec = self._pulses_per_sec_angular(speed)

        for m, direction in zip(self._motors, (-1, +1)):
            m.speed_regulation_enabled = m.SPEED_REGULATION_ON
            m.speed_sp = pulses_per_sec * direction

        self._motors_sync_start(RegulatedMotor.COMMAND_RUN_FOREVER)

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
        if radius == 0:
            return self.rotate(
                angle=angle,
                on_start=on_start,
                on_complete=on_complete,
                on_stalled=on_stalled,
                callback_args=callback_args
            )

        speed = abs(speed or self._travel_speed)
        tw_2 = float(self._track_width) / 2

        sides = (-1, 1)

        bias = tw_2 / radius
        speeds = (speed * (1 + side * bias) for side in sides)

        rd = math.radians(angle)
        distances = ((radius + side * tw_2) * rd for side in sides)

        for m, s, d in zip(self._motors, speeds, distances):
            m.speed_regulation_enabled = m.SPEED_REGULATION_ON
            m.speed_sp = self._pulses_per_sec_linear(s)
            m.position_sp = d / self._dist_per_pulse

        self._motors_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

        monitor = MotionMonitor(self,
                                on_start=on_start,
                                on_complete=on_complete,
                                on_stalled=on_stalled,
                                callback_args=callback_args
                                )
        monitor.start()

        return monitor

    def travel_arc(self, radius, distance, speed=None,
                   on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Similar to :py:meth:`arc` but specifying the distance along the arc instead of the turn angle.
        """
        if radius == 0:
            raise ValueError('invalid radius for travel_arc')

        angle = float(distance) / radius
        return self.arc(
            radius=radius,
            angle=angle,
            speed=speed,
            on_start=on_start,
            on_complete=on_complete,
            on_stalled=on_stalled,
            callback_args=callback_args
        )

    def steer(self, turn_rate, speed=None):
        """ Starts the robot moving forward along a curved path. This method is similar to the :py:meth:`arc`
        except it uses the `turn_rate` parameter do determine the curvature of the path and therefore has
        the ability to drive straight.

        `turn_rate` specifies the sharpness of the turn. Use values between -200 and +200.
        A positive (resp. negative) value means that the center of the turn is on the left (resp. right).

        This parameter determines the ratio of inner wheel speed to outer wheel speed as a percent.
        The formula used is : `ratio = 100 - abs(turn_rate)`. When `turn_rate` absolute value is greater than
        200, the ratio becomes negative, which means that the wheels will turn in opposite directions.
        The extreme values (-200 and +200) result in the robot turning in place (i.e. spinning on itself).

        Args:
            turn_rate (float): path turn rate
            speed (float): optional travel speed, used to override locally the current value
                of :py:attr:`travel_speed`. If positive (resp. negative), the robot moves forward (resp. backward)
        """
        if not turn_rate:
            self.drive(speed)

        else:
            # clip parameter in [-200, +200] range
            turn_rate = min(max(turn_rate, -200), 200)

            if abs(turn_rate) == 200:
                self.rotate_forever(speed)

            else:
                ratio = float(100 - abs(turn_rate))
                if turn_rate > 0:
                    speeds = speed * ratio, speed   # left turn => right wheel is the fastest and runs at 'speed'
                else:
                    speeds = speed, speed * ratio

                for m, s in zip(self._motors, speeds):
                    m.speed_regulation_enabled = m.SPEED_REGULATION_ON
                    m.speed_sp = self._pulses_per_sec_linear(s)

                self._motors_sync_start(RegulatedMotor.COMMAND_RUN_FOREVER)

    def steer_angle(self, turn_rate, angle, speed=None,
                    on_start=None, on_complete=None, on_stalled=None, callback_args=None):
        """ Same as :py:meth:`steer`, but ends the move after the robot has turned a given angle.

        Since the steering direction is defined by the turn rate, the sign of the angle is
        ignored.

        Args:
            turn_rate (float): see :py:meth:`steer`
            angle (float): steer angle in degrees
            speed (float): see :py:meth:`steer`
            on_start, on_complete, on_stalled, callback_args: see :py:meth:`steer`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        # TODO compute the radius based on the turn rate and fall back to :py:meth:`arc`


class MotionMonitor(threading.Thread):
    """ An instance of this class is returned by pilot motion commands.

    It extends the standard :py:class:`threading.Thread` class by adding
    a couple of convenience methods and properties. The most common use case
    is calling its :py:meth:`wait` method when synchronous behaviours are
    wanted. In addition, it provides a couple or dedicated properties
    (e.g. :py:attr:`stalled`, :py:attr:`running`,...) for getting information
    about whit is going.
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

        self.start()

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


class Point(namedtuple('Point', 'x y')):
    """ Model of a 2D point with convenience functions.

    Examples::

        >>> # using default tolerance (3)
        >>> Point(0, 0.0001) == Point(0, 0)
        True
        >>> Point(0, 0.01) == Point(0, 0)
        False
        >>> Point(0, 0.0007) == Point(0, 0)
        False
        >>> Point(1, 2) + Point(3, 4) == Point(4, 6)
        True
        >>> Point(3, 4) - Point(1, 2) == Point(2, 2)
        True
        >>> p = Point(1, 2)
        >>> p += Point(1, 1)
        >>> p == Point(2, 3)
        True
        >>> Point(2, 2) >= Point(1, 1)
        True
        >>> Point(1, 1) == Point(1, 1)
        True
    """
    __slot__ = ()

    #: the round factor used to nullify epsilons
    tolerance = 3

    def __new__(cls, x=0.0, y=0.0):
        return super(Point, cls).__new__(cls, x, y)

    def __eq__(self, other):
        return all(round(abs(a - b), self.tolerance) == 0 for a, b in zip(self, other))

    def __add__(self, other):
        return Point(*(a + b for a, b in zip(self, other)))

    def __sub__(self, other):
        return Point(*(a - b for a, b in zip(self, other)))

    def __gt__(self, other):
        return self.x * self.x + self.y * self.y > other.x * other.x + other.y * other.y


class Pose(namedtuple('Pose', 'x y heading')):
    """ A pose is specified by its 2D coordinates of a heading, given in radians.

    The addition and subtraction operators are defined for poses, as term by term
    arithmetic operations.

    The heading is normalized by getting it back in the range [-pi, +pi]

    .. warning:: Poses are immutable objects.
    """
    __slot__ = ()
    _2_pi = math.pi * 2

    #: the round factor used to nullify epsilons
    tolerance = 3

    def __new__(cls, x=0.0, y=0.0, heading=0.0):
        return super(Pose, cls).__new__(cls, x, y, cls.norm_heading(heading))

    @staticmethod
    def norm_heading(h):
        """ Returned the normalized heading, by getting in back between
        -pi and +pi.

        Args:
            h (float): the heading in radians

        Returns:
            float: the normalized heading (in radians too)
        """
        return ((h + math.pi) % Pose._2_pi) - math.pi

    def distance_to(self, destination):
        """ Return the distance to a given destination point.

        Args:
            destination (Point): the destination coordinates

        Returns:
            float: the distance to the point
        """
        dx, dy = destination.x - self.x, destination.y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to(self, destination):
        """ Return the bearing of given destination point, considering the pose heading.

        Args:
            destination (Point): the destination coordinates

        Returns:
            float: the bearing with respect to pose heading
        """
        dx, dy = destination.x - self.x, destination.y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        return math.copysign(math.acos(dy / dist), math.asin(-dx / dist))

    @property
    def location(self):
        """ The location of the pose, returned as a point.

        :type: Point
        """
        return Point(self.x, self.y)

    @property
    def coordinates(self):
        """ The coordinates of the pose, returned as a tuple.

        :type: tuple
        """
        return self.x, self.y

    def __add__(self, other):
        return Pose(self.x + other.x, self.y + other.y, self.heading + other.heading)

    def __sub__(self, other):
        return Pose(self.x - other.x, self.y - other.y, self.heading - other.heading)

    def translated(self, dx, dy):
        """ Returned the translated pose.

        Args:
            dx (float): X variation
            dy (float): Y variation

        Returns:
            Pose: the result of the pose translation
        """
        return Pose(self.x + dx, self.y + dy, self.heading)

    def rotated(self, angle):
        """ Returned the rotated pose.

        Args:
            angle (float): rotation angle, in radians

        Returns:
            Pose: the result of the pose rotation
        """
        return Pose(self.x, self.y, self.heading + angle)

    def circle_center(self, radius):
        """ Finds the center of a circle tangent to the pose heading, passing at
        the pose location and with the given radius.

        Args:
            radius (float): the circle radius

        Returns:
            Point: the circle center

        Examples::

            >>> p = Pose(heading=math.radians(0))
            >>> p.circle_center(1) == Point(0, 1)
            True
            >>> p.circle_center(0) == p.location
            True
            >>> p = Pose(heading=math.radians(45))
            >>> p.circle_center(1) == Point(-round(math.sqrt(2) / 2, Pose.tolerance), round(math.sqrt(2) / 2, Pose.tolerance))
            True
        """
        return Point(
            round(-radius * math.sin(self.heading), self.tolerance),
            round(radius * math.cos(self.heading), self.tolerance)
        )

    def arc_path(self, destination):
        """ Returns the arc which is tangent to the pose heading,
        starts at the pose location and ends a the destination point.

        The arc is returned as its center and its radius.

        In the special case where the destination aligned with the current pose,
        returns None since the arc radius would be infinite. Same if the destination
        is the current location.

        Args:
            destination (Point): the arc destination

        Returns:
            tuple[Point, float]: the center of the arc and its angle

        Examples::

            >>> p = Pose(heading=math.radians(90))
            >>> p.arc_path(Point(0, 1)) == None
            True
            >>> p.arc_path(Point(0, -1)) == None
            True
            >>> p.arc_path(Point(1, 0))
            (Point(x=0.5, y=-0.0), -3.142)
            >>> c, a = p.arc_path(Point(1, 1))
            >>> c == Point(1, 0)
            True
            >>> a == -round(math.pi / 2, 2)
            True
        """
        # eliminate the special case where we are at destination already
        if destination == self.location:
            return None

        # copied from bearing_to to avoid multiple computations
        # of distances and coordinate deltas
        dx, dy = destination.x - self.x, destination.y - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        alpha = round(math.copysign(math.acos(dy / dist), math.asin(-dx / dist)), self.tolerance)

        # special cases :
        # - alpha == 0 (destination is in front of us)
        # - alpha == pi (destination is behind us)
        # => the radius is infinite and so is the center
        # return None to indicate it
        if abs(alpha) in (0.0, round(math.pi, self.tolerance)):
            return None
        else:
            radius = dist / 2 / math.sin(alpha)
            # the arc angle is twice the angle between the chord and the tangent
            return self.circle_center(radius), 2 * alpha
