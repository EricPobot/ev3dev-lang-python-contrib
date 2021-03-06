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


class OmniWheel(StandardWheel):
    """ A omni-wheel (aka omni-directional wheel or holonomic wheel).

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

        super(OmniWheel, self).__init__(
            motor=motor, diameter=diameter, gear_ratio=gear_ratio, offset=offset, invert=invert
        )
        self._angle = angle

    @property
    def angle(self):
        """ Read-access to the wheel axis angle to the robot main axis.

        :type: float
        """
        return self._angle


class Chassis(object):
    """ Root abstract class for all kinds of chassis.

    It defines the interface common for all chassis.
    """
    _travel_speed = 0
    _rotate_speed = 0

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

    def travel(self, distance, speed=None):
        """ Travels for the specified distance at a given speed.

        This method is asynchronous, which means that it returns immediately. It returns
        the monitor created for tracking the motion.

        In addition, callable can be passed, which will be called on various events
        (start of the move, end of the move, stalled detection).

        Args:
            distance (float): the distance to be traveled, in wheel diameter unit
            speed (float): optional local override of the default speed as set by :py:attr:`travel_speed`
        """
        raise NotImplementedError()

    def arc(self, radius, angle, speed=None):
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
        """
        raise NotImplementedError()

    def rotate(self, angle, speed=None):
        """ Rotates for the specified amount of degrees.

        Nothing else but the special case of a null radius arc.

        See :py:meth:`arc` for detailed documentation.
        """
        return self.arc(0, angle, speed)

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

    def stop(self, stop_option=None):
        """ Immediate stop, using the current motor stop setting.

        Args:
            stop_option (int): optional stop command, among `WheeledChassis.StopOption` values.
                If not provided, use the current motors setting
        """
        raise NotImplementedError()


class WheeledChassis(Chassis):
    """ Represents the chassis of a wheeled robot, whatever configuration is used.

    .. note:: Even if we use the term *wheel*, this chassis class can also be used
        with treads based chassis, as long as treads are compatible with the type of platform.
    """
    class StopOption(object):
        """ The various options for stopping the chassis motors
        """
        #: disconnect power
        COAST = 1
        #: brake by short circuiting the motor
        BRAKE = 2
        #: actively hold the position
        HOLD = 3

    _stop_commands = {
        StopOption.COAST: RegulatedMotor.STOP_COMMAND_COAST,
        StopOption.BRAKE: RegulatedMotor.STOP_COMMAND_BRAKE,
        StopOption.HOLD: RegulatedMotor.STOP_COMMAND_HOLD
    }

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

    Usually, such chassis are made of two driving wheels, plus one or more
    passive casters. Any number of driving wheels can be provided
    (at least two of course), the sign of their offset indicating the left
    or right side they are mounted.

    If using omni-wheels for building the robot (not a good idea since no
    lateral guidance), they will be considered as standard ones.
    """
    def _check_wheels(self, wheels):
        """
        Args:
            wheels (list[StandardWheel]): the wheels of the chassis
        """
        super(DifferentialWheeledChassis, self)._check_wheels(wheels)

        if len(wheels) < 2:
            raise ValueError('this chassis needs at least two wheels')
        if not all((isinstance(wheel, StandardWheel) for wheel in wheels)):
            raise ValueError('this chassis supports standard wheels only')

    def travel(self, distance, speed=None):
        """ Travels for the specified distance at a given speed.

        This method is asynchronous, which means that it returns immediately. To manage the
        movement, it is easier to use a :py:class:`.navigation.MovePilot` on top of the chassis.

        The robot will stop when the distance have been reached.

        Args:
            distance (float): the distance to be traveled, in wheel diameter unit
            speed (float): optional local override of the default speed as set by :py:attr:`travel_speed`

        Returns:
            MotionMonitor: the motion monitoring object
        """
        speed = speed or self._travel_speed

        if not speed:
            return

        for w in self._wheels:
            w.angular_position_set_point = distance / w.radius

            w.speed_regulation_enabled = True
            w.angular_speed_set_point = speed / w.radius

        self._wheels_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

    def arc(self, radius, angle, speed=None):
        """ Moves the robot along an arc with a specified radius and angle, after which the robot stops moving.

        If radius is positive, the robot turns left, the center of the circle being on its left side. If it
        is negative, the move takes place on the opposite side. If radius is null, the robot rotates in place.

        The sign of the angle gives the direction of the spin (CCW if positive, CW if negative). Hence the combined
        signs of radius and angle specify the direction of move (forward or backward) along the arc. If both are
        the same, the robot will move forward. If they are different it will move backwards.

        This method is asynchronous, which means that it returns immediately. To manage the
        movement, it is easier to use a :py:class:`.navigation.MovePilot` on top of the chassis.

        The robot will stop when its heading has changed by the provided angle.

        .. note::

            In case of rotation in place (null radius), the currently set angular speed is used.

            If provided, the sign of the speed will be ignored, since inferred by the ones of radius and angle.

        Args:
            radius (float): radius of the arc, 0 for a rotation in place
            angle (float): the heading change
            speed (float): the speed along the path, if different from default one
        """
        speed = abs(speed or self._travel_speed)

        if not speed:
            return

        for w in self._wheels:
            position = (radius + w.offset) * math.radians(angle) / w.radius
            w.angular_position_set_point = position

            w.speed_regulation_enabled = True
            w.angular_speed_set_point = position * speed / float(angle)

        self._wheels_sync_start(RegulatedMotor.COMMAND_RUN_TO_REL_POS)

    def stop(self, stop_option=None):
        """ Immediate stop.

        Args:
            stop_option (int): optional stop command, among `WheeledChassis.StopOption` values.
                If not provided, use the current motors setting
        """
        stop_command = self._stop_commands[stop_option] if stop_option else None
        for w in self._wheels:
            w.motor.stop(stop_option=stop_command)


class OmniWheeledChassis(DifferentialWheeledChassis):
    """ Models a omni wheels (aka holonomic wheels) chassis,
    and provides the access to properties and methods specific to this type of
    mechanical architecture.

    This kind of platform is also called *kiwi drive* or *Killough*.

    .. warning:: work in progress
    """
    def _check_wheels(self, wheels):
        """
        Args:
            wheels (list[OmniWheel]): the wheels of the chassis
        """
        super(DifferentialWheeledChassis, self)._check_wheels(wheels)

        if len(wheels) < 3:
            raise ValueError('this chassis needs at least three wheels')
        if not all((isinstance(wheel, OmniWheel) for wheel in wheels)):
            raise ValueError('this chassis needs omni-wheels')

    def travel_cartesian(self, motion, speed=None):
        """ Travels according to a given relative motion.

        The motion is provided as a 3 components vector:
        - X coordinate
        - Y coordinate
        - rotation angle

        If the speed is provided, it must use the same format.

        Args:
            motion (iterable[float]): the relative motion vector
            speed (iterable[float]): the speed vector
        """
        try:
            x_rel, y_rel, angle = motion
        except ValueError:
            raise ValueError('motion vector size must be 3')
        try:
            x_speed, y_speed, r_speed = speed
        except ValueError:
            raise ValueError('speed vector size must be 3')

        raise NotImplementedError()
