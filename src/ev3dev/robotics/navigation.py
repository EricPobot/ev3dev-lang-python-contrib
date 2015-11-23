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

.. _LeJOS: http://www.lejos.org/ev3/docs/
"""

import threading
import time
from collections import namedtuple

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

    It automatically publishes :py:class:`Move` instances while executing the moves.
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
            MoveMonitor: the motion monitoring object

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
            def arrived(chassis, move):
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
        return MoveMonitor(
            self._chassis,
            Move(Move.MOVE_STRAIGHT, distance=distance),
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
            MoveMonitor: the motion monitoring object
        """
        self._chassis.arc(radius=radius, angle=angle, speed=speed)
        return MoveMonitor(
            self._chassis,
            Move(Move.MOVE_ARC, radius=radius, angle=angle),
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
            MoveMonitor: the motion monitoring object
        """
        self._chassis.rotate(angle=angle, speed=speed)
        return MoveMonitor(
            self._chassis,
            Move(Move.MOVE_ROTATE, angle=angle),
            on_start=on_start, on_complete=on_complete, on_stalled=on_stalled, callback_args=callback_args
        )

    def rotate_left(self, angle, **kwargs):
        """ Convenience method for a rotation in place leftward (i.e. CCW)

        The sign of the provided angle will be ignored and forced to the right one.

        Args:
            angle (float): the heading change
            **kwargs: same as :py:meth:`rotate`

        Returns:
            MoveMonitor: the motion monitoring object
        """
        return self.rotate(angle=abs(angle), **kwargs)

    def rotate_right(self, angle, **kwargs):
        """ Convenience method for a rotation in place rightward (i.e. CW)

        The sign of the provided angle will be ignored and forced to the right one.

        Args:
            angle (float): the heading change
            **kwargs: same as :py:meth:`rotate`

        Returns:
            MoveMonitor: the motion monitoring object
        """
        return self.rotate(angle=-abs(angle), **kwargs)

    def stop(self, stop_option=None):
        """ Immediate stop.

        Args:
            stop_option (int): optional stop command, among `WheeledChassis.StopOption` values.
                If not provided, use the current motors setting
        """
        self._chassis.stop(stop_option)


class MoveMonitor(threading.Thread):
    """ An instance of this class is returned by pilot motion commands.

    It extends the standard :py:class:`threading.Thread` class by adding
    a couple of convenience methods and properties. The most common use case
    is calling its :py:meth:`wait` method when synchronous behaviours are
    wanted. In addition, it provides a couple or dedicated properties
    (e.g. :py:attr:`stalled`, :py:attr:`running`,...) for getting information
    about whit is going.
    """
    def __init__(self, chassis, move, on_start=None, on_complete=None, on_stalled=None, callback_args=None, **kwargs):
        """ All the callbacks receive the pilot as first argument, and can accept
        additional keyword parameters, which will contain the content
        of the `callback_args` dictionary passed here.

        Args:
            chassis (WheeledChassis): the associated pilot
            move (Move): the movement which is monitor
            on_start (callable): an optional callback invoked when starting the motion
            on_complete (callable): an optional callback invoked at the normal completion of the motion.
            on_stalled (callable): an optional callback invoked when a motor stalled situation is detected
            callback_args (dict): an optional dictionary defining the kwargs which will be passed to the callbacks.
            \**kwargs: transmitted to super
        """
        super(MoveMonitor, self).__init__(**kwargs)
        self._chassis = chassis
        self._move = move
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
            self._on_start(self._chassis, self._move, **self._callback_args)

        motors = tuple((w.motor for w in self._chassis.wheels))
        prev_positions = None

        while not self._stopped:
            s_p = [(_m.state, _m.position) for _m in motors]

            # if both motors are holding their position, it means that they have reached the goal
            if all(('holding' in s for s, _ in s_p)):
                if self._on_complete:
                    self._on_complete(self._chassis, self._move, **self._callback_args)
                return

            # check if one of the motors is not stalled, by comparing the current positions
            # and the previous ones (if available)
            # TODO find why the speed cannot be used (always 0)
            if prev_positions and any((pp == sp[1] and 'holding' not in sp[0] for pp, sp in zip(prev_positions, s_p))):
                self._stalled = True
                if self._on_stalled:
                    self._on_stalled(self._chassis, self._move, **self._callback_args)
                return

            prev_positions = [p for _, p in s_p]
            time.sleep(0.1)


class NullMoveMonitor(object):
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


class Move(namedtuple('Move', 'move_type distance radius angle')):
    """ Models an elementary move (straight, arc, rotate,...) for communication
    to the callbacks.

    Instances are immutable.
    """
    __slots__ = ()

    MOVE_STRAIGHT, MOVE_ARC, MOVE_ROTATE = range(1, 4)

    _str_fmt = {
        MOVE_STRAIGHT: "Move(STRAIGHT, dist=%(dist)f)",
        MOVE_ARC: "Move(ARC, radius=%(radius)f, angle=%(angle)f)",
        MOVE_ROTATE: "Move(ROTATE, angle=%(angle)f)",
    }

    def __new__(cls, move_type, distance=None, radius=None, angle=None):
        if move_type == cls.MOVE_STRAIGHT:
            radius = angle = None
            if distance is None:
                raise ValueError('missing distance property')
        elif move_type == cls.MOVE_ARC:
            distance = None
            if any(a is None for a in (radius, angle)):
                raise ValueError('missing radius or angle property')
        elif move_type == cls.MOVE_ROTATE:
            distance = radius = None
            if angle is None:
                raise ValueError('missing angle property')
        else:
            raise ValueError('invalid move type')

        return super(Move, cls).__new__(cls, move_type, distance, radius, angle)

    def __str__(self):
        return self._str_fmt[self.move_type] % {'dist': self.distance, 'radius': self.radius, 'angle': self.angle}
