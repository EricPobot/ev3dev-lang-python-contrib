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


import math
from collections import namedtuple

__author__ = 'Eric Pascual'


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
            destination (ev3dev.robotics.geometry.Point): the destination coordinates

        Returns:
            float: the distance to the point
        """
        dx, dy = destination.x - self.x, destination.y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to(self, destination):
        """ Return the bearing of given destination point, considering the pose heading.

        Args:
            destination (ev3dev.robotics.geometry.Point): the destination coordinates

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