# define Epsilon as a small number for floating point operations accuracy
import math
from utils import bound, to_rad, to_deg, EPS


# a class representation of a 2D Vector with x and y coordinates with some useful methods
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # returns the length of the vector
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    # returns the angle of the vector in radians
    def angle_rad(self):
        return math.atan2(self.y, self.x)

    # returns the angle of the vector in degrees
    def angle_deg(self):
        return to_deg(self.angle_rad())

    # returns the safe normalized vector
    def normalize(self):
        length = self.length()
        if length < EPS:
            return Vector(0, 0)
        return Vector(self.x / length, self.y / length)

    # returns the dot product of two vectors
    def dot(self, other):
        if not isinstance(other, Vector):
            return None
        return self.x * other.x + self.y * other.y

    # returns the distance between two vectors
    def distance(self, other):
        if not isinstance(other, Vector):
            return None
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    # cross product of two vectors
    def cross(self, other):
        if not isinstance(other, Vector):
            return None
        return self.x * other.y - self.y * other.x

    def angle_between(self, other):
        if not isinstance(other, Vector):
            return None
        l1 = self.length()
        l2 = other.length()
        if l1 < EPS:
            return self.angle_rad()
        elif l2 < EPS:
            return other.angle_rad()

        return math.acos(bound(self.dot(other) / (l1 * l2), -1, 1))

    # sign angle between two vectors
    def sign_angle(self, other):
        if not isinstance(other, Vector):
            return None
        return math.copysign(1, self.cross(other))

    # returns angle to the given vector from the current vector
    def angle_to(self, other):
        if not isinstance(other, Vector):
            return None
        return math.copysign(self.angle_between(other), self.cross(other))

    # returns angle to the given vector from the current vector in degrees
    def angle_to_deg(self, other):
        if not isinstance(other, Vector):
            return None
        return to_deg(self.angle_to(other))

    # create vector from angle and size
    @staticmethod
    def from_angle_size(angle, size):
        return Vector(size * math.cos(angle), size * math.sin(angle))

    # rotates the vector by the given angle in radians
    def rotate_rad(self, angle):
        cos = math.cos(angle)
        sin = math.sin(angle)
        return Vector(self.x * cos - self.y * sin, self.x * sin + self.y * cos)

    # rotates the vector by the given angle in degrees
    def rotate_deg(self, angle):
        return self.rotate_rad(to_rad(angle))

    # interpolates between two vectors
    def interpolate(self, other, t):
        if not isinstance(other, Vector):
            return None
        return Vector(self.x + (other.x - self.x) * t, self.y + (other.y - self.y) * t)

    # returns the prependicular vector from the given vector
    def perpendicular(self):
        return Vector(-self.y, self.x)

    # overloads the + operator to subtract two vectors
    def __add__(self, other):
        if not isinstance(other, Vector):
            return None
        return Vector(self.x + other.x, self.y + other.y)

    # overloads the - operator to subtract two vectors
    def __sub__(self, other):
        if not isinstance(other, Vector):
            return None
        return Vector(self.x - other.x, self.y - other.y)

    # overloads the * operator to scale a vector
    def __mul__(self, other):
        if not isinstance(other, float):
            return None
        return Vector(self.x * other, self.y * other)

    # overloads the / operator to scale a vector
    def __truediv__(self, other):
        return self * (1 / other)

    # overloads the equal operator
    def __eq__(self, v):
        if not isinstance(v, Vector):
            return False
        if abs(self.x - v.y) > EPS:
            return False
        if abs(self.x - v.y) > EPS:
            return False
        return True

    # overloads the not equal operator
    def __ne__(self, v):
        return not (self == v)

    # overloads the hash function to make the vector hashable
    def __hash__(self):
        return hash(self.x) ^ hash(self.y)

    # overloads the length function to return the length of the vector
    def __len__(self):
        return self.length()

    # overloads the getitem function to return the x or y coordinate of the vector
    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        else:
            return None

    # overloads the iter function to iterate over the vector
    def __iter__(self):
        yield self.x
        yield self.y

    # overloads the str() function to print the vector
    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

    # overloads the repr() function to print the vector
    __repr__ = __str__

    # cast list to vector
    @staticmethod
    def from_list(list):
        return Vector(list[0], list[1])

    # cast tuple to vector
    @staticmethod
    def from_tuple(t):
        return Vector(t[0], t[1])

    # cast string to vector
    @staticmethod
    def from_string(s):
        s = s.strip()
        s = s.strip("()")
        s = s.split(",")
        return Vector(float(s[0]), float(s[1]))

    # cast vector to list
    def to_list(self):
        return [self.x, self.y]

    # cast vector to tuple
    def to_tuple(self):
        return (self.x, self.y)
