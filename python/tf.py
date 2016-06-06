#!/usr/bin/env python

import libamino as aa
import ctypes

######################
## Principal Angles ##
######################

class PrincipalAngle:
    """A rotation about a principal axis"""
    __slots__ = ['angle']

    def __init__(self,angle):
        self.angle = angle

    def __mul__(self,other):
        return quat(self) * other

    def __rmul__(self,other):
        return other * quat(self)

class XAngle(PrincipalAngle):
    """A rotation about the X axis"""
    def _quat(self,other):
        """Convert `self' to the quaternion and store in `other'"""
        return aa.tf_xangle2quat(self.angle, other)

    def __repr__(self):
        return "XAngle({0})".format(self.angle)


class YAngle(PrincipalAngle):
    """A rotation about the Y axis"""
    def _quat(self, other):
        """Convert `self' to the quaternion and store in `other'"""
        return aa.tf_yangle2quat(self.angle, other)

    def __repr__(self):
        return "YAngle({0})".format(self.angle)

class ZAngle(PrincipalAngle):
    """A rotation about the Z axis"""
    def _quat(self, other):
        """Convert `self' to the quaternion and store in `other'"""
        return aa.tf_zangle2quat(self.angle, other )

    def __repr__(self):
        return "ZAngle({0})".format(self.angle)

###########
## Vector #
###########

class Vec(ctypes.Structure):
    """Vectors of length 3"""

    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

    @staticmethod
    def from_xyz(x,y,z):
        """Create a vector from x, y, and z components"""
        return Vec(x,y,z)

    @staticmethod
    def create():
        """Create a vector from x, y, and z components"""
        return Vec.from_xyz(0.0, 0.0, 0.0)

    @staticmethod
    def ensure(thing):
        """Ensure that `thing' is a vector"""
        if isinstance(thing,Vec):
            return thing
        else: return vec(thing)

    def __str__(self):
        return "({0}i + {1}j + {2}k)".format(self.x,
                                             self.y,
                                             self.z)

    def __repr__(self):
        return "Vec([{0}, {1}, {2}])".format(self.x,
                                             self.y,
                                             self.z )

    def _vec(self,other):
        other.x = self.x
        other.y = self.y
        other.z = self.z

    def cross(self, other):
        """Compute the cross product"""
        return aa.tf_cross(self, Vec.ensure(other), Vec.create())

    def dot(self, other):
        """Compute the dot product"""
        return aa.tf_vdot(self, Vec.ensure(other))


    def __add__(self, other):
        return Vec.from_xyz(self.x + other.x,
                            self.y + other.y,
                            self.z + other.z)

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        return Vec.from_xyz(self.x - other.x,
                            self.y - other.y,
                            self.z - other.z)

    def __rsub__(self, other):
        return Vec.from_xyz(other.x - self.x,
                            other.y - self.y,
                            other.z - self.z)


    def __neg__(self):
        return Vec.from_xyz( -self.x, -self.y, -self.z)

    def __pos__(self):
        return Vec.from_xyz( self.x, self.y, self.z)

    #def __abs__(self):
    #   return sqrt( self.x * self.x + s

def vec(other):
    """Create a vector from some other type"""
    if type(other) is list:
        return Vec.from_xyz(other[0],other[1],other[2])
    elif type(other) is tuple:
        return Vec.from_xyz(other[0],other[1],other[2])
    else:
        return other._vec(self,Vec())

def dot(a,b):
    """Compute the dot product"""
    return Vec.ensure(a).dot(b)

def cross(a,b):
    """Compute the cross product"""
    return Vec.ensure(a).cross(b)

##########################
## Ordinary Quaternions ##
##########################

class Quat(ctypes.Structure):
    """Ordinary Quaternions"""

    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double),
                ("w", ctypes.c_double)]

    @staticmethod
    def from_xyzw(x,y,z,w):
        """Create a quaternion from x, y, z, and w components"""
        return Quat(x,y,z,w)

    @staticmethod
    def create():
        """Create a new quaternion"""
        return Quat.from_xyzw(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def identity():
        """Create an identity quaternion"""
        return Quat.from_xyzw(0.0, 0.0, 0.0, 1.0)

    @staticmethod
    def ensure(thing):
        """Ensure that `thing' is a quaternion.

        If `thing' is a quaternion, return it.
        Otherwise, convert `thing' to a quaternion"""

        if isinstance(thing,Quat):
            return thing
        else:
            return quat(thing)

    def _quat(self,other):
        """Copy `self' into the quaternion `other'"""
        other.x = self.x
        other.y = self.y
        other.z = self.z
        other.w = self.w
        return other

    def __str__(self):
        return "({0} + {1}i + {2}j + {3}k)".format( self.w,
                                                    self.x,
                                                    self.y,
                                                    self.z)

    def __repr__(self):
        return "Quat([{0}, {1}, {2}, {3}])".format(self.x,
                                                   self.y,
                                                   self.z,
                                                   self.w)

    def normalize(self):
        """Return the normalized quaternion of `self'"""
        return aa.tf_qnormalize2(self, Quat.create())

    def exp(self):
        """Return the exponential quaternion of `self'"""
        return aa.tf_qexp(self, Quat.create())

    def ln(self):
        """Return the natural logarithm quaternion of `self'"""
        return aa.tf_qln(self, Quat.create())

    ## Operators ##

    def __mul__(self, other):
        return aa.tf_qmul( self, Quat.ensure(other), Quat.create() )

    def __rmul__(self, other):
        return aa.tf_qmul( Quat.ensure(other), self, Quat.create() )

    def __add__(self, other):
        return aa.tf_qadd( self, Quat.ensure(other), Quat.create() )

    def __radd__(self, other):
        return aa.tf_qadd( Quat.ensure(other), self, Quat.create() )

    def __sub__(self, other):
        return aa.tf_qsub( self, Quat.ensure(other), Quat.create() )

    def __rsub__(self, other):
        return aa.tf_qsub( Quat.ensure(other), self, Quat.create() )

    def __pow__(self, a):
        return aa.tf_qpow( self, a, Quat.create() )


    def __neg__(self):
        return Quat.from_xyzw( -self.x, -self.y, -self.z, -self.w )

    def __pos__(self):
        return Quat(self)

    def __invert__(self):
        return aa.tf_qinv(self, Quat.create())

    def __abs__(self):
        return aa.tf_qnorm(self)


    def __len__(self):
        return 4

    def __getitem__(self,i):
        if 0 == i: return self.x
        elif 1 == i: return self.y
        elif 2 == i: return self.z
        elif 3 == i: return self.w
        else: raise IndexError

    def __setitem__(self,i,value):
        if 0 == i:   self.x = value
        elif 1 == i: self.y = value
        elif 2 == i: self.z = value
        elif 3 == i: self.w = value
        else: raise IndexError

def quat(thing):
    """Construct a quaternion representing `thing'"""
    if type(thing) is list:
        return Quat.from_xyzw(thing[0],thing[1],thing[2],thing[3])
    elif type(thing) is tuple:
        return Quat.from_xyzw(thing[0],thing[1],thing[2],thing[3])
    elif type(thing) is float:
        return Quat.from_xyzw(0.0, 0.0, 0.0, thing)
    elif type(thing) is int:
        return Quat.from_xyzw(0.0, 0.0, 0.0, float(thing))
    elif type(thing) is complex:
        return Quat.from_xyzw(thing.imag, 0.0, 0.0, thing.real)
    else:
        return thing._quat(Quat())

######################
## Dual Quaternions ##
######################

class DuQu(ctypes.Structure):
    _fields_ = [("rx", ctypes.c_double),
                ("ry", ctypes.c_double),
                ("rz", ctypes.c_double),
                ("rw", ctypes.c_double),
                ("dx", ctypes.c_double),
                ("dy", ctypes.c_double),
                ("dz", ctypes.c_double),
                ("dw", ctypes.c_double)]

    def __str__(self):
        return "({0}+{1}i+{2}j+{3}k) +  ({4}+{5}i+{6}j+{7}k)e".format(
            self.rw, self.rx, self.ry, self.rz,
            self.dw, self.dx, self.dy, self.dz )

    def __repr__(self):
        return "DuQu({0}, {1}, {2}, {3},   {4}, {5}, {6}, {7})".format(
            self.rx, self.ry, self.rz, self.rw,
            self.dx, self.dy, self.dz, self.dw )

#############################
## Quaternions-Translation ##
#############################

# class QuTr:
#     __slots__ = ["quat", "vec"]
