#!/usr/bin/env python

import libamino as aa
import ctypes

######################
## Principal Angles ##
######################

class PrincipalAngle:
    __slots__ = ['angle']

    def __init__(self,angle):
        self.angle = angle

    def __mul__(self,other):
        return Quat(self) * Quat(other)

class XAngle(PrincipalAngle):
    def quat(self,other):
        return aa.tf_xangle2quat(self.angle, other)

    def __repr__(self):
        return "XAngle({0})".format(self.angle)


class YAngle(PrincipalAngle):
    def quat(self, other):
        return aa.tf_yangle2quat(self.angle, other)

    def __repr__(self):
        return "YAngle({0})".format(self.angle)

class ZAngle(PrincipalAngle):
    def quat(self, other):
        return aa.tf_zangle2quat(self.angle, other )

    def __repr__(self):
        return "ZAngle({0})".format(self.angle)

##########################
## Ordinary Quaternions ##
##########################

class Quat(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double),
                ("w", ctypes.c_double)]

    @staticmethod
    def from_xyzw(x,y,z,w):
        return Quat((x,y,z,w))

    @staticmethod
    def create():
        return Quat.from_xyzw(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def identity():
        return Quat.from_xyzw(0.0, 0.0, 0.0, 1.0)

    @staticmethod
    def ensure(thing):
        if isinstance(thing,Quat):
            return thing
        else:
            return Quat(thing)


    def __init__(self,thing):
        if type(thing) is list:
            super(Quat,self).__init__(thing[0],thing[1],thing[2],thing[3])
        elif type(thing) is tuple:
            super(Quat,self).__init__(thing[0],thing[1],thing[2],thing[3])
        elif type(thing) is float:
            super(Quat,self).__init__(0.0, 0.0, 0.0, thing)
        elif type(thing) is int:
            super(Quat,self).__init__(0.0, 0.0, 0.0, float(thing))
        elif type(thing) is complex:
            super(Quat,self).__init__(thing.imag, 0.0, 0.0, thing.real)
        else:
            thing.quat(self)

    def quat(self,other):
        other.x = self.x
        other.y = self.y
        other.z = self.z
        other.w = self.w

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
        return aa.tf_qnormalize2(self, Quat.create())

    def exp(self):
        return aa.tf_qexp(self, Quat.create())

    def ln(self):
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
