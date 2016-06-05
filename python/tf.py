#!/usr/bin/env python

import libamino as aa

######################
## Principal Angles ##
######################

class PrincipalAngle:
    __slots__ = ['angle']

    def __init__(self,angle):
        self.angle = angle

class XAngle(PrincipalAngle):
    def quat(self):
        return aa.tf_xangle2quat(self.angle, aa.QuatStruct())

    def __str__(self):
        return "<YAngle " + str(self.angle) + ">"

class YAngle(PrincipalAngle):
    def quat(self):
        return aa.tf_yangle2quat(self.angle, aa.QuatStruct())

    def __str__(self):
        return "<YAngle " + str(self.angle) + ">"

class ZAngle(PrincipalAngle):
    def quat(self):
        return aa.tf_zangle2quat(self.angle, aa.QuatStruct() )

    def __str__(self):
        return "<YAngle " + str(self.angle) + ">"

##########################
## Ordinary Quaternions ##
##########################

class Quat:
    __slots__ = ['data']

    @staticmethod
    def from_xyzw(x,y,z,w):
        return Quat(aa.QuatStruct(x,y,z,w))

    # Conversion Constructor
    def __init__(self, thing):
        self.data = thing.quat()

    def quat(self):
        return self.data

    def __str__(self):
        return ("<Quat [" +
                str(self.data.x) + ", " +
                str(self.data.y) + ", " +
                str(self.data.z) + ", " +
                str(self.data.w) +
                "]>")

    def normalize(self):
        return Quat(aa.tf_qnormalize(self.data.copy()))
