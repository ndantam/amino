#!/usr/bin/env python

from ctypes import *

lib = cdll.LoadLibrary("libamino.so")

##########################
## Ordinary Quaternions ##
##########################

class QuatStruct(Structure):
    _fields_ = [("x", c_double),
                ("y", c_double),
                ("z", c_double),
                ("w", c_double)]

    def copy(self):
        return QuatStruct(self.x, self.y, self.z, self.w)

    def quat(self):
        return self.copy()

def tf_qnormalize(q):
    lib.aa_tf_qnormalize(byref(q))
    return q

def tf_xangle2quat(angle,q):
    lib.aa_tf_xangle2quat(c_double(angle),byref(q))
    return q

def tf_yangle2quat(angle,q):
    lib.aa_tf_yangle2quat(c_double(angle),byref(q))
    return q

def tf_zangle2quat(angle,q):
    lib.aa_tf_zangle2quat(c_double(angle),byref(q))
    return q
