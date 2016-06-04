#!/usr/bin/env python

from ctypes import *

libamino = cdll.LoadLibrary("libamino.so")

print libamino

class QuatStruct(Structure):
    _fields_ = [("x", c_double),
                ("y", c_double),
                ("z", c_double),
                ("w", c_double)]

class Quat:
    def __init__(self):
        self.data = QuatStruct(0,0,0,1)

    def __init__(self,x,y,z,w):
        self.data = QuatStruct(x,y,z,w)

    def __str__(self):
        return ("<Quat [" +
                str(self.data.x) + ", " +
                str(self.data.y) + ", " +
                str(self.data.z) + ", " +
                str(self.data.w) +
                "]>")

    def normalize(self):
        libamino.aa_tf_qnormalize(byref(self.data))
