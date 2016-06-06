#!/usr/bin/env python

from ctypes import *

lib = cdll.LoadLibrary("libamino.so")

##########################
## Ordinary Quaternions ##
##########################

lib.aa_tf_qnorm.restype = c_double
def tf_qnorm(q):
    return lib.aa_tf_qnorm(byref(q))

def tf_qnormalize2(q,r):
    lib.aa_tf_qnormalize2(byref(q), byref(r))
    return r

def tf_qinv(q,r):
    lib.aa_tf_qinv(byref(q),byref(r))
    return r

def tf_qexp(q,r):
    lib.aa_tf_qexp(byref(q),byref(r))
    return r

def tf_qln(q,r):
    lib.aa_tf_qln(byref(q),byref(r))
    return r

def tf_qpow(q,a,r):
    lib.aa_tf_qpow(byref(q), c_double(a), byref(r))
    return r


def tf_qmul(a,b,c):
    lib.aa_tf_qmul(byref(a), byref(b), byref(c))
    return c

def tf_qadd(a,b,c):
    lib.aa_tf_qadd(byref(a), byref(b), byref(c))
    return c

def tf_qsub(a,b,c):
    lib.aa_tf_qsub(byref(a), byref(b), byref(c))
    return c

def tf_xangle2quat(angle,q):
    lib.aa_tf_xangle2quat(c_double(angle),byref(q))
    return q

def tf_yangle2quat(angle,q):
    lib.aa_tf_yangle2quat(c_double(angle),byref(q))
    return q

def tf_zangle2quat(angle,q):
    lib.aa_tf_zangle2quat(c_double(angle),byref(q))
    return q
