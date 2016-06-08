#!/usr/bin/env python

from ctypes import *

lib = cdll.LoadLibrary("libamino.so")

##########
## Util ##
##########

def fcpy(dst,src,cnt):
    memmove( byref(dst),
             byref(src),
             c_size_t(cnt*sizeof(c_double)) )
    return dst

def fzero(dst,cnt):
    memset( byref(dst), c_int(0),
            c_size_t(cnt*sizeof(c_double)) )
    return dst



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

def tf_rotmat2quat(R,q):
    lib.aa_tf_rotmat2quat(byref(R),byref(q))
    return q

def tf_eulerzyx2quat(z,y,x,q):
    lib.aa_tf_eulerzyx2quat(c_double(z),
                            c_double(y),
                            c_double(x),
                            byref(q))
    return q

def tf_quat2axang(q,a):
    lib.aa_tf_quat2axang(byref(q),byref(a))
    return a

def tf_quat2rotvec(q,a):
    lib.aa_tf_quat2rotvec(byref(q),byref(a))
    return a

lib.aa_tf_vdot.restype = c_double
def tf_vdot(a,b):
    return lib.aa_tf_vdot(byref(a), byref(b))

def tf_cross(a,b,c):
    lib.aa_tf_cross(byref(a), byref(b), byref(c))
    return c

################
## Axis Angle ##
################
def tf_axang2quat(a,q):
    lib.aa_tf_axang2quat(byref(a),byref(q))
    return q

def tf_axang2rotmat(a,r):
    lib.aa_tf_axang2rotmat(byref(a),byref(r))
    return r

def tf_axang2rotvec(a,r):
    lib.aa_tf_axang2rotvec(byref(a),byref(r))
    return r

#####################
## Rotation Vector ##
#####################
def tf_rotvec2quat(a,q):
    lib.aa_tf_rotvec2quat(byref(a),byref(q))
    return q

def tf_rotvec2rotmat(a,r):
    lib.aa_tf_rotvec2rotmat(byref(a),byref(r))
    return r

def tf_rotvec2axang(a,r):
    lib.aa_tf_rotvec2axang(byref(a),byref(r))
    return r

#####################
## Rotation Matrix ##
#####################

def tf_xangle2rotmat(angle,R):
    lib.aa_tf_xangle2rotmat(c_double(angle),byref(R))
    return R

def tf_yangle2rotmat(angle,R):
    lib.aa_tf_yangle2rotmat(c_double(angle),byref(R))
    return R

def tf_zangle2rotmat(angle,R):
    lib.aa_tf_zangle2rotmat(c_double(angle),byref(R))
    return R

def tf_quat2rotmat(q,R):
    lib.aa_tf_quat2rotmat(byref(q),byref(R))
    return R

def tf_eulerzyx2rotmat(z,y,x,R):
    lib.aa_tf_eulerzyx2rotmat(c_double(z),
                              c_double(y),
                              c_double(x),
                              byref(R))
    return R

def tf_rotmat2axang(q,r):
    lib.aa_tf_rotmat2axang(byref(q),byref(r))
    return r

def tf_rotmat_mul(a,b,c):
    lib.aa_tf_rotmat_mul(byref(a), byref(b), byref(c))
    return c

###########################
## Transformation Matrix ##
###########################

def tf_duqu2tfmat(s,t):
    lib.aa_tf_duqu2tfmat(byref(s),byref(t))
    return t

def tf_tfmat_mul(a,b,c):
    lib.aa_tf_tfmat_mul(byref(a),byref(b),byref(c))
    return c


def tf_qv2tfmat(q,v,t):
    lib.aa_tf_qv2tfmat(byref(q),byref(v),byref(t))
    return t


def tf_tfmat2qv(t,q,v):
    lib.aa_tf_tfmat2qv(byref(t),byref(q),byref(v))

##################
## Euler Angles ##
##################

def tf_rotmat2eulerzyx(R,e):
    lib.aa_tf_rotmat2eulerzyx(byref(R), byref(e))
    return e

def tf_quat2eulerzyx(q,e):
    lib.aa_tf_quat2eulerzyx(byref(q), byref(e))
    return e

#####################
## Dual Quaternion ##
#####################

def tf_qv2duqu(q,v,s):
    lib.aa_tf_qv2duqu(byref(q),byref(v),byref(s))
    return s

def tf_duqu_trans(s,v):
    lib.aa_tf_duqu_trans(byref(s),byref(v))
    return v

def tf_duqu_ln(s,r):
    lib.aa_tf_duqu_ln(byref(s),byref(r))
    return r

def tf_duqu_exp(s,r):
    lib.aa_tf_duqu_exp(byref(s),byref(r))
    return r

def tf_duqu_conj(s,r):
    lib.aa_tf_duqu_conj(byref(s),byref(r))
    return r

def tf_duqu_mul(a,b,c):
    lib.aa_tf_duqu_mul(byref(a),byref(b),byref(c))
    return c

def tf_duqu_add(a,b,c):
    lib.aa_tf_duqu_add(byref(a),byref(b),byref(c))
    return c

def tf_duqu_sub(a,b,c):
    lib.aa_tf_duqu_sub(byref(a),byref(b),byref(c))
    return c

def tf_tfmat2duqu(s,t):
    lib.aa_tf_tfmat2duqu(byref(s),byref(t))
    return t


############################
## Quaternion-Translation ##
############################

def tf_qv_chain(aq,av,bq,bv,cq,cv):
    lib.aa_tf_qv_chain( byref(aq), byref(av),
                        byref(bq), byref(bv),
                        byref(cq), byref(cv))

def tf_duqu2qv(s,q,v):
    lib.aa_tf_duqu2qv( byref(s), byref(q), byref(v) )
