#!/usr/bin/env python

import libamino as aa
reload(aa)

import ctypes

class Orientation(object):
    __slots__=[]
    def orientation(self):
        return self._quat(Quat.create())
    def rotation(self):
        return self._rotmat(RotMat.create())

class OrientationStruct(ctypes.Structure,Orientation):
    __slots__=[]


######################
## Principal Angles ##
######################

class PrincipalAngle(Orientation):
    """A rotation about a principal axis"""
    __slots__ = ['angle']

    def __init__(self,angle):
        self.angle = angle

    def __mul__(self,other):
        return quat(self) * other

    def __rmul__(self,other):
        return other * quat(self)


    def _duqu(self,other):
        """Convert `self' to a dual quaternion and store in `other'"""
        other.dx=0.0
        other.dy=0.0
        other.dz=0.0
        other.dw=0.0
        return self._quat(other)

class XAngle(PrincipalAngle):
    """A rotation about the X axis"""

    def _quat(self,other):
        """Convert `self' to a quaternion and store in `other'"""
        return aa.tf_xangle2quat(self.angle, other)
    def _rotmat(self,other):
        """Convert `self' to a rotation matrix and store in `other'"""
        return aa.tf_xangle2rotmat(self.angle, other)
    def _eulerzyx(self,other):
        other.x=self.angle
        other.y=0.0
        other.z=0.0
        return other

    def __repr__(self):
        return "XAngle({0})".format(self.angle)


class YAngle(PrincipalAngle):
    """A rotation about the Y axis"""
    def _quat(self, other):
        """Convert `self' to a quaternion and store in `other'"""
        return aa.tf_yangle2quat(self.angle, other)
    def _rotmat(self,other):
        """Convert `self' to a rotation matrix and store in `other'"""
        return aa.tf_yangle2rotmat(self.angle, other)
    def _eulerzyx(self,other):
        other.x=0.0
        other.y=self.angle
        other.z=0.0
        return other

    def __repr__(self):
        return "YAngle({0})".format(self.angle)

class ZAngle(PrincipalAngle):
    """A rotation about the Z axis"""
    def _quat(self, other):
        """Convert `self' to a quaternion and store in `other'"""
        return aa.tf_zangle2quat(self.angle, other )
    def _rotmat(self,other):
        """Convert `self' to a rotation matrix and store in `other'"""
        return aa.tf_zangle2rotmat(self.angle, other)
    def _eulerzyx(self,other):
        other.x=0.0
        other.y=0.0
        other.z=self.angle
        return other

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
    __slots__=[]

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
        return thing if isinstance(thing,Vec) else vec(thing)

    def __str__(self):
        return "({0}i + {1}j + {2}k)".format(self.x,
                                             self.y,
                                             self.z)

    def __repr__(self):
        return "Vec([{0}, {1}, {2}])".format(self.x,
                                             self.y,
                                             self.z )

    def _vec(self,other):
        return aa.fcpy(other,self,3)

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
        return other._vec(Vec.create())

def dot(a,b):
    """Compute the dot product"""
    return Vec.ensure(a).dot(b)

def cross(a,b):
    """Compute the cross product"""
    return Vec.ensure(a).cross(b)

##########################
## Ordinary Quaternions ##
##########################

class Quat(OrientationStruct):
    """Ordinary Quaternions"""

    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double),
                ("w", ctypes.c_double)]
    __slots__=[]

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
        return thing if isinstance(thing,Quat) else quat(thing)

    def _quat(self,other):
        """Copy `self' into the quaternion `other'"""
        return aa.fcpy(other,self,4)

    def _rotmat(self,other):
        return aa.tf_quat2rotmat(self,other)

    def _eulerzyx(self,other):
        return aa.tf_quat2eulerzyx(self,other)

    def orientation(self):
        return self._quat(Quat.create())

    def rotation(self):
        return self._rotmat(RotMat.create())

    def _duqu(self,other):
        """Copy `self' into the dual quaternion `other'"""
        other.dx = 0.0
        other.dy = 0.0
        other.dz = 0.0
        other.dw = 0.0
        return self._quat(other)

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
        return self._quat(Quat.create())

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


#####################
## Rotation Matrix ##
#####################
class RotMat(ctypes.Array,Orientation):
    _length_ = 9
    _type_ = ctypes.c_double
    __slots__=[]

    @staticmethod
    def from_elts(xx,yx,zx, xy,yy,zy, xz,yz,zz):
        """Create a rotation matrix from elements"""
        return RotMat(xx,yz,zx, xy,yy,zy, xz,yz,zz)

    @staticmethod
    def identity():
        """Create a dual quaternion"""
        return RotMat(1,0,0, 0,1,0, 0,0,1)

    @staticmethod
    def create():
        """Create a dual quaternion"""
        return RotMat.identity()

    @staticmethod
    def ensure(thing):
        """Ensure `thing' is a rotation matrix"""
        return thing if isinstance(thing,RotMat) else rotmat(thing)

    def _quat(self,other):
        return aa.tf_rotmat2quat(self,other)

    def _duqu(self,other):
        other.dx = 0.0
        other.dy = 0.0
        other.dz = 0.0
        other.dw = 0.0
        return self._quat(other)

    def _eulerzyx(self,other):
        return aa.tf_rotmat2eulerzyx(self,other)


    def __repr__(self):
        return "RotMat({0},{1},{2}, {3},{4},{5}, {6},{7},{8})".format(
            self[0], self[1], self[2],
            self[3], self[4], self[5],
            self[6], self[7], self[8],
        )

    def __str__(self):
        return ("[{0}\t{3}\t{6}]\n" +
                "[{1}\t{4}\t{7}]\n" +
                "[{2}\t{5}\t{8}]").format(
            self[0], self[1], self[2],
            self[3], self[4], self[5],
            self[6], self[7], self[8]
        )

    def __mul__(self,other):
        return aa.tf_rotmat_mul(self,RotMat.ensure(other),RotMat.create())

    def __rmul__(self,other):
        return aa.tf_rotmat_mul(RotMat.ensure(other),self,RotMat.create())

def rotmat(thing):
    """Convert `thing' to a rotation matrix"""
    return thing._rotmat(RotMat())

###########################
## Transformation Matrix ##
###########################
class TfMat(ctypes.Array):
    _length_ = 12
    _type_ = ctypes.c_double
    __slots__=[]

    @staticmethod
    def from_elts(xx,yx,zx, xy,yy,zy, xz,yz,zz, tx,ty,tz):
        """Create a rotation matrix from elements"""
        return TfMat(xx,yz,zx, xy,yy,zy, xz,yz,zz, tx,ty,tz)

    @staticmethod
    def identity():
        """Create a dual quaternion"""
        return TfMat(1,0,0, 0,1,0, 0,0,1, 0,0,0)

    @staticmethod
    def create():
        """Create a dual quaternion"""
        return TfMat.identity()

    @staticmethod
    def ensure(thing):
        """Ensure that `thing' is a TfMat"""
        return thing if isinstance(thing,TfMat) else tfmat(thing)

    def _duqu(self,other):
        return aa.tf_tfmat2duqu(self,other)

    def translation(self):
        return Vec.from_xyz(self[9],self[10],self[11])

    def orientation(self):
        return aa.tf_rotmat2quat(self,Quat.create())

    def rotation(self):
        return aa.fcpy(RotMat.create(),self,9)

    def __mul__(self,other):
        return aa.tf_tfmat_mul(self,TfMat.ensure(other),TfMat.create())

    def __rmul__(self,other):
        return aa.tf_tfmat_mul(TfMat.ensure(other),self,TfMat.create())

    def __repr__(self):
        return "TfMat({0},{1},{2}, {3},{4},{5}, {6},{7},{8}, {9},{10},{11})".format(
            self[0], self[1], self[2],
            self[3], self[4], self[5],
            self[6], self[7], self[8],
            self[9], self[10], self[11]
        )

    def __str__(self):
        return ("[{0}\t{3}\t{6}\t| {9}]\n" +
                "[{1}\t{4}\t{7}\t| {10}]\n" +
                "[{2}\t{5}\t{8}\t| {11}]\n" +
                "[0.0\t0.0\t0.0\t| 1.0]" ).format(
            self[0], self[1], self[2],
            self[3], self[4], self[5],
            self[6], self[7], self[8],
            self[9], self[10], self[11]
        )

def tfmat(thing):
    """Convert `thing' to a rotation matrix"""
    return thing._tfmat(TfMat.create())




#################
## Euler Angle ##
#################

class EulerZYX(OrientationStruct):
    _fields_ = [("z", ctypes.c_double),
                ("y", ctypes.c_double),
                ("x", ctypes.c_double)]
    __slots__=[]

    @staticmethod
    def from_zyx(z,y,x):
        """Create a ZYX Euler angle from z, y, and x"""
        return EulerZYX(z,y,x)

    @staticmethod
    def from_rpy(z,y,x):
        """Create a ZYX Euler angle from roll, pitch, and yaw"""
        return EulerZYX(y,p,r)

    @staticmethod
    def from_ypr(y,p,r):
        """Create a ZYX Euler angle from yaw, pitch, and roll"""
        return EulerZYX(y,p,r)

    def __repr__(self):
        return "EulerZYX({0},{1},{2})".format(self.z, self.y, self.x)

    def _quat(self,other):
        return aa.tf_eulerzyx2quat(self.z,self.y,self.x,other)

    def _duqu(self,other):
        return aa.tf_eulerzyx2quat(self.z,self.y,self.x,other)

    def orientation(self):
        return self._quat(Quat.create())

    def rotation(self):
        return self._rotmat(RotMat.create())

    def _rotmat(self,other):
        return aa.tf_eulerzyx2rotmat(self.z,self.y,self.x,other)


    def __mul__(self,other):
        return quat(self) * other

    def __rmul__(self,other):
        return other * quat(self)

def eulerzyx(thing):
    """Convert `thing' to an Euler ZYX angle"""
    return thing._eulerzyx(EulerZYX())

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
    __slots__=[]

    @staticmethod
    def from_elts(rx,ry,rz,rw, dx,dy,dz,dw):
        """Create a dual quaternion from elements"""
        return DuQu(rx,ry,rz,rw, dx,dy,dz,dw)

    @staticmethod
    def create():
        """Create a dual quaternion"""
        return DuQu()

    @staticmethod
    def ensure(thing):
        """Ensure that `thing' is a vector"""
        return thing if isinstance(thing,DuQu) else duqu(thing)


    def __str__(self):
        return "({0}+{1}i+{2}j+{3}k) +  ({4}+{5}i+{6}j+{7}k)e".format(
            self.rw, self.rx, self.ry, self.rz,
            self.dw, self.dx, self.dy, self.dz )

    def __repr__(self):
        return "DuQu({0}, {1}, {2}, {3},   {4}, {5}, {6}, {7})".format(
            self.rx, self.ry, self.rz, self.rw,
            self.dx, self.dy, self.dz, self.dw )

    def real(self):
        return Quat.from_xyzw( self.rx, self.ry, self.rz, self.rw )

    def dual(self):
        return Quat.from_xyzw( self.dx, self.dy, self.dz, self.dw )

    def _duqu(self,other):
        return aa.fcpy(other,self,8)

    def _tfmat(self,other):
        return aa.tf_duqu2tfmat(self,other)

    def orientation(self):
        return self.real()

    def rotation(self):
        return aa.tf_quat2rotmat(self,RotMat.create())

    def translation(self):
        return aa.tf_duqu_trans(self,Vec.create())


    def __mul__(self, other):
        return aa.tf_duqu_mul( self, DuQu.ensure(other), DuQu.create() )

    def __rmul__(self, other):
       return aa.tf_duqu_mul( DuQu.ensure(other), self, DuQu.create() )

    def __add__(self, other):
        return aa.tf_duqu_add( self, DuQu.ensure(other), DuQu.create() )

    def __radd__(self, other):
        return aa.tf_duqu_add( DuQu.ensure(other), self, DuQu.create() )

    def __sub__(self, other):
        return aa.tf_duqu_sub( self, DuQu.ensure(other), DuQu.create() )

    def __rsub__(self, other):
        return aa.tf_duqu_sub( DuQu.ensure(other), self, DuQu.create() )



    def __neg__(self):
        return DuQu.from_elts( -self.rx, -self.ry, -self.rz, -self.rw,
                               -self.dx, -self.dy, -self.dz, -self.dw )

    def __pos__(self):
        return self._duqu(DuQu.create())


    def __len__(self):
        return 8

    def __getitem__(self,i):
        if   0 == i: return self.rx
        elif 1 == i: return self.ry
        elif 2 == i: return self.rz
        elif 3 == i: return self.rw
        elif 4 == i: return self.dx
        elif 5 == i: return self.dy
        elif 6 == i: return self.dz
        elif 7 == i: return self.dw
        else: raise IndexError

    def __setitem__(self,i,value):
        if   0 == i: self.rx = value
        elif 1 == i: self.ry = value
        elif 2 == i: self.rz = value
        elif 3 == i: self.rw = value
        elif 4 == i: self.dx = value
        elif 5 == i: self.dy = value
        elif 6 == i: self.dz = value
        elif 7 == i: self.dw = value
        else: raise IndexError

def duqu(thing):
    """Convert `thing' to a dual quaternion"""
    if type(thing) is list:
        return DuQu.from_elts( thing[0],thing[1],thing[2],thing[3],
                               thing[4],thing[5],thing[6],thing[7] )
    elif type(thing) is tuple:
        return DuQu.from_elts( thing[0],thing[1],thing[2],thing[3],
                               thing[4],thing[5],thing[6],thing[7] )
    elif type(thing) is float:
        return DuQu.from_elts(0.0, 0.0, 0.0, thing,
                              0.0, 0.0, 0.0, 0.0)
    elif type(thing) is int:
        return DuQu.from_elts(0.0, 0.0, 0.0, float(thing),
                              0.0, 0.0, 0.0, 0.0)
    elif type(thing) is complex:
        return DuQu.from_elts(thing.imag, 0.0, 0.0, thing.real,
                              0.0, 0.0, 0.0, 0.0)
    else:
        return thing._duqu(DuQu.create())

def duqu2(orientation,translation):
    """Convert `thing' to a dual quaternion"""
    q = Quat.ensure(orientation)
    v = Vec.ensure(translation)
    return aa.tf_qv2duqu(q,v,DuQu.create())


#############################
## Quaternions-Translation ##
#############################

# class QuTr:
#     __slots__ = ["quat", "vec"]
