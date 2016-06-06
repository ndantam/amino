#!/usr/bin/env python

import tf

x = tf.YAngle(3.14159)
print str(x)

#q = tf.Quat(x)
q = tf.Quat([1,2,3,4])
#q = q.normalize()


print str(q)
