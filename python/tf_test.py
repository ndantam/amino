#!/usr/bin/env python

import tf


q = tf.Quat(1,2,3,4)
q.normalize()

print str(q)
