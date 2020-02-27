#!/usr/bin/env python3

from __future__ import print_function

from pydrake.common.eigen_geometry import (
  Quaternion
)
from pydrake.math import (
  RollPitchYaw
)
from maliput.api import (
    LanePosition,
    LanePositionResult,
    GeoPosition,
    RoadGeometry,
    RoadGeometryId,
    RoadPosition,
    Rotation
)
import maliput.math as maliput


# Using maliput::math binding from pybind11 2.0.
geo_pos = GeoPosition(5,6,7)
p = geo_pos.xyz()
print('({}, {}, {}) and size: {}'.format(p[0], p.y(), p.z(), p.size()))
quat = maliput.Quaternion(1,2,3,4)
print('({}, {}, {}, {})'.format(quat.w(), quat.x(), quat.y(), quat.z()))

# Using drake and eigen bindings from pybind11 2.4.
d = RollPitchYaw(1,5,6)
d_vector = d.vector()
print('({}, {}, {})'.format(d_vector[0], d_vector[1], d_vector[2]))
d_quat = d.ToQuaternion()
print('({}, {}, {}, {})'.format(d_quat.w(), d_quat.x(), p.y(), p.z()))

