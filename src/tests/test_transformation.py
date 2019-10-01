import numpy as np
import copy
from geometry_msgs.msg import Pose
import tf
import math

from ..rosweld.calibrationpoint import CalibrationPoint
from ..rosweld.point import Point
from ..rosweld.transformation import Transformation

class TestTransformation(object):

    def nearly_equal(self, a, b, sig_fig=6):
        return ( a==b or 
            int(a*10**sig_fig) == int(b*10**sig_fig)
        )
    
    def test_get_transformation_matrix(self):
        M = Transformation.get_transformation_matrix([])
        assert M.all() == np.identity(4).all()

        point = CalibrationPoint(Point())

        M = Transformation.get_transformation_matrix([point, point])
        assert M.all() == np.array((
                [1., 0., 0., 1.],
                [0., 1., 0., 1.],
                [0., 0., 1., 1.],
                [0., 0., 0., 1.])).all()

        p = Pose()
        p.position = Point(0.,0.,0.)
        M = Transformation.get_transformation_matrix([
                CalibrationPoint(Point(0., 0.14, 0.), copy.deepcopy(p)),
                CalibrationPoint(Point(0.25, 0.12, 0.), copy.deepcopy(p)),
                CalibrationPoint(Point(0., 0., 0.), copy.deepcopy(p))]
                )

        assert M.all() == np.array((
                [0., 0., 0., 0.],
                [0., 0., 0., 0.],
                [0., 0., 0., 0.],
                [0., 0., 0., 1.])).all()

        exp = np.array([
                        [  0.99966246832397065,  -0.025979788767932607	,   0.00000000000000034520997171938461,  0.29720907924080858],
                        [  0.024466255937143053,   0.94142404386902978,  -0.3363363969981561,  -0.10156291562514719	],
                        [   0.0087379485489792949	,   0.33622287281036767,   0.94174191159483744,   0.09944181584816171	],
                        [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]).flatten()

        p1 = Pose()
        p1.position = Point(0.3, -0.1, 0.1)
        p2 = Pose()
        p2.position = Point(0.55, -0.1, 0.1)
        p3 = Pose()
        p3.position = Point(0.55, 0.04, 0.15)  

        M = Transformation.get_transformation_matrix([
                CalibrationPoint(Point(0., 0., 0.), p1),
                CalibrationPoint(Point(0.25, 0., 0.), p2),
                CalibrationPoint(Point(0.25, 0.14, 0.), p3)]
                ).flatten()
        
        assert len(M) == len(exp)
        for i in range(len(M)):
                assert self.nearly_equal(M[i], exp[i])
        assert np.allclose(M, exp)

#     def test_apply_path(self):
#         path = "415.500	-30.000	-10.000	0.000000	0.000000	1.000000\r\n\
#                 392.289	-30.000	-10.000	0.000000	0.000000	1.000000"

#         p1 = Pose()
#         p1.position = Point(0.3, -0.1, 0.1)
#         p2 = Pose()
#         p2.position = Point(0.55, -0.1, 0.1)
#         p3 = Pose()
#         p3.position = Point(0.55, 0.04, 0.1)  
#         c1 = Point(0., 0., 0.)
#         c2 = Point(0.25, 0., 0.)
#         c3 = Point(0.25, 0.14, 0.)
#         trafo_matrix = Transformation.get_transformation_matrix([
#                 CalibrationPoint(c1, p1),
#                 CalibrationPoint(c2, p2),
#                 CalibrationPoint(c3, p3)]
#                 )

#         poses = []
        
#         for line in path.splitlines():
#             data = line.split("\t")
#             pose = []
#             pose.append(float(data[0]) / 1000)
#             pose.append(float(data[1]) / 1000)
#             pose.append(float(data[2]) / 1000)
#             pose.append(float(data[3]))
#             pose.append(float(data[4]))
#             pose.append(float(data[5]))
#             poses.append(pose)

#         tf_poses = Transformation.apply_path(poses, trafo_matrix)
        
#         assert self.nearly_equal(poses[0][0] + 0.3, tf_poses[0].position.x, 3)  
#         assert self.nearly_equal(poses[1][0] + 0.3, tf_poses[1].position.x, 3)  
#         _o = tf_poses[0].orientation
#         orientation = tf.transformations.euler_from_quaternion(np.array([_o.x, _o.y, _o.z, _o.w]))
#         assert np.allclose(np.array([math.pi, 0, 0]), orientation)
