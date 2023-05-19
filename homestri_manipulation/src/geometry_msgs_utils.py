from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped, Pose, Wrench, Transform, Vector3, Quaternion
import numpy as np
import quaternion
import rospy
import tf.transformations

def create_pose(tx, ty, tz, rx, ry, rz, rw):
    pose = Pose()
    pose.position.x = tx
    pose.position.y = ty
    pose.position.z = tz
    pose.orientation.x = rx
    pose.orientation.y = ry
    pose.orientation.z = rz
    pose.orientation.w = rw

    return pose

def stamp_pose(pose, frame="", time=None):
    if time == None: time = rospy.Time.now()

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = frame
    poseStamped.header.stamp = time
    poseStamped.pose = pose
    return poseStamped


def create_wrench(fx, fy, fz, tx, ty, tz):
    wrench = Wrench()
    wrench.force.x = fx
    wrench.force.y = fy
    wrench.force.z = fz
    wrench.torque.x = tx
    wrench.torque.y = ty
    wrench.torque.z = tz

    return wrench


def stamp_wrench(wrench, frame="", time=None):
    if time == None: time = rospy.Time.now()

    wrenchStamped = WrenchStamped()
    wrenchStamped.header.frame_id = frame
    wrenchStamped.header.stamp = time
    wrenchStamped.wrench = wrench
    return wrenchStamped

def stamp_transform(transform, child_frame_id="", frame="", time=None):
    if time == None: time = rospy.Time.now()

    transformStamped = TransformStamped()
    transformStamped.child_frame_id = child_frame_id
    transformStamped.header.frame_id = frame
    transformStamped.header.stamp = time
    transformStamped.transform = transform
    return transformStamped

def pose_to_matrix(p):
    # create trans and rot vectors
    trans = [p.position.x, p.position.y, p.position.z]
    rot = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    # numpy arrays to 4x4 transform matrix 
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)

    mat = np.dot(trans_mat, rot_mat)

    return mat, trans_mat, rot_mat

def matrix_to_pose(mat):
    rot = tf.transformations.quaternion_from_matrix(mat)
    trans = tf.transformations.translation_from_matrix(mat)

    pose = create_pose(
            trans[0],
            trans[1],
            trans[2],
            rot[0],
            rot[1],
            rot[2],
            rot[3]
        )

    return pose

def rotational_error(q1, q2):
    q1 = np.quaternion(q1.x, q1.y, q1.z, q1.w)
    q2 = np.quaternion(q2.x, q2.y, q2.z, q2.w)

    q_err = q1 * q2.conjugate()

    if q_err.w < 0:
        angle_of_rotation = 2 * np.arccos(-q_err.w)
    else:
        angle_of_rotation = 2 * np.arccos(q_err.w)

    # q_err_norm = q_err / np.linalg.norm(quaternion.as_float_array(q_err))
    # axis = np.array([q_err_norm.x, q_err_norm.y, q_err_norm.z])

    # axis_of_rotation = angle_of_rotation * axis
    # axis_of_rotation_magnitude = np.linalg.norm(axis_of_rotation)

    return angle_of_rotation #, axis_of_rotation_magnitude

def translational_error(t1, t2):
    t1 = np.array((t1.x, t1.y, t1.z))
    t2 = np.array((t2.x, t2.y, t2.z))

    dist = np.linalg.norm(t1 - t2)

    return dist