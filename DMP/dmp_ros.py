# DMP ROS Library
#
# A ROS wrapper for the Python DMP library
#
# Authors: Timotej Gaspar
# E-mail: timotej.gaspar@ijs.si

# Numpy
# import numpy as np

# DMP python library
from dmp import DMP, CartDMP, PeriodicDMP

# ROS messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from dmp_msgs.msg import JointSpaceDMP
from dmp_msgs.msg import CartesianSpaceDMP
from dmp_msgs.msg import Float32Array


def encode_dmp(trajectory_samples=None, num_weights=None, a_z=48.0, a_x=2.0, id=0):
    """Encoding a DMP from the provided trajectory array.
    """
    
    # Assume the input trajectory is of type PoseStamped
    try:
        traj_array, time_array = __parse_pose_stamped_array(trajectory_samples)
        cart_dmp = CartDMP(traj_array,
                           time_array,
                           num_weights=num_weights,
                           a_x=a_x,
                           a_z=a_z)
        cart_dmp_ros = __convert_to_ros_dmp(cart_dmp)
        cart_dmp_ros.id = id
        return cart_dmp_ros
    except Exception:
        pass

    # Assume the input trajectory is of type JointStates
    try:
        joint_pos, joint_vel, time_vect = __parse_joint_state_array(trajectory_samples)
        # cart_dmp.id = id
        joint_dmp = DMP(pos_data=joint_pos,
                        time=time_vect,
                        vel_data=joint_vel,
                        num_weights=num_weights,
                        a_z=a_z,
                        a_x=a_x)
        joint_dmp_ros = __convert_to_ros_dmp(joint_dmp)
        joint_dmp_ros.id = id
        return joint_dmp_ros
    except Exception:
        pass

    raise Exception("Wrong data passed !!")


def __parse_joint_state_array(trajectory_samples):
        """The input trajectory samples are parsed into a shape that can be digested
        by the core DMP Python library.
        """
        # Make sure the arrays are clear before proceeding
        joint_pos = []
        joint_vel = []
        time_vect = []

        # Extract the position and velocity vector
        for sample in trajectory_samples:
            joint_pos.append(sample.position)
            joint_vel.append(sample.velocity)
            time_vect.append(sample.header.stamp.to_sec() - trajectory_samples[0].header.stamp.to_sec())

        if not any(joint_vel):
            joint_vel = []
        return joint_pos, joint_vel, time_vect


def __parse_pose_stamped_array(trajectory_samples):
    """The input trajectory samples are parsed into a shape that can be digested by
    the core DMP Python library.
    """

    traj_array = []
    time_array = []

    for pose_sample in trajectory_samples:
        traj_array.append([pose_sample.pose.position.x,
                           pose_sample.pose.position.y,
                           pose_sample.pose.position.z,
                           pose_sample.pose.orientation.w,
                           pose_sample.pose.orientation.x,
                           pose_sample.pose.orientation.y,
                           pose_sample.pose.orientation.z,
                           ])
        time_array.append(pose_sample.header.stamp.to_sec())

    return traj_array, time_array


def __convert_to_ros_dmp(dmp_in):
    """Converts a DMP class into a ROS style DMP message
    """

    # Assume the input DMP is a Cartesian Space DMP
    try:
        # Create the object with the simple attributes already copied
        dmp_out = CartesianSpaceDMP(a_x=dmp_in.a_x,
                                    a_z=dmp_in.a_z,
                                    b_z=dmp_in.b_z,
                                    tau=dmp_in.tau,
                                    N=dmp_in.num_weights,
                                    c=dmp_in.c,
                                    sigma=dmp_in.sigma)

        # Copy the rest of the attributes
        dmp_out.p0.position = Point(x=dmp_in.y0[0],
                                    y=dmp_in.y0[1],
                                    z=dmp_in.y0[2])
        dmp_out.p0.orientation = Quaternion(w=dmp_in.q0.w,
                                            x=dmp_in.q0.x,
                                            y=dmp_in.q0.y,
                                            z=dmp_in.q0.z)

        dmp_out.goal.position = Point(x=dmp_in.goal[0],
                                      y=dmp_in.goal[1],
                                      z=dmp_in.goal[2])
        dmp_out.goal.orientation = Quaternion(w=dmp_in.q_goal.w,
                                              x=dmp_in.q_goal.x,
                                              y=dmp_in.q_goal.y,
                                              z=dmp_in.q_goal.z)

        dmp_out.w = [Float32Array(dmp_in.weights_pos[0]),
                     Float32Array(dmp_in.weights_pos[1]),
                     Float32Array(dmp_in.weights_pos[2]),
                     Float32Array(dmp_in.weights_rot[0]),
                     Float32Array(dmp_in.weights_rot[1]),
                     Float32Array(dmp_in.weights_rot[2]),
                     ]

        return dmp_out
    except Exception:
        pass

    # Assume the input DMP is a Joint Space DMP
    try:
        # Create the object with the simple attributes already copied
        dmp_out = JointSpaceDMP(a_x=dmp_in.a_x,
                                a_z=dmp_in.a_z,
                                b_z=dmp_in.b_z,
                                tau=dmp_in.tau,
                                N=dmp_in.num_weights,
                                c=dmp_in.c,
                                sigma=dmp_in.sigma)

        # Copy the rest of the attributes
        dmp_out.y0 = JointState(position=dmp_in.y0.tolist())
        dmp_out.goal = JointState(position=dmp_in.goal.tolist())

        for i in range(len(dmp_out.y0.position)):
            dmp_out.w.append(Float32Array(dmp_in.weights_pos[i, :].tolist()))

        return dmp_out
    except Exception:
        pass

    raise Exception("Could not convert to a ROS type DMP !!")
