#!/usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math


def print_diff(data, tag, unit):
    print("{}: {:.2f} {}".format(tag, max(data) - min(data), unit))


if __name__ == '__main__':
    rospy.init_node("tf_analyst")

    target_frame = rospy.get_param("~target_frame")
    source_frame = rospy.get_param("~source_frame")
    end_time = rospy.get_param("/end_time")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    time = []
    x = []
    y = []
    z = []
    roll = []
    pitch = []
    yaw = []

    rate = rospy.Rate(200.0)
    last_time = 0
    last_log_time = 0
    while not rospy.is_shutdown() and last_time <= end_time:
        try:
            trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
            trans_time = trans.header.stamp.to_sec()

            if trans_time - last_log_time > 5:
                last_log_time = trans_time
                rospy.loginfo("time: {}/{}".format(trans_time, end_time))

            if trans_time != last_time:
                # get new transform
                last_time = trans_time
                time.append(trans_time)

                x.append(trans.transform.translation.x)
                y.append(trans.transform.translation.y)
                z.append(trans.transform.translation.z)

                quaternion = (
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w)
                euler = euler_from_quaternion(quaternion)
                roll.append(math.degrees(euler[0]))
                pitch.append(math.degrees(euler[1]))
                yaw.append(math.degrees(euler[2]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Get transform failed!")
        rate.sleep()

    # print difference
    print_diff(x, "x", "米")
    print_diff(y, "y", "米")
    print_diff(z, "z", "米")
    print_diff(roll, "roll", "度")
    print_diff(pitch, "pitch", "度")
    print_diff(yaw, "yaw", "度")

    # plot
    time_r = [t - time[0] for t in time]

    plt.figure(1)

    plt.subplot(611)
    plt.plot(time_r, x)
    plt.ylabel("x")
    plt.xlabel("time")

    plt.subplot(612)
    plt.plot(time_r, y)
    plt.ylabel("y")
    plt.xlabel("time")

    plt.subplot(613)
    plt.plot(time_r, z)
    plt.ylabel("z")
    plt.xlabel("time")

    plt.subplot(614)
    plt.plot(time_r, roll)
    plt.ylabel("roll")
    plt.xlabel("time")

    plt.subplot(615)
    plt.plot(time_r, pitch)
    plt.ylabel("pitch")
    plt.xlabel("time")

    plt.subplot(616)
    plt.plot(time_r, yaw)
    plt.ylabel("yaw")
    plt.xlabel("time")

    plt.show()
