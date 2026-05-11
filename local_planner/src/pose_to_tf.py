#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

br = None

def pose_cb(msg):
    global br
    if br is None:
        return
    t = TransformStamped()
    # MAVROS is using wall time, but Gazebo is using sim time.
    # We MUST use rospy.Time.now() (sim time) so the point cloud TF lookup succeeds!
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "local_origin"
    t.child_frame_id = "fcu"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation = msg.pose.orientation
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('pose_to_tf_bridge')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb, queue_size=1)
    rospy.spin()
