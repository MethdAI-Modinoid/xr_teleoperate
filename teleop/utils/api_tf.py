import threading
import rclpy
from rclpy.node import Node
from typing import List
import math
import numpy as np

from tf2_ros import Buffer, TransformListener
from fastapi import FastAPI, HTTPException, Query
from sensor_msgs.msg import JointState

app = FastAPI()

joint_names = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint"
]

RIGHT_HAND_OPEN = np.array([
        0.0672,  # thumb0
        0.5666,  # thumb1
        -0.0679,  # thumb2
        -0.0211,  # middle0
        -0.0112,  # middle1
        -0.0162,  # index0
        -0.0283,  # index1
    ])

RIGHT_HAND_CLOSE = np.array([
        -0.03833567723631859,  # thumb0
       -0.36572766304016113,  # thumb1
        -0.024161333218216896,  # thumb2
         0.9473425149917603,  # middle0
        -0.044050849974155426,  # middle1
        0.9455186128616333,  # index0
        -0.06319903582334518,  # index1
    ])

class TFNode(Node):
    def __init__(self):
        super().__init__("tf_api_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.new_pub = self.create_publisher(JointState, "joint_states", 10)

    def angle_pub(self, float_arr):
        new_float  = JointState()
        new_float.header.stamp = self.get_clock().now().to_msg()
        new_float.name = joint_names
        # new_float.position = float_arr[:29]
        new_float.velocity = []
        new_float.effort = []
        # print(len(new_float.position))
        # print(len(joint_names))

# curl "http://localhost:8000/pub_joints?float_arr=0.0&float_arr=0.1&float_arr=0.2&float_arr=0.3&float_arr=0.3&float_arr=0.3&float_arr=0.3

        joint_vec = [0.0] * 29

        print("float arr:", float_arr)

        for k, val in enumerate(float_arr):
            idx = 21 + k
            if idx >= len(joint_vec):
                break
            joint_vec[idx] = val
        print(len(joint_names))
        print(len(joint_vec))
        assert len(joint_names) == len(joint_vec)
        new_float.position = joint_vec

        self.new_pub.publish(new_float)
        # velocity and effort can remain empty if not used
        print("is dis looping")

    def quat_to_rpy(self, w, x, y, z):
        """
        Quaternion (w, x, y, z) -> Roll, Pitch, Yaw
        ZYX convention (yaw-pitch-roll), radians
        Assumes quaternion is normalized
        """

        # Roll (x-axis)
        sinr_cosp = 2.0 * (w*x + y*z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis)
        sinp = 2.0 * (w*y - z*x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # gimbal lock
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis)
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


tf_node = None

def ros_spin():
    rclpy.spin(tf_node)

@app.on_event("startup")
def startup():
    global tf_node
    rclpy.init()
    tf_node = TFNode()
    threading.Thread(target=ros_spin, daemon=True).start()

@app.on_event("shutdown")
def shutdown():
    rclpy.shutdown()

@app.get("/pub_joints")
def publish_joints(float_arr: List[float] = Query(...)):
    tf_node.angle_pub(float_arr)
    print("published", float_arr)
    return {"status": "published"}

@app.get("/transform")
def get_transform():
    try:
        transform = tf_node.tf_buffer.lookup_transform("pelvis", "right_rubber_hand", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0))
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        roll, pitch, yaw, = tf_node.quat_to_rpy(qw, qx, qy, qz)

        return [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, roll, pitch, yaw]
    except Exception as e:
        raise HTTPException(status_code=404, detail=str(e))
