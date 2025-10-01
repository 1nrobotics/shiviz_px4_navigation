#!/usr/bin/env python3

import sys
import time
import argparse

import capnp
import numpy as np

import ecal.core.core as ecal_core

import pathlib

current_path = str(pathlib.Path(__file__).parent.resolve())
print("working in path " + current_path)

# 导入capnp消息定义路径
path_to_add = pathlib.Path(current_path) / '../../vk_sdk/capnp'
sys.path.append(str(path_to_add.resolve()))
sys.path.append("/opt/vilota/messages")

capnp.add_import_hook()
import odometry3d_capnp as eCALOdometry3d

# ROS相关导入（替换为PoseStamped）
import rospy
from geometry_msgs.msg import PoseStamped  # 关键：mavros/vision_pose/pose的消息类型
import tf.transformations  # 用于坐标系转换


from ecal.core.subscriber import MessageSubscriber

class ByteSubscriber(MessageSubscriber):
    """eCAL原始字节订阅器（保持不变）"""
    def __init__(self, name):
        topic_type = "base:byte"
        super(ByteSubscriber, self).__init__(name, topic_type)
        self.callback = None

    def receive(self, timeout=0):
        ret, msg, time = self.c_subscriber.receive(timeout)
        return ret, msg, time

    def set_callback(self, callback):
        self.callback = callback
        self.c_subscriber.set_callback(self._on_receive)

    def rem_callback(self, callback):
        self.c_subscriber.rem_callback(self._on_receive)
        self.callback = None

    def _on_receive(self, topic_name, msg, time):
        self.callback(topic_name, msg, time)    


class MavrosVisionPosePublisher:
    """发布器类：从eCAL接收位姿，转换为/mavros/vision_pose/pose"""
    def __init__(self, ecal_topic: str, use_monotonic: bool, is_ned_input: bool) -> None:
        self.first_message = True
        self.use_monotonic = use_monotonic
        self.is_ned_input = is_ned_input  # 标记输入eCAL数据是否为NED坐标系

        # 1. 初始化ROS发布器：固定话题为/mavros/vision_pose/pose
        self.vision_pose_pub = rospy.Publisher(
            "/mavros/vision_pose/pose",  # PX4监听的外部位姿话题
            PoseStamped, 
            queue_size=10  # 队列大小，避免消息堆积
        )

        # 打印初始化信息
        print(f"=== Mavros Vision Pose Publisher 初始化 ===")
        print(f"eCAL订阅话题: {ecal_topic}")
        print(f"ROS发布话题: /mavros/vision_pose/pose")
        print(f"输入坐标系 (NED): {self.is_ned_input}")
        print(f"使用单调时间戳: {self.use_monotonic}")

        # 2. 初始化eCAL订阅器并绑定回调
        self.ecal_sub = ByteSubscriber(ecal_topic)
        self.ecal_sub.set_callback(self.ecal_msg_callback)

    def ned_to_enu(self, x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z):
        """
        NED坐标系→ENU坐标系转换（PX4要求ENU输入）
        - 位置：x_ENU = y_NED, y_ENU = x_NED, z_ENU = -z_NED
        - 姿态：NED→ENU的旋转矩阵对应四元数调整（绕Z轴旋转-90°，再绕X轴旋转180°）
        """
        # 位置转换
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned

        # 姿态转换：NED四元数 → ENU四元数
        # 旋转矩阵：R_enu_ned = [[0,1,0],[1,0,0],[0,0,-1]]
        R_ned_to_enu = np.array([[0, 1, 0],
                                 [1, 0, 0],
                                 [0, 0, -1]])
        # 将NED四元数转换为旋转矩阵
        R_ned = tf.transformations.quaternion_matrix([q_ned_x, q_ned_y, q_ned_z, q_ned_w])[:3, :3]
        # 应用ENU转换
        R_enu = R_ned @ R_ned_to_enu.T
        # 转换回四元数（x,y,z,w顺序）
        q_enu = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((R_enu, np.zeros((3,1)))), [0,0,0,1]))
        )
        q_enu_w, q_enu_x, q_enu_y, q_enu_z = q_enu[3], q_enu[0], q_enu[1], q_enu[2]

        return x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z
    
    def ned_to_enu_1(self, x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z):
        """
        修正版：NED坐标系→ENU坐标系转换（PX4要求ENU输入）
        位置：x_ENU = y_NED, y_ENU = x_NED, z_ENU = -z_NED（原逻辑正确）
        姿态：NED四元数 → ENU四元数（修正旋转逻辑）
        """
        # 1. 位置转换（原逻辑正确，保留）
        x_enu = y_ned
        y_enu = x_ned
        z_enu = -z_ned

        # 2. 姿态转换：修正旋转逻辑
        # 步骤1：NED姿态 → 机体姿态（先绕X轴转180°，抵消NED与ENU的Z轴方向差异）
        # 绕X轴转180°的四元数（x,y,z,w）：(1, 0, 0, 0)
        q_rot_x180 = np.array([1.0, 0.0, 0.0, 0.0])  # x,y,z,w
        # 步骤2：绕Z轴转-90°（顺时针），对齐NED与ENU的XY轴
        # 绕Z轴转-90°的四元数（x,y,z,w）：(0, 0, -√2/2, √2/2) ≈ (0,0,-0.7071,0.7071)
        q_rot_z_neg90 = np.array([0.0, 0.0, -0.70710678, 0.70710678])  # x,y,z,w

        # 将NED四元数转换为（x,y,z,w）格式（与tf一致）
        q_ned = np.array([q_ned_x, q_ned_y, q_ned_z, q_ned_w])

        # 姿态旋转：先乘X轴180°，再乘Z轴-90°（tf.transformations.quaternion_multiply是右乘）
        q_temp = tf.transformations.quaternion_multiply(q_ned, q_rot_x180)
        q_enu = tf.transformations.quaternion_multiply(q_temp, q_rot_z_neg90)

        # 提取ENU四元数的w,x,y,z（tf输出是x,y,z,w，需对应PoseStamped字段）
        q_enu_x, q_enu_y, q_enu_z, q_enu_w = q_enu[0], q_enu[1], q_enu[2], q_enu[3]

        return x_enu, y_enu, z_enu, q_enu_w, q_enu_x, q_enu_y, q_enu_z


    def ecal_msg_callback(self, topic_name, msg, time_ecal):
        """eCAL消息回调：解析→转换→发布到/mavros/vision_pose/pose"""
        # 解析eCAL的Odometry3d消息
        with eCALOdometry3d.Odometry3d.from_bytes(msg) as odom_msg:
            # 首次接收消息时打印元信息
            if self.first_message:
                print(f"eCAL消息体信息:")
                print(f"  参考帧: {odom_msg.referenceFrame}")
                print(f"  机体帧: {odom_msg.bodyFrame}")
                print(f"  速度帧: {odom_msg.velocityFrame}")
                self.first_message = False

            # 每100帧打印一次数据（调试用，可注释）
            if odom_msg.header.seq % 100 == 0:
                print(f"\n帧序号: {odom_msg.header.seq}")
                print(f"设备延迟: {odom_msg.header.latencyDevice / 1e6:.2f} ms")
                print(f"NED原始位置: ({odom_msg.pose.position.x:.2f}, {odom_msg.pose.position.y:.2f}, {odom_msg.pose.position.z:.2f})")

            # --------------------------
            # 1. 初始化PoseStamped消息
            # --------------------------
            pose_stamped = PoseStamped()

            # 时间戳：优先使用eCAL消息自带时间（转换为ROS时间）
            if self.use_monotonic:
                # 若使用单调时间，从eCAL的stamp（ns）转换
                pose_stamped.header.stamp = rospy.Time.from_sec(odom_msg.header.stamp / 1e9)
            else:
                # 否则使用ROS当前时间（不推荐，可能有延迟）
                pose_stamped.header.stamp = rospy.Time.now()

            # 帧ID：固定为"odom"（PX4默认参考帧，与MAVROS配置匹配）
            pose_stamped.header.frame_id = "odom"

            # --------------------------
            # 2. 数据映射与坐标系转换
            # --------------------------
            # 提取eCAL中的NED位姿
            x_ned = odom_msg.pose.position.x
            y_ned = odom_msg.pose.position.y
            z_ned = odom_msg.pose.position.z
            q_ned_w = odom_msg.pose.orientation.w
            q_ned_x = odom_msg.pose.orientation.x
            q_ned_y = odom_msg.pose.orientation.y
            q_ned_z = odom_msg.pose.orientation.z

            # 若输入是NED，转换为PX4要求的ENU
            if self.is_ned_input:
                x, y, z, q_w, q_x, q_y, q_z = self.ned_to_enu(
                    x_ned, y_ned, z_ned, q_ned_w, q_ned_x, q_ned_y, q_ned_z
                )
            else:
                # 若已是ENU，直接使用
                x, y, z = x_ned, y_ned, z_ned
                q_w, q_x, q_y, q_z = q_ned_w, q_ned_x, q_ned_y, q_ned_z

            # 映射到位姿消息
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = q_w
            pose_stamped.pose.orientation.x = q_x
            pose_stamped.pose.orientation.y = q_y
            pose_stamped.pose.orientation.z = q_z

            # --------------------------
            # 3. 发布消息到/mavros/vision_pose/pose
            # --------------------------
            self.vision_pose_pub.publish(pose_stamped)


def main():  
    # 打印eCAL版本信息
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    # --------------------------
    # 解析命令行参数
    # --------------------------
    parser = argparse.ArgumentParser(description="从eCAL订阅位姿，发布到/mavros/vision_pose/pose")
    parser.add_argument(
        '--ecal_topic', 
        type=str, 
        default="S1/vio_odom", 
        help="eCAL订阅话题（默认：S1/vio_odom）"
    )
    parser.add_argument(
        '--is_ned_input', 
        action="store_true", 
        help="是否输入NED坐标系数据（默认：False，即ENU）"
    )
    parser.add_argument(
        '--monotonic_time', 
        action="store_true", 
        help="是否使用eCAL消息自带的单调时间戳（推荐：True）"
    )
    args = parser.parse_args()

    # --------------------------
    # 初始化eCAL和ROS
    # --------------------------
    # 初始化eCAL
    ecal_core.initialize(sys.argv, "mavros_vision_pose_bridge")
    ecal_core.set_process_state(1, 1, "Running")  # 标记进程状态为运行中

    # 初始化ROS节点（名称唯一，避免冲突）
    rospy.init_node("ecal_to_mavros_vision_pose", anonymous=True)

    # --------------------------
    # 启动发布器并阻塞
    # --------------------------
    # 创建发布器实例
    publisher = MavrosVisionPosePublisher(
        ecal_topic=args.ecal_topic,
        use_monotonic=args.monotonic_time,
        is_ned_input=args.is_ned_input
    )

    # ROS阻塞（保持节点运行）
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n用户终止程序")
    finally:
        # 释放eCAL资源
        ecal_core.finalize()
        print("eCAL资源已释放")


if __name__ == "__main__":
    main()
