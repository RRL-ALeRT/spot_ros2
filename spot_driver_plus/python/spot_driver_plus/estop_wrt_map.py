#!/usr/bin/env python3

import openvino as ov
import cv2
from yolov8_pose_utils import *

import numpy as np
from scipy.spatial.transform import Rotation as R

from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Pose, TransformStamped, Point32

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import cv_bridge
from sensor_msgs.msg import Image
from world_info_msgs.msg import BoundingBox, BoundingBoxArray, Keypoints, KeypointsArray

DEBUG = False

# Relative 3D coordinates of 5 points with the first point as the origin (0,0,0)
# Centre, TL, TR, BR, BL
relative_points = np.array([[0, 0, 0],
                            [-0.13, 0.13, -0.08*1],
                            [0.13, 0.13, -0.08*1],
                            [0.13, -0.13, -0.08*1],
                            [-0.13, -0.13, -0.08*1]], dtype=np.float32)

IMAGE_TOPIC = '/rs_front/color/image_raw'
SCORE_THRESHOLD = 0.8
BOUNDING_BOX_EDGE_LIMIT = 30

fx = 618.9777221679688
fy = 619.1235961914062
cx = 330.1564025878906
cy = 236.6548614501953
# Camera intrinsic parameters
focal_length = (fx, fy)  # Focal length in pixels
principal_point = (cx, cy)  # Principal point (center of the image) in pixels
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

k1 = k2 = p1 = p2 = k3 = 0
# Distortion coefficients (if any)
dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

# Load a model
pose_model_path = "/home/max1/Desktop/shubham_thesis/estop_set.onnx"
device = "GPU"
kpt_shape = [5, 2]
label_map = ["estop_set"]


class FramePublisher(Node):
    def __init__(self):
        super().__init__('tf2_frame_publisher')

        self.create_subscription(Image, IMAGE_TOPIC, self.camera_cb, 1)

        self.bounding_box_pub = self.create_publisher(BoundingBoxArray, IMAGE_TOPIC + '/bb', 1)
        self.keypoints_pub = self.create_publisher(KeypointsArray, IMAGE_TOPIC + '/kp', 1)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.publish_tf2_continuously)

        core = ov.Core()
        pose_ov_model = core.read_model(pose_model_path)

        if device != "CPU":
            pose_ov_model.reshape({0: [1, 3, 640, 640]})
        self.compiled_model = core.compile_model(pose_ov_model, device)

    def camera_cb(self, msg):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, 'bgr8')

        input_image = image[:, :, ::-1]
        results = detect(input_image, self.compiled_model, kpt_shape)

        image_width = image.shape[1]
        image_height = image.shape[0]

        bb_msg = BoundingBoxArray()
        bb_msg.header = msg.header
        bb_msg.type = label_map[0]

        kp_msg = KeypointsArray()
        kp_msg.header = msg.header
        kp_msg.type = label_map[0]

        for detection in results:
            if len(detection['box']) == 0:
                continue
            keypoints = detection.get('kpt')[0]
            if keypoints is None:
                continue
            box = detection['box'][0]
            x1, y1, x2, y2, score, label = box
            if x1 <= BOUNDING_BOX_EDGE_LIMIT or x2 >= image_width - BOUNDING_BOX_EDGE_LIMIT or  \
               y1 <= BOUNDING_BOX_EDGE_LIMIT or y2 >= image_height - BOUNDING_BOX_EDGE_LIMIT:
                continue

            if score < SCORE_THRESHOLD:
                continue

            EDGE_LIMIT_FOR_BOUNDING_BOX = 20

            keypoints_near_edge = False
            for keypoint in keypoints:
                if not (EDGE_LIMIT_FOR_BOUNDING_BOX < keypoint[0] < image_width - EDGE_LIMIT_FOR_BOUNDING_BOX):
                    continue
                if not (EDGE_LIMIT_FOR_BOUNDING_BOX < keypoint[1] < image_height - EDGE_LIMIT_FOR_BOUNDING_BOX):
                    continue

            if len(keypoints > 0):
                u1 = keypoints[0][0]
                v1 = keypoints[0][1]
                u2 = keypoints[1][0]
                v2 = keypoints[1][1]
                u3 = keypoints[2][0]
                v3 = keypoints[2][1]
                u4 = keypoints[3][0]
                v4 = keypoints[3][1]
                u5 = keypoints[4][0]
                v5 = keypoints[4][1]
            else:
                continue            

            image_points = np.array([[u5, v5],
                                     [u1, v1],
                                     [u2, v2],
                                     [u4, v4],
                                     [u3, v3]], dtype=np.float32)

            label = int(label)
            label_name = label_map[label] if label < len(label_map) else 'Unknown'
            label_str = f'{label_name}: {score:.2f}'  # Create a label string

            if DEBUG:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                # Draw the bounding box label
                cv2.putText(image, label_str, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)

                for point in image_points:
                    image = cv2.circle(image, (int(point[0]), int(point[1])), 1, (0,0,255), 5)

            bb = BoundingBox()
            bb.name = label_map[0]
            bb.confidence = float(score)
            bb.cx = float(x1)
            bb.cy = float(y1)
            bb.width = float(abs(x2 - x1))
            bb.height = float(abs(y2 - y1))
            bb_msg.array.append(bb)

            kp = Keypoints()
            kp.name = label_map[0]
            for image_kp in image_points:
                point = Point32()
                point.x = float(image_kp[0])
                point.y = float(image_kp[1])
                kp.array.append(point)
            kp_msg.array.append(kp)

            # Use solvePnP to estimate camera pose
            success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(relative_points, image_points, camera_matrix, dist_coeffs)
            # success, rotation_vector, translation_vector = cv2.solvePnP(relative_points, image_points, camera_matrix, dist_coeffs)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Convert the rotation matrix to a quaternion
            r = R.from_matrix(rotation_matrix)
            quaternion = r.as_quat()

            self.build_tf2(translation_vector, quaternion, msg.header.frame_id)

        self.bounding_box_pub.publish(bb_msg)
        self.keypoints_pub.publish(kp_msg)

        if DEBUG:
            cv2.imshow("", image)
            cv2.waitKey(1)

    def build_tf2(self, xyz, quat, header_frame_id):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                header_frame_id,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {header_frame_id} to map: {ex}')
            return

        target_pose = Pose()
        target_pose.position.x = xyz[0][0]
        target_pose.position.y = xyz[1][0]
        target_pose.position.z = xyz[2][0]
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]
        target_pose_wrt_map = do_transform_pose(target_pose, t)

        self.t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = label_map[0]

        self.t.transform.translation.x = target_pose_wrt_map.position.x
        self.t.transform.translation.y = target_pose_wrt_map.position.y
        self.t.transform.translation.z = target_pose_wrt_map.position.z

        self.t.transform.rotation.x = target_pose_wrt_map.orientation.x
        self.t.transform.rotation.y = target_pose_wrt_map.orientation.y
        self.t.transform.rotation.z = target_pose_wrt_map.orientation.z
        self.t.transform.rotation.w = target_pose_wrt_map.orientation.w

    def publish_tf2_continuously(self):
        if hasattr(self, 't'):
            # Send the transformation
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.t)


rclpy.init()
node = FramePublisher()

rclpy.spin(node)

rclpy.shutdown()
