#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

class YOLOv5PerceptionNode:
    def __init__(self):
        rospy.init_node('yolov5_perception_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/detected_objects', PoseArray, queue_size=10)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"Using device: {self.device}")
        # Load YOLOv5 model - assuming custom trained weights are in the package directory
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5s.pt', force_reload=False).to(self.device)
        rospy.loginfo("YOLOv5 model loaded")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image)
            detections = results.xyxy[0].cpu().numpy()  # xmin, ymin, xmax, ymax, conf, class
            pose_array = PoseArray()
            pose_array.header = msg.header
            for det in detections:
                xmin, ymin, xmax, ymax, conf, cls = det
                if conf < 0.5:
                    continue
                # For simplicity, estimate object center in image coordinates
                cx = (xmin + xmax) / 2
                cy = (ymin + ymax) / 2
                # Placeholder for 3D localization - here we just set z=0
                pose = Pose()
                pose.position.x = cx
                pose.position.y = cy
                pose.position.z = 0.0
                pose_array.poses.append(pose)
            self.pose_pub.publish(pose_array)
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        node = YOLOv5PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
