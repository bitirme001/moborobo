#!/usr/bin/env python
#sensor_msgs/CompressedImage

# many thanks: https://github.com/ultralytics/ultralytics/issues/561
     # and: https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

import rospy
import cv2
import time
import numpy as np
import torch

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mobile_sam import sam_model_registry, SamPredictor


class RoadDetector():

	def __init__(self):
		self.bridge = CvBridge()
		self.output = None


	def overlay(self, image, mask, color, alpha, resize=None):

		color = color[::-1]
		colored_mask = np.expand_dims(mask, 0).repeat(3, axis=0)
		colored_mask = np.moveaxis(colored_mask, 0, -1)
		masked = np.ma.MaskedArray(image, mask=colored_mask, fill_value=color)
		image_overlay = masked.filled()

		if resize is not None:
			image = cv2.resize(image.transpose(1, 2, 0), resize)
			image_overlay = cv2.resize(image_overlay.transpose(1, 2, 0), resize)

		image_combined = cv2.addWeighted(image, 1 - alpha, image_overlay, alpha, 0)

		return image_combined


	def callback(self, msg):

		try:

			cv2_img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2RGB)
			sam_checkpoint = "MobileSAM/weights/mobile_sam.pt"
			model_type = "vit_t"

			device = "cuda" if torch.cuda.is_available() else "cpu"
			sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
			sam.to(device=device)
			sam.eval()
			
			# Very Slow
			predictor = SamPredictor(sam)
			predictor.set_image(cv2_img)
			
			input_point = np.array([[cv2_img.shape[1]//2, cv2_img.shape[0] - 25]])
			input_label = np.array([1])

			mask, _, _ = predictor.predict(
				point_coords=input_point,
				point_labels=input_label,
				multimask_output=False,
			)

			self.output = self.overlay(cv2_img, mask, (0,255,0), 0.3)


		except CvBridgeError as e:
			print(e)

		else:
			pass



	def road_publish(self):

		pub = rospy.Publisher("zed_node/rgb/road", Image, queue_size=10)
		rospy.init_node("road_detector", anonymous=True)
		rospy.Subscriber("zed_node/rgb/image_rect_color", Image, self.callback)
		rate = rospy.Rate(100)
		
		while not rospy.is_shutdown():
			if self.output is not None:
				pub.publish(self.bridge.cv2_to_imgmsg(self.output))
			rate.sleep()


if __name__ == "__main__":
	RoadDetector().road_publish()
