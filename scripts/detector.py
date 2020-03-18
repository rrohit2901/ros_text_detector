#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import argparse
import os.path
import numpy as np
import pytesseract
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils.object_detection import non_max_suppression
from text_detector.msg import sample


rospy.init_node("detector", anonymous = True)
detec_pub = rospy.Publisher("detection_result", sample, queue_size = 1)

def decode_predictions(scores, geometry):
	# grab the number of rows and columns from the scores volume, then
	# initialize our set of bounding box rectangles and corresponding
	# confidence scores
	(numRows, numCols) = scores.shape[2:4]
	rects = []
	confidences = []

	# loop over the number of rows
	for y in range(0, numRows):
		# extract the scores (probabilities), followed by the
		# geometrical data used to derive potential bounding box
		# coordinates that surround text
		scoresData = scores[0, 0, y]
		xData0 = geometry[0, 0, y]
		xData1 = geometry[0, 1, y]
		xData2 = geometry[0, 2, y]
		xData3 = geometry[0, 3, y]
		anglesData = geometry[0, 4, y]

		# loop over the number of columns
		for x in range(0, numCols):
			# if our score does not have sufficient probability,
			# ignore it
			if scoresData[x] < 0.5:
				continue

			# compute the offset factor as our resulting feature
			# maps will be 4x smaller than the input image
			(offsetX, offsetY) = (x * 4.0, y * 4.0)

			# extract the rotation angle for the prediction and
			# then compute the sin and cosine
			angle = anglesData[x]
			cos = np.cos(angle)
			sin = np.sin(angle)

			# use the geometry volume to derive the width and height
			# of the bounding box
			h = xData0[x] + xData2[x]
			w = xData1[x] + xData3[x]

			# compute both the starting and ending (x, y)-coordinates
			# for the text prediction bounding box
			endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
			endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
			startX = int(endX - w)
			startY = int(endY - h)

			# add the bounding box coordinates and probability score
			# to our respective lists
			rects.append((startX, startY, endX, endY))
			confidences.append(scoresData[x])

	# return a tuple of the bounding boxes and associated confidences
	return (rects, confidences)


def vid_calllback(data):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    (origH, origW) = image.shape[:2]
    orig = image.copy()
    (newH, newW) = (320, 320)
    rW = origW / float(newW)
    rH = origH / float(newH)

	# resize the image and grab the new image dimensions
    image = cv2.resize(image, (newW, newH))
    (H, W) = image.shape[:2]

	# define the two output layer names for the EAST detector model that
	# we are interested -- the first is the output probabilities and the
	# second can be used to derive the bounding box coordinates of text
    layerNames = [
		"feature_fusion/Conv_7/Sigmoid",
		"feature_fusion/concat_3"]

	# load the pre-trained EAST text detector
    net = cv2.dnn.readNet('/home/rrohit2901/catkin_ws/src/text_detector/scripts/frozen_east_text_detection.pb')

	# construct a blob from the image and then perform a forward pass of
	# the model to obtain the two output layer sets
    blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
		(123.68, 116.78, 103.94), swapRB=True, crop=False)
    net.setInput(blob)
    (scores, geometry) = net.forward(layerNames)

	# decode the predictions, then  apply non-maxima suppression to
	# suppress weak, overlapping bounding boxes
    (rects, confidences) = decode_predictions(scores, geometry)
    boxes = non_max_suppression(np.array(rects), probs=confidences)

	# initialize the list of results
    results = []

	# loop over the bounding boxes
    for (startX, startY, endX, endY) in boxes:
		# scale the bounding box coordinates based on the respective
		# ratios
        startX = int(startX * rW)
        startY = int(startY * rH)
        endX = int(endX * rW)
        endY = int(endY * rH)

		# in order to obtain a better OCR of the text we can potentially
		# apply a bit of padding surrounding the bounding box -- here we
		# are computing the deltas in both the x and y directions
        dX = int((endX - startX) * 0.0)
        dY = int((endY - startY) * 0.0)

		# apply padding to each side of the bounding box, respectively
        startX = max(0, startX - dX)
        startY = max(0, startY - dY)
        endX = min(origW, endX + (dX * 2))
        endY = min(origH, endY + (dY * 2))

		# extract the actual padded ROI
        roi = orig[startY:endY, startX:endX]

		# in order to apply Tesseract v4 to OCR text we must supply
		# (1) a language, (2) an OEM flag of 4, indicating that the we
		# wish to use the LSTM neural net model for OCR, and finally
		# (3) an OEM value, in this case, 7 which implies that we are
		# treating the ROI as a single line of text
        config = ("-l rus --oem 1 --psm 7")
        text = pytesseract.image_to_string(roi, config=config)
        text = text.encode('utf-8')
		# add the bounding box coordinates and OCR'd text to the list
		# of results
        results.append(((startX, startY, endX, endY), text))
        center_x = int(( startX + endX ) / 2)
        center_y = int(( startY + endY ) / 2)
        mess = sample()
        mess.x = center_x
        mess.y = center_y
        mess.data = text
        detec_pub.publish(mess)

	# sort the results bounding box coordinates from top to bottom
    results = sorted(results, key=lambda r:r[0][1])

	# loop over the result
    output = orig.copy()
    for ((startX, startY, endX, endY), text) in results:

		# strip out non-ASCII text so we can draw the text on the image
		# using OpenCV, then draw the text and a bounding box surrounding
		# the text region of the input image
        cv2.rectangle(output, (startX, startY), (endX, endY),
			(0, 0, 255), 2)
        o_text = text.decode('utf-8')
        cv2.putText(output, o_text, (startX, startY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

		# show the output image
    cv2.imshow("Text Detection", output)
    cv2.waitKey(100)


def main():
    rospy.Subscriber("video_feed", Image, vid_calllback)
    rospy.spin()

if __name__=='__main__':
    main()