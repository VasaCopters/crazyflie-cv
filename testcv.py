#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("../lib")

import cv2, math
import numpy as np
import time
from threading import Thread
import cflib
import cflib.crtp
from threading import Timer
from cflib.crazyflie import Crazyflie
from cfclient.utils.logconfigreader import LogConfig
import logging

logging.basicConfig(level=logging.ERROR)

def nothing(x):
		pass

class CrazyTracker:
	def __init__(self):
		self._log = False;
		self._cv = False;
		
		if self._cv:		
			# Init cameras
			self._camera_devices = [0, 1]
			self._w = 640
			self._h = 480
			self._camera = {}
			self._pos = {}
			self._z_orig = -300;
			self._x_orig = 0;
			self._y_orig = 0;

			# Camera properties
			self._clip = 620

			for i in self._camera_devices:
				self._camera[i] = cv2.VideoCapture(i)
				self._camera[i].set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
				self._camera[i].set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
				self._pos[i] = ((0,0),0)
				self._camera[i].set(cv2.CAP_PROP_EXPOSURE, -6)

			# Overlay to see thresholds
			self._overlay = np.zeros((self._h,self._w, 3), np.uint8)
			self._overlay[:,:,:] = (255, 221, 201)
		
		# Create controlpanel
		cv2.namedWindow('Controls')
		cv2.createTrackbar('H-Value','Controls',251,255,nothing)
		cv2.createTrackbar('H-Range','Controls',15,128,nothing)
		cv2.createTrackbar('S-Low','Controls',102,255,nothing)
		cv2.createTrackbar('S-High','Controls',255,255,nothing)
		cv2.createTrackbar('V-Low','Controls',95,255,nothing)
		cv2.createTrackbar('V-High','Controls',245,255,nothing)
		cv2.createTrackbar('Gauss','Controls',10,50,nothing)
		cv2.createTrackbar('MinSize','Controls',4,96,nothing)
		cv2.createTrackbar('Overlay', 'Controls',0,1,nothing)
		cv2.createTrackbar('Reset', 'Controls',0,1,nothing)
		cv2.createTrackbar('ON/OFF', 'Controls',0,1,nothing)
		cv2.createTrackbar('Start', 'Controls',0,1,nothing)
		cv2.createTrackbar('Land', 'Controls',0,1,nothing)
		cv2.createTrackbar('Cancel', 'Controls',0,1,nothing)
		cv2.createTrackbar('Command', 'Controls',0,1,nothing)
		cv2.createTrackbar('Param', 'Controls',0,255,nothing)
		cv2.resizeWindow('Controls', 350, 300)
		self._font = cv2.FONT_HERSHEY_SIMPLEX

	def _stab_log_data(self, timestamp, data, logconf):
		"""Callback froma the log API when data arrives"""
		print "[%d][%s]: %s" % (timestamp, logconf.name, data)

	def _stab_log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print "Error when logging %s: %s" % (logconf.name, msg)

	def run(self):

		# Init Crazyflier
		cflib.crtp.init_drivers(enable_debug_driver=False)
		print "Scanning interfaces for Crazyflies..."
		available = cflib.crtp.scan_interfaces()
		time.sleep(2) #Some USB bugfix
		print "Crazyflies found:"
		for i in available:
			print i[0]
		uri = available[-1][0]
		cf = Crazyflie()
		cf.open_link(uri)
		time.sleep(3) #TODO: use the callback instead
		cf.commander.send_setpoint(0, 0, 0, 0)

		# Init image dictionaries
		if self._cv:
			img = {}
			img_orig = {}
			img_blur = {}
			img_hsv = {}
			mask_hsv = {}
			mask_hsv_inv = {}
			img_masked = {}
			img_tinted = {}

		if self._log:
			self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=100)
			self._lg_stab.add_variable("stabilizer.pitch", "float")
			self._lg_stab.add_variable("stabilizer.yaw", "float")
			self._lg_stab.add_variable("stabilizer.roll", "float")

			try:
				cf.log.add_config(self._lg_stab)
				self._lg_stab.error_cb.add_callback(self._stab_log_error)
				self._lg_stab.start()

			except KeyError as e:
				print "Could not start log configuration," \
					"{} not found in TOC".format(str(e))
			except AttributeError:
				print "Could not add Stabilizer log config, bad configuration."
			self._lg_stab.data_received_cb.add_callback(self._stab_log_data)

		while True:
			# Read controlpanel
			h_value 	= cv2.getTrackbarPos('H-Value', 'Controls')
			h_range		= cv2.getTrackbarPos('H-Range', 'Controls')
			s_low 		= cv2.getTrackbarPos('S-Low', 'Controls')
			s_high 		= cv2.getTrackbarPos('S-High', 'Controls')
			v_low 		= cv2.getTrackbarPos('V-Low', 'Controls')
			v_high 		= cv2.getTrackbarPos('V-High', 'Controls')
			gauss		= cv2.getTrackbarPos('Gauss','Controls') * 2 + 1
			min_size	= cv2.getTrackbarPos('MinSize','Controls') + 1
			overlay 	= cv2.getTrackbarPos('Overlay','Controls')
			onoff 		= cv2.getTrackbarPos('ON/OFF','Controls')
			z_target	= cv2.getTrackbarPos('Height','Controls')
			show_overlay= cv2.getTrackbarPos('Overlay','Controls') == 1
			reset       = cv2.getTrackbarPos('Reset','Controls') == 1
			start 		= cv2.getTrackbarPos('Start','Controls') == 1
			land 		= cv2.getTrackbarPos('Land','Controls') == 1
			cancel 		= cv2.getTrackbarPos('Cancel','Controls') == 1
			command		= cv2.getTrackbarPos('Command','Controls') == 1
			cmdParam	= cv2.getTrackbarPos('Param','Controls')


			if start:
				cv2.setTrackbarPos('Start','Controls',0)
			if land:
				cv2.setTrackbarPos('Land','Controls',0)
			if cancel:
				cv2.setTrackbarPos('Cancel','Controls',0)
			if command:
				cv2.setTrackbarPos('Command','Controls',0)

			#default xyz
			sendx = 0
			sendy = 0
			sendz = 0

			# Camera vision code
			if self._cv:
				h_min = h_value - h_range
				h_max = h_value + h_range

				for i in self._camera_devices:
					self._camera[i].grab()

				for i in self._camera_devices:
					_, img_orig[i] = self._camera[i].retrieve()
					img_blur[i] = cv2.GaussianBlur(img_orig[i], (gauss,gauss), 0)
					img_hsv[i] = cv2.cvtColor(img_blur[i], cv2.COLOR_BGR2HSV_FULL)

					# Take care of region split for hue (red warps around 255)
					if h_min < 0 or h_max > 255:
						if h_min < 0:
							h_upper = h_min + 255
							h_lower = h_max

						elif h_max > 255:
							h_upper = h_min
							h_lower = h_max - 255

						mask_lower1 = np.array([h_upper, s_low, v_low],np.uint8)
						mask_upper1 = np.array([255, s_high, v_high],np.uint8)
						mask_hsv_lower = cv2.inRange(img_hsv[i], mask_lower1, mask_upper1)

						mask_lower2 = np.array([0, s_low, v_low],np.uint8)
						mask_upper2 = np.array([h_lower, s_high, v_high],np.uint8)
						mask_hsv_upper = cv2.inRange(img_hsv[i], mask_lower2, mask_upper2)

						mask_hsv[i] = cv2.bitwise_or(mask_hsv_lower, mask_hsv_upper)
					else:
						mask_lower = np.array([h_min, s_low, v_low],np.uint8)
						mask_upper = np.array([h_max, s_high, v_high],np.uint8)
						mask_hsv[i] = cv2.inRange(img_hsv[i], mask_lower, mask_upper)
					
					# close and open mask (remove small objects)
					kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(min_size,min_size))
					mask_hsv[i] = cv2.morphologyEx(mask_hsv[i], cv2.MORPH_CLOSE, kernel)
					mask_hsv[i] = cv2.morphologyEx(mask_hsv[i], cv2.MORPH_OPEN, kernel)

					img_masked[i] = cv2.bitwise_and(img_blur[i], img_blur[i], mask = mask_hsv[i])

					#IMPORTANT: This row will change the mask to 0-1 instead of 0-255 and also there will be some countour lines if you invert the mask
					_, contours, hierarchy = cv2.findContours(mask_hsv[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
					
					# Find largest contour (we might need to change this to a range)
					max_area = 0
					largest_contour = None
					for idx, contour in enumerate(contours):
						area = cv2.contourArea(contour)
						if area > max_area:
							max_area = area
							largest_contour = contour

					if not largest_contour == None:
						moment = cv2.moments(largest_contour)
						self._pos[i]  = cv2.minEnclosingCircle(largest_contour)
						found = True
					else:
						found = False

					#NOT NICE CODE!! DON'T LOOK HERE!!!
					if show_overlay:
						mask_hsv_inv[i] = 1 - mask_hsv[i] #Invert now for nice effect
						img_tinted[i] = cv2.addWeighted(img_orig[i], 0.2, self._overlay, 0.8, 0)
						cv2.bitwise_xor(img_tinted[i], img_tinted[i], img_tinted[i], mask = mask_hsv[i])
						cv2.bitwise_xor(img_orig[i], img_orig[i], img_orig[i], mask = mask_hsv_inv[i])
						img[i] = cv2.add(img_tinted[i], img_orig[i])
					else:
						img[i] = img_orig[i]
						x = int(self._pos[i][0][0])
						y = int(self._pos[i][0][1])
						r = int(self._pos[i][1])
						if found:
							cv2.circle(img[i], (x, y), r, (255, 0, 255), 2)
						


				#fix xyz if found otherwise turn the quad off for security reasons (if cv is disabled the quad will not be turned off)
				if found:
					x1 = self._pos[0][0][0]
					x2 = self._pos[1][0][0]
					y1 = self._pos[0][0][1]
					y2 = self._pos[1][0][1]

					B = 280
					D = x2 - x1
					f = self._clip
					z = f*B/D
					x = {}
					y = {}
					x[0] = (x1-self._w/2)*z/f
					y[0] = (self._h-y1)*z/f
					x[1] = (x2-self._w/2)*z/f
					y[1] = (self._h-y2)*z/f

					if (reset):
						self._z_orig = z
						self._x_orig = x[0]
						self._y_orig = y[0]



					sendx = int(x[0] - self._x_orig)
					sendy = int(z - self._z_orig)
					sendz = int(y[0] - self._y_orig)
				else:
					onoff = False;


# Flags from C-code
#define ON_FLAG			0x1
#define START_FLAG		0x02
#define LAND_FLAG		0x04
#define CANCEL_FLAG		0x08
#define COMMAND_FLAG	0x10
#define PARAM_FLAG		0xFF00

	 		#TODO: maybe change to custom send function (but since this is the exact type of arguments we want we can use it for now i guess)
	 		modeFlags = onoff | start << 1 | land << 2 | cancel << 3 | command << 4 | cmdParam << 8
	 		cf.commander.send_setpoint(sendx, sendy, sendz, modeFlags)

	 		if self._cv:
				for i in self._camera_devices:
					if not show_overlay and found:
						cv2.putText(img[i], 'XYZ:('+`sendx` + ';' + `sendy`+';'+`sendz`+')' , (10, self._h-15), self._font, 1, (0, 255, 255), 1, cv2.LINE_AA)
					cv2.imshow("Video" + `i`, img[i])

			if cv2.waitKey(1) == 27:
				cf.commander.send_setpoint(0, 0, 0, 0)
				time.sleep(0.1)
				cf.close_link()
				cv2.destroyAllWindows()
				for i in self._camera_devices:
					self._camera[i].release()
				break




if __name__ == '__main__':
	cflib.crtp.init_drivers(enable_debug_driver=False)
	program = CrazyTracker()
	program.run()