'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			eYRC#BM#2586
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_1a.py
# Functions:		detect_shapes
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################





##############################################################

def detect_shapes(img):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list
	containing details of colored (non-white) shapes in that image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	Returns:
	---
	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image
	
	Example call:
	---
	shapes = detect_shapes(img)
	"""    
	detected_shapes = []

	##############	ADD YOUR CODE HERE	##############
	def detect_shapes(img):
		detected_shapes=[]
                blue=np.where((img == [255,0,0]).all(axis = 2))
                green=np.where((img == [0,255,0]).all(axis =  2))
                red=np.where((img == [0,0,255]).all(axis = 2))
                orange=np.where((img==[0,165,255]).all(axis=2))
                img1gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                ret, thresh = cv2.threshold(img1gray,180,255,cv2.THRESH_BINARY)
                cv2.imshow("thresholded image",thresh)
                cont , hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                for i in range(1,len(cont)):
			Cent = cv2.moments(cont[i])
                        cx =int(Cent["m10"]/Cent["m00"])
                        cy =int(Cent["m01"]/Cent["m00"])
                        if cy in blue[0] and cx in blue[1]:
				color='Blue'
                        elif cy in green[0] and cx in green[1]:
				color='Green'
                        elif cy in red[0] and cx in red[1]:
				color='Red'
                        elif cy in orange[0] and cx in orange[1]:
                                color='Orange'
                        app = cv2.approxPolyDP(cont[i],0.01*cv2.arcLength(cont[i],True),True)
                        if len(app) == 3:
				shape = 'Triangle'
			elif len(app)==4:
				(x,y,a,b)=cv2.boundingRect(app)
				if(a/b==1):
					shape='Square'
				else:
					shape = 'Rectangle'
			elif len(app)==5:
				shape = 'Pentagon'
			elif len(app)==6:
				shape = 'Hexagon'
			else:
				shape ='Circle'
			li1=[color,shape,(cx,cy)]
                        detected_shapes.append(li1)
		return detected_shapes

	
	 


	##################################################
	
	return detected_shapes

def get_labeled_image(img, detected_shapes):
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the image and the detected shapes list as an argument
	and returns a labelled image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image

	Returns:
	---
	`img` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	img = get_labeled_image(img, detected_shapes)
	"""
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for detected in detected_shapes:
		colour = detected[0]
		shape = detected[1]
		coordinates = detected[2]
		cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
	return img

if __name__ == '__main__':
	
	# path directory of images in 'test_images' folder
	img_dir_path = 'test_images/'

	# path to 'test_image_1.png' image file
	file_num = 1
	img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
	
	# read image using opencv
	img = cv2.imread(img_file_path)
	
	print('\n============================================')
	print('\nFor test_image_' + str(file_num) + '.png')
	
	# detect shape properties from image
	detected_shapes = detect_shapes(img)
	print(detected_shapes)
	
	# display image with labeled shapes
	img = get_labeled_image(img, detected_shapes)
	cv2.imshow("labeled_image", img)
	cv2.waitKey(2000)
	cv2.destroyAllWindows()
	
	choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
	
	if choice == 'y':

		for file_num in range(1, 15):
			
			# path to test image file
			img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
			
			# read image using opencv
			img = cv2.imread(img_file_path)
	
			print('\n============================================')
			print('\nFor test_image_' + str(file_num) + '.png')
			
			# detect shape properties from image
			detected_shapes = detect_shapes(img)
			print(detected_shapes)
			
			# display image with labeled shapes
			img = get_labeled_image(img, detected_shapes)
			cv2.imshow("labeled_image", img)
			cv2.waitKey(2000)
			cv2.destroyAllWindows()


