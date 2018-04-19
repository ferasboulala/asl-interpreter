import cv2
import numpy as np

'''
Adaptive filter functions script
'''

def img_avg(img):
	try:
		return int(img.sum(0).sum(0)/(img > 0).sum(0).sum(0))
	except:
		return 0

'''
Gamma correction function:
	Low gamma := brighter image
	High gamma := darker image
	gamma = 1 := Unchanged
'''
def gamma(img, gamma=1):
	if gamma == 1:
		return img
	gamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** gamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	return cv2.LUT(cv2.medianBlur(img, 3), table)

'''
This function predicts the right correction according
to the average intensities of the pixels belonging to
the hand of the user. 
	desired_avg = 160
If presumes that the image has gone through a gamma
effect, making it either too bright or too dark. This
function will approximate that gamma by evaluating the
distance between the desired_avg and apply the correction
	correct_gamma = 1 / predicted_gamma
The correction is based on the difference between the
desired_avg and the avg. Here's the span:
	100 ---1/2.5--- 160 (1) ---2.5 --- 220
	predicted_gamma = pow(pow(2.5, 1/60), delta)
'''
def adaptive_filter(img):
	avg = img_avg(img)
	desired_avg = 160 # Can always be changed
	delta = avg - desired_avg
	predicted_gamma = 1

	if abs(delta) > 15:
		predicted_gamma = 1.015 ** delta

	return gamma(img, 1/predicted_gamma)