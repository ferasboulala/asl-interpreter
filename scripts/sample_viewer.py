from matplotlib import pyplot as plt
import sys, os
from adaptive_filt import *

'''
This function displays the relevant images. It darkens an
image and corrects it. One can apply a median filter to the
image prior to the adaptive_filter() but the low pixel density
that the kinect provides will result in an important loss
of information.
'''
def display(img, figure):
	print('Original average : {0}'.format(img_avg(img)))

	dark = gamma(img, 0.3) #cv2.medianBlur(img, 3)
	print('Darker average : {0}'.format(img_avg(dark)))

	corrected = adaptive_filter(dark)
	print('Corrected average : {0}'.format(img_avg(corrected)))

	laplacian = cv2.Laplacian(img, cv2.CV_64F)
	sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
	sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)

	imgs = [img, dark, corrected, laplacian, sobelx, sobely]
	labels = ['Originale', 'Assombrie', 'Corrigee', 'Laplacien', 'Sobel-X', 'Sobel-Y']

	for i in range(len(imgs)):
		plt.subplot(2,3,i + 1), plt.imshow(imgs[i], cmap='gray')
		plt.title(labels[i]), plt.xticks([]), plt.yticks([])

	figure.canvas.draw()

'''
This function waits for a user input and returns the
character associated with it.
'''
def wait_key():
	result = None
	if os.name == 'nt':
		import msvcrt
		result = msvcrt.getch()
	else:
		import termios
		fd = sys.stdin.fileno()

		oldterm = termios.tcgetattr(fd)
		newattr = termios.tcgetattr(fd)
		newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(fd, termios.TCSANOW, newattr)

		try:
			result = sys.stdin.read(1)
		except IOError:
			pass
		finally:
			termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

	return result

def main():
	filename = '../samples/training/rgb/raw/'
	try:
		img = cv2.imread(filename + 'a/0.jpg', 0)
		letter = 'a'
		n = 0
		print('[SYMBOL] : Letter or number [SPACE] : Next image [ESC] : Quit')

		plt.ion() # Interactive mode
		figure = plt.figure()

		while True:
			option = ord(wait_key())
			f = filename

			if option == 27: # esc
				break
			elif option == 32: # space
				n += 100
				f += letter + '/' + str(n) + '.jpg'
			else:
				if option >= 97 and option < 123 or option < 59 and option > 47:
					n = 0
					letter = unichr(option).lower()
					f += letter + '/0.jpg'
				else :
					print('Cannot find the filename' + f)
					continue

			img = cv2.imread(f, 0)
			if img == None and option == 32: # reached max
				print('Reached the last image of this symbol at n = {0}'.format(n))
				n = 0
				continue
			else:
				display(img, figure)

	except KeyboardInterrupt:
		pass

	finally:
		print('Closing program')
		cv2.destroyAllWindows()
		plt.close()

if __name__ == '__main__':
	main()