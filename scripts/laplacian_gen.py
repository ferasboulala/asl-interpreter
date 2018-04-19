import cv2

# Short script to convert raw rgb images to laplacian images for the CNN

def main():
	try:
		for i in range(65, 91):
			n = 0
			while True:
				in_filename = '../samples/training/rgb/raw/' + unichr(i).lower() + \
					'/' + str(n) + '.jpg'
				out_filename = '../samples/training/rgb/laplacian/' + unichr(i).lower() \
					+ '/' + str(n) + '.jpg'
				img = cv2.imread(in_filename, 0)
				if img is None:
					break
				laplacian = cv2.Laplacian(img, cv2.CV_8U)
				cv2.imwrite(out_filename, laplacian)
				n += 1

		for i in range(48, 58):
			n = 0
			while True:
				in_filename = '../samples/training/rgb/raw/' + unichr(i).lower() + \
					'/' + str(n) + '.jpg'
				out_filename = '../samples/training/rgb/laplacian/' + unichr(i).lower() \
					+ '/' + str(n) + '.jpg'
				img = cv2.imread(in_filename, 0)
				if img is None:
					break
				laplacian = cv2.Laplacian(img, cv2.CV_8U)
				cv2.imwrite(out_filename, laplacian)
				n += 1
	except KeyboardInterrupt:
		pass

if __name__ == '__main__':
	main()