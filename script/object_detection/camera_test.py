import cv2 

vid = cv2.VideoCapture("http://10.248.24.172:4747/video/400x400") 

while(True): 
	
	ret, frame = vid.read()
	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'): 
		break

# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows() 
