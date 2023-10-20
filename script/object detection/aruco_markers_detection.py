import cv2 
import cv2.aruco as aruco


vid = cv2.VideoCapture(0) 

while(True): 
	
	ret, frame = vid.read() 
	aruco_dict = aruco.Dictionary(aruco.DICT_6X6_250)
	corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict)
	image_with_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

	for i in range(len(ids)):
		marker_id = ids[i][0]
		marker_corners = corners[i][0]
		print(f"marker_corners: {marker_corners}\tid: {marker_id}")

	cv2.imshow('frame', frame) 
	

	if cv2.waitKey(1) & 0xFF == ord('q'): 
		break

# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows() 
