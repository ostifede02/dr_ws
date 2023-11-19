import cv2
import numpy as np
from filterpy.kalman import KalmanFilter

kf = KalmanFilter(4, 2)
kf.predict()

vid = cv2.VideoCapture("http://10.248.24.172:4747/video/")

# create the dictionary for markers type
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
distortion = np.array((-0.43948,0.18514,0,0))



def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray,(5,5),cv2.BORDER_DEFAULT)
    corners, marker_ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
        
    if len(corners) > 0:
        for i in range(0, len(marker_ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

    return frame



def main():

    while True:
        ret, frame = vid.read()

        frame = pose_estimation(frame, dictionary, intrinsic_camera, distortion)

        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break


    vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

