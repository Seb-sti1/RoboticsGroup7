import os.path

import cv2
import numpy as np
import pickle

"""
For the camera calibration, we need to find the intrinsic and extrinsic parameters of the camera.
"""


def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    non_working_ports = []
    dev_port = 0
    working_ports = []
    available_ports = []
    while len(non_working_ports) < 6:  # if there are more than 5 non working ports stop the testing.
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports.append(dev_port)
            print("Port %s is not working." % dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" % (dev_port, h, w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." % (dev_port, h, w))
                available_ports.append(dev_port)
        dev_port += 1
    return available_ports, working_ports, non_working_ports


def load_or_create_config():
    """
    This function will do the camera calibration and save the results to a file.
    :return: port, cameraMatrix
    """
    port, camera_matrix, color_lower, color_upper = None, None, None, None

    if os.path.exists('camera_calibration.pkl'):
        with open('camera_calibration.pkl', 'rb') as f:
            data = pickle.load(f)

            if len(data) > 0:
                port = data[0]
            if len(data) > 1:
                camera_matrix = data[1]
            if len(data) > 2:
                color_lower = data[2]
            if len(data) > 3:
                color_upper = data[3]

    if port is None or camera_matrix is None or color_lower is None or color_upper is None:
        print("No or partial calibration. Lets do it now!")
    else:
        return port, camera_matrix, color_lower, color_upper

    if port is None:
        _, a, _ = list_ports()
        print("Available ports: ", a)
        port = int(input("Which port is the camera connected to? "))

    if camera_matrix is None or color_lower is None or color_upper is None:
        cap = cv2.VideoCapture(port)

    if camera_matrix is None:
        print("Please enter the size of the chessboard in squares (width, height)")
        chessboardSize = (int(input("width:")), int(input("height:")))  # 9 6
        frameSize = None

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

        size_of_chessboard_squares_mm = float(input("Size of the chessboard squares in mm: "))  # 20.57
        objp = objp * size_of_chessboard_squares_mm

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        print("To quit press q")
        print("To take a image t")
        while True:

            ret, frame = cap.read()
            if frameSize is None:
                frameSize = frame.shape[:2]

            cv2.imshow('camera', frame)
            k = cv2.waitKey(5)
            if k == ord('q') or k == 27:
                break
            elif k == ord('t'):
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

                # If found, add object points, image points (after refining them)
                if ret:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                    # Draw and display the corners
                    cv2.drawChessboardCorners(frame, chessboardSize, corners2, ret)
                    cv2.imshow('detected chess', frame)

                    print("Add image to calibration? (y/n)")
                    s = cv2.waitKey()
                    if s == ord('y'):
                        print("Image added")
                        print("To quit press q")
                        print("To take a image t")
                        imgpoints.append(corners)
                        objpoints.append(objp)
                else:
                    print("chessboard not found")

        cv2.destroyAllWindows()

        ret, camera_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

        # Reprojection Error
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        print("Total error: {}".format(mean_error / len(objpoints)))

    if color_lower is None or color_upper is None:
        print("Let's choose the HSV values")
        # Create a window
        cv2.namedWindow('image')

        # Create trackbars for color change
        # Hue is from 0-179 for Opencv
        cv2.createTrackbar('HMin', 'image', 0, 179, lambda x: None)
        cv2.createTrackbar('SMin', 'image', 0, 255, lambda x: None)
        cv2.createTrackbar('VMin', 'image', 0, 255, lambda x: None)
        cv2.createTrackbar('HMax', 'image', 0, 179, lambda x: None)
        cv2.createTrackbar('SMax', 'image', 0, 255, lambda x: None)
        cv2.createTrackbar('VMax', 'image', 0, 255, lambda x: None)

        # Set default value for Max HSV trackbars
        cv2.setTrackbarPos('HMax', 'image', 179)
        cv2.setTrackbarPos('SMax', 'image', 255)
        cv2.setTrackbarPos('VMax', 'image', 255)

        # Initialize HSV min/max values
        hMin = sMin = vMin = hMax = sMax = vMax = 0
        phMin = psMin = pvMin = phMax = psMax = pvMax = 0

        while True:
            ret, frame = cap.read()

            hMin = cv2.getTrackbarPos('HMin', 'image')
            sMin = cv2.getTrackbarPos('SMin', 'image')
            vMin = cv2.getTrackbarPos('VMin', 'image')
            hMax = cv2.getTrackbarPos('HMax', 'image')
            sMax = cv2.getTrackbarPos('SMax', 'image')
            vMax = cv2.getTrackbarPos('VMax', 'image')

            # Set minimum and maximum HSV values to display
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            # Convert to HSV format and color threshold
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # Print if there is a change in HSV value
            if ((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax)):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
                hMin, sMin, vMin, hMax, sMax, vMax))
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display result image
            cv2.imshow('image', result)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        color_lower = (hMin, sMin, vMin)
        color_upper = (hMax, sMax, vMax)

        cv2.destroyAllWindows()

    print("Calibration done. Saving to file.")

    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump((port, camera_matrix, color_lower, color_upper), f)

    return port, camera_matrix, color_lower, color_upper


"""
Function to find the ellipse of the ring in the image.
"""
def perspective_transform(image, pitch_angle):
    # Assuming pitch_angle is in degrees

    # Get image dimensions
    height, width = image.shape[:2]

    # Convert pitch angle to radians
    pitch_angle_rad = np.radians(pitch_angle)
    # Define the source and destination points for perspective transform
    src_points = np.float32([[0, 0], [width - 1, 0], [0, height - 1], [width - 1, height - 1]])

    # Define the destination points based on the pitch angle
    dst_points = src_points.copy()

    # Adjust the destination points based on the pitch angle
    dst_points[[2, 3], 1] += np.tan(pitch_angle_rad) * (width / 2)

    # Calculate the perspective transform matrix
    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective transform
    transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, dst_points[2, 1].astype(np.int32)))

    return transformed_image


def filter_yellow(frame, color_lower, color_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_lower, color_upper)
    frame_mask = cv2.bitwise_and(frame, frame, mask=mask)
    ret, threshold = cv2.threshold(cv2.cvtColor(frame_mask, cv2.COLOR_BGR2GRAY), 125, 255, cv2.THRESH_BINARY)

    return threshold


def get_inside_and_outside_ellipses(threshold, contour_area_lower_threshold, contour_area_upper_threshold):
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ordered_contour = sorted(contours, key=cv2.contourArea, reverse=True)[:5]

    # find the
    outside_ellipse = None
    inside_ellipse = None
    for i, c in enumerate(ordered_contour):
        if len(c) < 5 \
                or cv2.contourArea(c) < contour_area_lower_threshold \
                or cv2.contourArea(c) > contour_area_upper_threshold:
            continue
        try:
            ellipse = cv2.fitEllipse(c)
            if i == 0:
                outside_ellipse = ellipse
            if i == 1:
                inside_ellipse = ellipse
        except AssertionError as e:
            pass

    return outside_ellipse, inside_ellipse


class Camera:
    def __init__(self, camera_matrix, color_lower, color_upper, port=0, debug=False):
        self.port = port
        self.camera = cv2.VideoCapture(port)
        self.debug = debug
        self.focal_length = (camera_matrix[0][0] + camera_matrix[1][1]) / 2
        self.center = camera_matrix[0][2], camera_matrix[1][2]
        self.color_lower = color_lower
        self.color_upper = color_upper

        cv2.namedWindow("camera")
        if debug:
            cv2.namedWindow("debug")

    def get_frame(self):
        frame = None

        if self.camera.isOpened():  # try to get the first frame
            rval, frame = self.camera.read()
        else:
            rval = False

        return frame if rval else None

    def get_ring_position(self, pitch_angle, ring_ext_diam=100, ring_int_diam=60,
                          contour_area_lower_threshold=1500, contour_area_upper_threshold=70000):
        """
        :param pitch_angle: IN DEGREES for now
        :param ring_ext_diam: exterior diameter of the ring in mm
        :param ring_int_diam: interior diameter of the ring in mm
        :param contour_area_lower_threshold:
        :param contour_area_upper_threshold:
        :return: ring position relative to the camera in meters, or None if no ring is found, or False if the user wants to quit
        """
        x, y, z = [0, 0, 0]

        frame = self.get_frame()
        if frame is None:
            return None

        compensated = perspective_transform(frame, pitch_angle)

        threshold = filter_yellow(compensated, self.color_lower, self.color_upper)
        outside_ellipse, inside_ellipse = get_inside_and_outside_ellipses(threshold,
                                                                          contour_area_lower_threshold,
                                                                          contour_area_upper_threshold)

        if outside_ellipse is not None and inside_ellipse is not None:
            cv2.ellipse(compensated, outside_ellipse, (0, 255, 0), 2)
            cv2.ellipse(compensated, inside_ellipse, (255, 0, 0), 2)
            # find the center of the ellipses
            ((inside_centx, inside_centy), (inside_width, inside_height), inside_angle) = inside_ellipse
            ((outside_centx, outside_centy), (outside_width, outside_height), outside_angle) = outside_ellipse

            # scale
            s_out = ring_ext_diam / outside_height
            s_ins = ring_int_diam / inside_height
            s_avg = (s_out + s_ins) / 2

            avg_centx = (inside_centx + outside_centx) / 2
            avg_centy = (inside_centy + outside_centy) / 2
            cv2.line(compensated, (int(self.center[0]), int(self.center[1])), (int(avg_centx), int(self.center[1])), (255, 0, 0), 1)
            cv2.line(compensated, (int(avg_centx), int(self.center[1])), (int(avg_centx), int(avg_centy)), (255, 0, 0), 1)

            x = s_avg * self.focal_length/1000
            y = (avg_centx - self.center[0]) * s_avg / 1000
            z = (self.center[1] - avg_centy) * s_avg / 1000
            cv2.putText(compensated, "x: %.2f, y: %.2f, z: %.2f" % (x, y, z),
                        (int(avg_centx), int(avg_centy) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("camera", compensated)
        if self.debug:
            cv2.imshow("debug", threshold)
        key = cv2.waitKey(5)
        if key == 27:  # exit on ESC
            return False

        return x, y, z

    def close(self):
        self.camera.release()
        if self.debug:
            cv2.destroyWindow("camera")
            cv2.destroyWindow("debug")


if __name__ == '__main__':
    port, camera_matrix, color_lower, color_upper = load_or_create_config()
    c = Camera(camera_matrix, color_lower, color_upper, port, true)

    while True:
        ring = c.get_ring_position(0)

        if ring is False:
            break
        elif ring is None:
            pass
        elif sum(ring) > 0:
            print(ring)

    c.close()
