import cv2
import numpy as np
import math as ma

from src.HSVPicker import pick_hsv


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


class Camera:
    def __init__(self, tip_to_camera_tf, port=0, show=True, ):
        self.port = port
        self.camera = cv2.VideoCapture(port)
        self.show = show
        self.tip_to_camera_frame = tip_to_camera_tf

        if show:
            cv2.namedWindow("camera")
            cv2.namedWindow("debug")
            cv2.namedWindow("debug1")
            cv2.namedWindow("debug2")

    def get_frame(self):
        frame = None

        if self.camera.isOpened():  # try to get the first frame
            rval, frame = self.camera.read()
        else:
            rval = False

        return frame if rval else None

    def get_ring_position(self, contour_area_lower_threshold=1500, contour_area_upper_threshold=60000):
        frame = self.get_frame()
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (14, 45, 140), (50, 130, 255))
        frame_mask = cv2.bitwise_and(frame, frame, mask=mask)
        ret, threshold = cv2.threshold(cv2.cvtColor(frame_mask, cv2.COLOR_BGR2GRAY), 125, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        ordered_contour = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        print(sorted([cv2.contourArea(c) for c in contours])[::-1][:5])

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

        if outside_ellipse is not None and inside_ellipse is not None:
            cv2.ellipse(frame, outside_ellipse, (0, 255, 0), 2)
            cv2.ellipse(frame, inside_ellipse, (255, 0, 0), 2)

        if self.show:
            cv2.imshow("camera", frame)
            cv2.imshow("debug", frame_mask)
            cv2.imshow("debug", threshold)
            key = cv2.waitKey(5)
            if key == 27:  # exit on ESC
                return False

        return [0, 0, 0]

    def close(self):
        self.camera.release()
        if self.show:
            cv2.destroyWindow("camera")
            cv2.destroyWindow("debug")
            cv2.destroyWindow("debug2")
            cv2.destroyWindow("debug3")


if __name__ == '__main__':
    c = Camera(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    Theta = -30 / 180 * ma.pi
    Dist = 0.25
    T04 = np.array([[ma.cos(Theta), - ma.sin(Theta), 0, Dist * ma.cos(Theta)],
                    [ma.sin(Theta), ma.cos(Theta), 0, Dist * ma.sin(Theta)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    while True:
        ring = c.get_ring_position()

        if ring is False:
            break

    c.close()
