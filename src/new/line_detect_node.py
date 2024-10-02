#!/usr/bin/env python3

import rospy
import cv2 as cv
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tfg.msg import LineDetect

class LineDetectNode():

    def __init__(self):
        """
        Inicia o nó para detecção da linha 
        """

        rospy.init_node('Line_Detection_Node', anonymous=True)

        self.bridge = CvBridge()
        
        self.hsv_values = [[0, 0, 0], [6, 255, 37]]

        rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)
        
        self.line_detect_msg = LineDetect()
        self.line_info_pub = rospy.Publisher('/tfg/line_info', LineDetect, queue_size=1)
        rospy.spin()

    def color_filter(self, cv_image):
        """
        Filtro de cor
        """

        lower = np.array(self.hsv_values[0])
        upper = np.array(self.hsv_values[1])
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV) 

        mask = cv.inRange(hsv, lower, upper)

        mask = cv.dilate(mask, np.ones((11, 11), np.uint8), iterations=1)
        mask = cv.erode(mask, np.ones((7, 7), np.uint8), iterations=1)

        # Morphological operations for further noise removal and closing gaps
        mask = cv.morphologyEx(mask, cv.MORPH_DILATE, np.ones((8, 8), np.uint8))

        return mask

    def find_draw_offset(self, cv_image):
        """
        Offset para desenhar melhor na imagem processada
        """

        altura, largura = cv_image.shape
        centro_x = largura // 2
        centro_y = altura // 2

        offset = (centro_x - largura // 2, centro_y - altura // 2)
        
        return offset
        
    def hough_lines(self):
        """
        Encontra as linhas retas na imagem e depois faz um fit para uma única reta,
        encontrnado a sua distância do centro da imagem e inclinação
        """

        draw_image = np.copy(self.cv_image)

        mask = self.color_filter(self.cv_image)
        
        canny_image = cv.Canny(mask, 50, 200, None, 3)

        offset = self.find_draw_offset(canny_image)

        # Função HoughLines para encontrar as linhas retas
        lines = cv.HoughLinesP(
            canny_image,
            rho=1,
            theta=np.pi / 180,
            threshold=70,
            minLineLength=25,
            maxLineGap=10,
        )

        angle = center_x = float('nan')
        
        if lines is not None and len(lines) > 0:
            points = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x1 += offset[0]
                y1 += offset[1]
                x2 += offset[0]
                y2 += offset[1]
                points.append([x1, y1])
                points.append([x2, y2])
            
            points = np.array(points, dtype= np.float32)

            [vx, vy, x, y] = cv.fitLine(points, cv.DIST_L2, 0, 0.01, 0.01)

            center_x = x

            angle = math.degrees(math.atan2(vy, vx))

            if angle <= 0:
                angle += 90.0
            else:
                angle -= 90

            # Draw the approximated line on the image
            m = 50
            cv.line(
                draw_image,
                (int(x - m * vx[0]), int(y - m * vy[0])),
                (int(x + m * vx[0]), int(y + m * vy[0])),
                (0, 255, 0),
                2,
            )
            cv.circle(draw_image, (int(x), int(y)), 2, (255, 0, 0), 3)
            cv.imshow("Line Detection", draw_image)

            center_x = center_x - 320 # 320 corrresponde ao meio da imagem em pixels

        if not math.isnan(center_x) and not math.isnan(angle):
            
            self.line_detect_msg.angle.data = angle
            self.line_detect_msg.center_x.data = center_x 
            
            # Publica os valores de center_x e angle no tópico /tfg/line_info
            self.line_info_pub.publish(self.line_detect_msg)

    def camera_callback(self, imagem):

      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(imagem, "bgr8")
      except CvBridgeError as e:
        print(e)

      if self.cv_image is not None:
         self.hough_lines()

      if cv.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Image shutdown")


if __name__ == "__main__":

    node = LineDetectNode()