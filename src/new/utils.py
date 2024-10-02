import math
import cv2 as cv
import numpy as np
from geopy.point import Point
from geopy.distance import geodesic

def zoom(cv_image, scale):

    altura, largura, _ = cv_image.shape

    # Definir o tamanho do quadrado que deseja cortar
    tamanho_quadrado = scale  # Defina o tamanho do quadrado aqui

    # Calcular as coordenadas do centro
    centro_x = largura // 2
    centro_y = altura // 2

    # Calcular as coordenadas do corte
    inicio_x = centro_x - tamanho_quadrado // 2
    fim_x = centro_x + tamanho_quadrado // 2
    inicio_y = centro_y - tamanho_quadrado // 2
    fim_y = centro_y + tamanho_quadrado // 2

    # Fazer o corte da imagem
    corte = cv_image[inicio_y:fim_y, inicio_x:fim_x]

    return corte

def color_filter(cv_image, 
                 hsv_values= [[0, 0, 0], [6, 255, 37]]):

    lower = np.array(hsv_values[0])
    upper = np.array(hsv_values[1])
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV) 

    mask = cv.inRange(hsv, lower, upper)

    mask = cv.dilate(mask, np.ones((11, 11), np.uint8), iterations=1)
    mask = cv.erode(mask, np.ones((7, 7), np.uint8), iterations=1)

        # Morphological operations for further noise removal and closing gaps
    mask = cv.morphologyEx(mask, cv.MORPH_DILATE, np.ones((8, 8), np.uint8))

    return mask

def find_draw_offset(cv_image):

    altura, largura = cv_image.shape
    centro_x = largura // 2
    centro_y = altura // 2

    offset = (centro_x - largura // 2, centro_y - altura // 2)
    
    return offset

def hough_lines(src):

    draw_image = np.copy(src)

    mask = color_filter(src)
    
    canny_image = cv.Canny(mask, 50, 200, None, 3)

    offset = find_draw_offset(canny_image)

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
        cv.imshow("asd", draw_image)

        center_x = center_x - 320
 
    return center_x, angle

def calc_next_heading(gps1, gps2):
    """
    Calculate heading between two GPS Points
    """
    gps1 = Point(gps1)
    gps2 = Point(gps2)

    delta_lat = math.radians(gps2.latitude - gps1.latitude)
    delta_long = math.radians(gps2.longitude - gps1.longitude)

    angle_rad = math.atan2(delta_long, delta_lat)
    angle_deg = math.degrees(angle_rad)
        
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg

def gps_dist(gps1, gps2):
    """
    Calculate the distance between two GPS Points
    """
    gps1 = Point(gps1)
    gps2 = Point(gps2)

    return geodesic(gps1,gps2).meters
