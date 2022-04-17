import cv2
import numpy as np


def color_identification(hsv):
    img = cv2.imread("image_for_color_identification.jpg")

    white_mask = cv2.inRange(hsv, np.array([0, 0, 100]), np.array([0, 255, 255]))
    red_mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    green_mask = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([60, 255, 255]))
    yellow_mask = cv2.inRange(hsv, np.array([10, 100, 100]), np.array([30, 255, 255]))
    purple_mask = cv2.inRange(hsv, np.array([140, 0, 0]), np.array([150, 255, 255]))
    pink_mask = cv2.inRange(hsv, np.array([160, 0, 0]), np.array([170, 255, 255]))
    blue_mask = cv2.inRange(hsv, np.array([60, 0, 0]), np.array([140, 255, 255]))

    # COLOR_CONTOURS
    white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, white_contours, -1, (255, 0, 0), 3)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, red_contours, -1, (255, 0, 0), 3)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, green_contours, -1, (255, 0, 0), 3)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, yellow_contours, -1, (255, 0, 0), 3)
    purple_contours, _ = cv2.findContours(purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, purple_contours, -1, (255, 0, 0), 3)
    pink_contours, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, pink_contours, -1, (255, 0, 0), 3)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, blue_contours, -1, (255, 0, 0), 3)

    # cv2.imshow("image with contours", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Color-wise_list stores the complete contour points for each hexagon
    colorwise_list = [white_contours, red_contours, green_contours, yellow_contours, purple_contours, pink_contours,
                      blue_contours]
    colors_list = []
    for contours in colorwise_list:
        color_list = []
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] == 0:
                cx = int(M['m10'] / (M['m00'] + 0.0001))
                cy = int(M['m01'] / (M['m00'] + 0.0001))
            else:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)
            color_list.append((cx, cy))
        colors_list.append(color_list)
    # colors_list stores the centers of the hexagons in 2-D array colour-wise

    # Converting this 2-D array to 1-D array
    all_hexagons = colors_list[0] + colors_list[1] + colors_list[2] + colors_list[3] + colors_list[4] + \
                   colors_list[5] + colors_list[6]
    # Now removing multiple occurrences of a single location i.e. blue tiles
    all_hexagons = list(set(all_hexagons))
    all_hexagons = sorted(all_hexagons, key=lambda x: x[1])

    ordered_2D_hexagons = []
    start = 0
    for i in range(len(all_hexagons) - 1):  # made a 2d array on the basis of y-coordinate value
        if all_hexagons[i + 1][1] - all_hexagons[i][1] > 3:
            ordered_2D_hexagons.append(all_hexagons[start:i + 1])
            start = i + 1
        elif i + 1 == len(all_hexagons) - 1:
            ordered_2D_hexagons.append(all_hexagons[start:])

    for i in range(len(ordered_2D_hexagons)):
        ordered_2D_hexagons[i] = sorted(ordered_2D_hexagons[i], key=lambda x: x[0])
    print(colors_list)
    # print(ordered_2D_hexagons)
    return ordered_2D_hexagons, colors_list
