import gym
import pixelate_arena
import time
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import math
from collections import deque, namedtuple
import cv2.aruco as aruco

from color_identification import color_identification


def get_node_pairs(n1, n2, both_ends=True):
    if both_ends:
        node_pairs = [[n1, n2], [n2, n1]]
    else:
        node_pairs = [[n1, n2]]
    return node_pairs


class Dijkstra:
    def __init__(self, edges):
        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(

            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'

        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:

            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])

            if distances[current_vertex] == inf:
                break

            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost

                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

            vertices.remove(current_vertex)

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path, distances[dest]


Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
    return Edge(start, end, cost)


def move(initial_location, final_location):
    total_angle = math.atan2((final_location[1] - initial_location[1]), (final_location[0] - initial_location[0]))
    current_point = (ordered_2D_hexagons[initial_location[0]][initial_location[1]][0],
                     ordered_2D_hexagons[initial_location[0]][initial_location[1]][1])
    final_point = (ordered_2D_hexagons[final_location[0]][final_location[1]][0],
                   ordered_2D_hexagons[final_location[0]][final_location[1]][1])
    print(current_point, final_point)

    variable_rotational = None
    x = 0
    while True:
        if x % 10 == 0:
            image = env.camera_feed()
            image = np.ascontiguousarray(image, dtype=np.uint8)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            # print("corners - ", corners)
            if (ids is not None) and (corners is not None):
                variable_rotational = corners
                current_point = ((corners[0][0][0][0] + corners[0][0][2][0]) / 2,
                                 (corners[0][0][0][1] + corners[0][0][2][1]) / 2)
            else:
                corners = variable_rotational
                current_point = ((corners[0][0][0][0] + corners[0][0][2][0]) / 2,
                                 (corners[0][0][0][1] + corners[0][0][2][1]) / 2)

            husky_dir = math.atan2((corners[0][0][0][1] - corners[0][0][3][1]),
                                   (corners[0][0][0][0] - corners[0][0][3][0]))
            target_dir = math.atan2((final_point[1] - current_point[1]), (final_point[0] - current_point[0]))

        x += 1
        p.stepSimulation()
        speed = 7
        if math.fabs(husky_dir - target_dir) < 0.05:
            env.move_husky(0, 0, 0, 0)
            break
        else:
            if target_dir * husky_dir > 0:
                speed = (target_dir - husky_dir) * 15
                env.move_husky(speed, -speed, speed, -speed)
            else:
                if husky_dir < 0:
                    if 0 <= target_dir - husky_dir <= math.pi:
                        env.move_husky(speed, -speed, speed, -speed)
                    else:
                        env.move_husky(-speed, speed, -speed, speed)
                else:
                    if 0 >= target_dir - husky_dir >= -math.pi:
                        env.move_husky(-speed, speed, -speed, speed)
                    else:
                        env.move_husky(speed, -speed, speed, -speed)

    # translational motion
    variable_translation = None
    x = 0
    while True:
        if x % 15 == 0:
            img = env.camera_feed()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
            if (ids is not None) and (corners is not None):
                variable_translation = corners
                current_point = (
                    (corners[0][0][0][0] + corners[0][0][2][0]) / 2, (corners[0][0][0][1] + corners[0][0][2][1]) / 2)
            # print("center - ", center_husky)
            else:
                corners = variable_translation
                current_point = (
                    (corners[0][0][0][0] + corners[0][0][2][0]) / 2, (corners[0][0][0][1] + corners[0][0][2][1]) / 2)
            # print("center - ", center_husky)
        x += 1

        distance = math.sqrt((final_point[1] - current_point[1]) ** 2 + (final_point[0] - current_point[0]) ** 2)

        p.stepSimulation()
        if distance < 7:
            env.move_husky(0, 0, 0, 0)
            break
        else:
            if distance > 12:
                env.move_husky(12, 12, 12, 12)
            else:
                env.move_husky(distance / 2, distance / 2, distance / 2, distance / 2)


if __name__ == "__main__":
    env = gym.make("pixelate_arena-v0")
    env.remove_car()
    p.stepSimulation()
    img = env.camera_feed()
    img = np.ascontiguousarray(img, dtype=np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imwrite("image_for_color_identification.jpg", img)

    inf = float('inf')
    ordered_2D_hexagons, colorwise_list = color_identification(hsv)
    # print("colors_list - ", colorwise_list)
    value_of_colors = [1, 1, 4, 2, 3, 1, 200]  # white, red, green, yellow, purple, pink
    # Now putting (inf, inf) for black places
    for i in range(len(ordered_2D_hexagons)):
        j = 0
        while j < len(ordered_2D_hexagons[i]) - 1:
            if ordered_2D_hexagons[i][j] != (-1, -1):
                no_of_black_tiles = (ordered_2D_hexagons[i][j + 1][0] - ordered_2D_hexagons[i][j][0]) // 44 - 1
                for k in range(no_of_black_tiles):
                    ordered_2D_hexagons[i].insert(j + 1, (-1, -1))
            j += 1

    # Now we have all the hexagons along with their center coordinates
    # I will now convert these hexagons to a matrix of 13x26
    # obtained_13x26_matrix = obtain_matrix(ordered_2D_hexagons)
    for i in range(13):
        for j in range(int(math.fabs(6 - i))):
            ordered_2D_hexagons[i].insert(0, (-1, -1))
        if i <= 6:
            count = 0
            for j in range(6 + i):
                ordered_2D_hexagons[i].insert(7 - i + j + count, (-1, -1))
                count += 1
        else:
            count = 0
            for j in range(18 - i):
                ordered_2D_hexagons[i].insert(i + j + count - 5, (-1, -1))
                count += 1
        for j in range(int(math.fabs(6 - i))):
            ordered_2D_hexagons[i].append((-1, -1))

    obtained_13x26_matrix = ordered_2D_hexagons

    connections = []
    for i in range(len(obtained_13x26_matrix)):
        for j in range(len(obtained_13x26_matrix[i])):
            if obtained_13x26_matrix[i][j] != (-1, -1):
                neighbour_nodes = [(i, j - 2), (i, j + 2), (i - 1, j - 1), (i - 1, j + 1), (i + 1, j - 1),
                                   (i + 1, j + 1)]
                for node in neighbour_nodes:
                    try:
                        if obtained_13x26_matrix[node[0]][node[1]] != (-1, -1):
                            if obtained_13x26_matrix[node[0]][node[1]] in colorwise_list[6]:  # This is done
                                # separately because
                                connections.append(((i, j), node, value_of_colors[6]))  # were blue tile is present,
                                # 2 colours are there... so to only consider one colour
                            else:
                                for k in range(6):
                                    if obtained_13x26_matrix[node[0]][node[1]] in colorwise_list[k]:
                                        connections.append(((i, j), node, value_of_colors[k]))
                    except IndexError:
                        pass

    # print(connections)
    algorithm = Dijkstra(connections)
    env.respawn_car()
    p.stepSimulation()
    img = env.camera_feed()
    img = np.ascontiguousarray(img, dtype=np.uint8)
    cv2.imshow("img-", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    last_corr_rotational = None
    last_corr_translation = None
    center_husky = []
    initial_direction_corr = []

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Constant parameters used in Aruco methods
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    # Create grid board object we're using in our stream
    board = aruco.GridBoard_create(
        markersX=2,
        markersY=2,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)
    # Create vectors we'll be using for rotations and translations for postures
    rvecs, tvecs = None, None

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

    center_husky_x = (corners[0][0][0][0] + corners[0][0][2][0]) / 2
    center_husky_y = (corners[0][0][1][1] + corners[0][0][3][1]) / 2
    center_husky = [round(center_husky_x), round(center_husky_y)]

    final_path = []

    # NOW FINDING THE LOCATIONS OF THE RED TILES
    red_hexagons = []
    first_hexagon = None
    for hexagon in colorwise_list[1]:
        for i in range(len(obtained_13x26_matrix)):
            for j in range(len(obtained_13x26_matrix[i])):
                if obtained_13x26_matrix[i][j] == hexagon:
                    red_hexagons.append((i, j))
                    if -10 < hexagon[0] - center_husky[0] < 10:
                        first_hexagon = (i, j)

    print(red_hexagons)
    path_from_first_hexagon = []
    distance_from_first_hexagon = []
    for red_hexagon in red_hexagons:
        if red_hexagon != first_hexagon:
            path_from_first_hexagon.append(algorithm.dijkstra(first_hexagon, red_hexagon)[0])
            distance_from_first_hexagon.append(algorithm.dijkstra(first_hexagon, red_hexagon)[1])

    # print(path_from_first_hexagon)
    # print(distance_from_first_hexagon)

    if distance_from_first_hexagon[0] > distance_from_first_hexagon[1]:
        first_to_second = list(path_from_first_hexagon[1])
        second_hexagon = first_to_second[len(first_to_second) - 1]
    else:
        first_to_second = list(path_from_first_hexagon[0])
        second_hexagon = first_to_second[len(first_to_second) - 1]

    final_path += first_to_second
    current_location = final_path[0]
    # print("current - ", current_location)
    for i in range(1, len(final_path)):
        move(current_location, final_path[i])
        current_location = final_path[i]

    final_path = []
    red_hexagons.remove(first_hexagon)
    red_hexagons.remove(second_hexagon)
    third_hexagon = red_hexagons[0]

    second_to_third, _ = algorithm.dijkstra(second_hexagon, third_hexagon)
    # print(second_to_third)
    final_path += list(second_to_third)
    # print(final_path)

    stop_location = final_path[-1]
    current_location = final_path[0]
    for i in range(1, len(final_path)):
        move(current_location, final_path[i])
        current_location = final_path[i]

    final_path = []

    # unlocking the shapes in the pink tiles
    env.unlock_antidotes()
    img = env.camera_feed()
    img = np.ascontiguousarray(img, dtype=np.uint8)
    cv2.imwrite("unlocked_shape_location.jpg", img)
    pink_shapes = []
    shapes = []

    p.stepSimulation()
    img = env.camera_feed()
    img = np.ascontiguousarray(img, dtype=np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, np.array([60, 0, 0]), np.array([140, 255, 255]))

    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, blue_contours, -1, (0, 0, 255), 2)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    lst_area = []
    for contour in blue_contours:
        area = cv2.contourArea(contour)
        lst_area.append(area)
        M = cv2.moments(contour)
        if M['m00'] == 0:
            cx = int(M['m10'] / (M['m00'] + 0.0001))
            cy = int(M['m01'] / (M['m00'] + 0.0001))
        else:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        cv2.circle(img, (cx, cy), 3, (0, 255, 0), -1)

        for center in colorwise_list[5]:
            if math.fabs(center[0] - cx) < 5 and math.fabs(center[1] - cy) < 5:
                pink_shapes.append((center, area, len(contour)))
        for center in colorwise_list[4]:
            if math.fabs(center[0] - cx) < 5 and math.fabs(center[1] - cy) < 5:
                shapes.append([center, area, len(contour)])
        for center in colorwise_list[3]:
            if math.fabs(center[0] - cx) < 5 and math.fabs(center[1] - cy) < 5:
                shapes.append([center, area, len(contour)])
        for center in colorwise_list[2]:
            if math.fabs(center[0] - cx) < 5 and math.fabs(center[1] - cy) < 5:
                shapes.append([center, area, len(contour)])
    # print(pink_shapes)
    # print(shapes)

    # square shape
    square = []
    for i in pink_shapes:
        if i[2] == 4:
            square.append(i[0])
    for j in shapes:
        if j[2] == 4:
            if j[0] not in square:
                square.append(j[0])
            else:
                pass
    # print(square)

    # Circle shape
    circle = []
    for i in pink_shapes:
        if 35 < i[2] < 44:
            circle.append(i[0])
    for j in shapes:
        if 35 < j[2] < 44:
            if j[0] not in circle:
                circle.append(j[0])
    # print(circle)

    # triangle shape
    triangle = []
    for i in pink_shapes:
        if i[2] > 44:
            triangle.append(i[0])
    for j in shapes:
        if j[2] > 44:
            if j[0] not in triangle:
                triangle.append(j[0])
    # print(triangle)

    pink_hexagons = [triangle[0], square[0], circle[0]]
    new_pink = []
    shape_hexagons = [triangle[1], square[1], circle[1]]
    new_shapes = []
    for i in range(len(obtained_13x26_matrix)):
        for j in range(len(obtained_13x26_matrix[i])):
            for k in range(len(pink_hexagons)):
                if math.fabs(obtained_13x26_matrix[i][j][0] - pink_hexagons[k][0]) < 5 and math.fabs(
                        obtained_13x26_matrix[i][j][1] - pink_hexagons[k][1]) < 5:
                    new_pink.append([k, i, j])
            for k in range(len(shape_hexagons)):
                if math.fabs(obtained_13x26_matrix[i][j][0] - shape_hexagons[k][0]) < 5 and math.fabs(
                        obtained_13x26_matrix[i][j][1] - shape_hexagons[k][1]) < 5:
                    new_shapes.append([k, i, j])

    # print(new_pink)    # -- | order --> 0->triangle, 1->square, 2->circle
    # print(new_shapes)  # -- |
    new_pink = sorted(new_pink, key=lambda x: x[0])
    new_shapes = sorted(new_shapes, key=lambda x: x[0])
    for i in range(3):
        new_pink[i].pop(0)
        new_shapes[i].pop(0)
    # print(new_pink, new_shapes)

    final_order = []
    final_order += new_pink
    final_order += new_shapes
    # print(final_order)

    current_location = stop_location
    for i in range(len(final_order)):
        final_path += list(algorithm.dijkstra(current_location, tuple(final_order[i]))[0])[1:]
        current_location = tuple(final_order[i])
    # print(final_path)

    current_location = stop_location
    for i in range(0, len(final_path)):
        move(current_location, final_path[i])
        current_location = final_path[i]

    print("PATH COMPLETED")
