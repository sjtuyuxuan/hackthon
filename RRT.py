import numpy as np
import cv2
from numpy import linalg


class Node(object):
    def __init__(self,item):
        self.item = item
        self.sons=[]
    def add_node (self,item):
        self.sons.append(Node(item))

def SearchTree(point, root):
    ans = [(np.linalg.norm(root.item - point), root)]
    if len(root.sons) > 0:
        for son in root.sons:
            ans.append(SearchTree(point, son))
        return min(ans)
    else:
        return np.linalg.norm(root.item - point), root

def PrintTree(root, points, size):
    points.append((int(root.item[0] * size),int(root.item[1] * size)))
    for son in root.sons:
        PrintTree(son, points, size)

def InObs(obs, point):
    for o in obs:
        if np.linalg.norm(point - o[0]) < o[1]:
            return True
    return False

def LineCrossObs(obs, point1, point2):
    for o in obs:
        dir1 = o[0] - point1
        dir2 = o[0] - point2
        cross = dir1[1] * dir2[0] - dir1[0] * dir2[1]
        dis = abs(cross / np.linalg.norm(point2 - point1))
        if dis < o[1]:
            return True
    return False

def main():
    # Q1
    np.random.seed(40)
    root = Node(np.array([50, 50]))
    list = []
    for index in range(500):
        random_point = np.random.rand(2) * 100
        dis, node_found = SearchTree(random_point, root)
        direction = (random_point - node_found.item) / np.linalg.norm((random_point - node_found.item))
        new_point = node_found.item + direction
        node_found.sons.append(Node(new_point))
    canvas = np.zeros((500, 500, 3), dtype="uint8")
    cv2.rectangle(canvas, (0, 0), (500, 500), (255, 255, 255), -1)
    points = []
    PrintTree(root, points, 5)
    for point in points:
        cv2.circle(canvas, point, 3, (255, 0, 0), -1)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(0)
    
    # Q2
    size_2 = 10   
    canvas = np.zeros((1000, 1000, 3), dtype="uint8")
    cv2.rectangle(canvas, (0, 0), (1000, 1000), (255, 255, 255), -1)
    obs = []
    for index in range(40):
        random_obs = np.random.rand(2) * 100
        random_radius = np.random.rand() * 8
        obs.append((random_obs, random_radius))
        cv2.circle(canvas, (int(random_obs[0] * size_2), int(random_obs[1] * size_2)),\
            int(random_radius * size_2), (0, 0, 0), -1)
    Goal = np.array([-1, -1])
    Start = np.array([-1, -1])
    while Goal[0] < 0:
        random_goal = np.random.rand(2) * 100
        if not InObs(obs, random_goal):
            Goal = random_goal
    while Start[0] < 0:
        random_start = np.random.rand(2) * 100
        if not InObs(obs, random_start):
            Start = random_start
    root2 = Node(Start)
    cv2.circle(canvas, (int(Goal[0] * size_2), int(Goal[1] * size_2)),\
        size_2, (0, 255, 0), -1)
    cv2.circle(canvas, (int(Start[0] * size_2), int(Start[1] * size_2)),\
        size_2, (0, 0, 255), -1)
    while True:
        random_point = np.random.rand(2) * 100
        dis, node_found = SearchTree(random_point, root2)
        # if LineCrossObs(obs, node_found.item, random_point):
        #     continue
        direction = (random_point - node_found.item) / np.linalg.norm((random_point - node_found.item))
        new_point = node_found.item + direction
        if InObs(obs, new_point):
            continue
        node_found.sons.append(Node(new_point))
        if not LineCrossObs(obs, Goal, new_point):
            cv2.line(canvas, (int(Goal[0] * size_2), int(Goal[1] * size_2)),\
                (int(new_point[0] * size_2), int(new_point[1] * size_2)), (0, 255, 0), 3)
            break
    points = []
    PrintTree(root2, points, size_2)
    for point in points:
        cv2.circle(canvas, point, 6, (255, 0, 0), -1)
    cv2.imshow("Canvas", canvas)
    cv2.waitKey(0)



if __name__ == '__main__':
    main()