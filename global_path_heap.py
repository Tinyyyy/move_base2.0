import math
import numpy as np
import heapq


def postpoints(p, img):
    return [(x, y) for x in range(p[0] - 1, p[0] + 2) for y in range(p[1] - 1, p[1] + 2) if
            img[x][y] == 0 and (x, y) != p]


def cal_global_path(map, start, end):
    img = map.copy()
    costmap = img.copy().astype(np.float64)
    costmap[:] = 0
    obstacle = list(zip(*np.where(img > 30)))
    radius = 20
    ob = np.ones([2 * radius + 1, 2 * radius + 1])
    if img[start]!=0:
        return []
    for i in range(0, radius + radius + 1):
        for j in range(0, radius + radius + 1):
            ob[i][j] = 5000 - math.sqrt((radius - i) ** 2 + (radius - j) ** 2)
    if img[end] != 0:
        print("goal is not reachable,choose the nearest free place instead!")
        free = np.array(list(zip(*np.where(img == 0))))
        ind = np.sum((free - end) ** 2, axis=1).argmin()
        end = tuple(free[ind].tolist())
    for p in obstacle:
        l1 = -min(0, p[0] - radius)
        h1 = radius + radius + 1 + min(img.shape[0] - (p[0] + radius + 1), 0)
        l2 = -min(0, p[1] - radius)
        h2 = radius + radius + 1 + min(img.shape[1] - (p[1] + radius + 1), 0)
        tmp = costmap[max(0, p[0] - radius):min(img.shape[0], p[0] + radius + 1),
              max(0, p[1] - radius):min(img.shape[1], p[1] + radius + 1)]
        costmap[max(0, p[0] - radius):min(img.shape[0], p[0] + radius + 1),
        max(0, p[1] - radius):min(img.shape[1], p[1] + radius + 1)] = np.vectorize(max)(ob[l1:h1, l2:h2], tmp)
    costmap[costmap == 0] = costmap[costmap > 0].min()
    costmap = (costmap - costmap.min()) / (costmap.max() - costmap.min()) * 10
    costmap = costmap ** 8
    costmap = (costmap - costmap.min()) / (costmap.max() - costmap.min()) * 10
    costmap[costmap == 0] = costmap[costmap != 0].min()
    d = img.copy().astype(np.float64)
    d[:] = 9999999
    d[start] = 0
    finished = img.copy()
    finished[:] = 0
    vis = []
    heapq.heappush(vis, (d[start], start))
    for ww in range(img.size):
        if len(vis) == 0:
            return []
        p = heapq.heappop(vis)[1]
        if p == end:
            break
        if finished[p] == 0:
            finished[p] = 1
            post = postpoints(p, img)
            for i in post:
                if d[p] + costmap[p] + costmap[i] < d[i]:
                    d[i] = d[p] + costmap[p] + costmap[i]
                    heapq.heappush(vis, (d[i], i))
    point = end
    path = [point]
    while point != start:
        post = postpoints(point, img)
        for i in post:
            if abs(d[i] + costmap[i] +costmap [point]- d[point]) < 1e-13:
                point = i
                path.append(point)
                break
    return path
