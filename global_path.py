import sys
from PIL import Image
import math
import numpy as np
import time

def postpoints(p,img):
    return [(x,y) for x in range(p[0]-1,p[0]+2) for y in range(p[1]-1,p[1]+2) if img[x][y]==0 and (x,y)!=p]
    # l=[(p[0] - 1, p[1]), (p[0] + 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)]
    # return [a for a in l if img[a]==254]

def cal_global_path(map,start,end):
    s1=time.time()
    kernel=np.array([[1,1,1],[1,1,1],[1,1,1]]).astype(np.float)/9.0

    img=map.copy()
    # img=signal.convolve2d(img,kernel,boundary='symm',mode='same')
    costmap=img.copy().astype(np.float)
    costmap[:]=0
    obstacle=list(zip(*np.where(img>30)))
    radius=20
    ob=np.ones([2*radius+1,2*radius+1])
    for i in range(0,radius + radius+1):
        for j in range(0,radius + radius+1):
            ob[i][j]=5000-math.sqrt((radius-i)**2+(radius-j)**2)
    s2=time.time()
    # print(img[end],end)
    if img[end]!=0:
        print("goal is not reachable,choose the nearest free place instead!")
        free=np.array(list(zip(*np.where(img==0))))
        ind=np.sum((free-end)**2,axis=1).argmin()
        end=tuple(free[ind].tolist())
    # print(end)
    for p in obstacle:
        l1=-min(0,p[0]-radius)
        h1=radius+radius+1+min(img.shape[0]-(p[0]+radius+1),0)
        l2 = -min(0, p[1] - radius)
        h2 = radius + radius + 1 + min(img.shape[1] - (p[1] + radius + 1), 0)
        tmp=costmap[max(0,p[0]-radius):min(img.shape[0],p[0]+radius+1),max(0,p[1]-radius):min(img.shape[1],p[1]+radius+1)]
        costmap[max(0,p[0]-radius):min(img.shape[0],p[0]+radius+1),max(0,p[1]-radius):min(img.shape[1],p[1]+radius+1)]=np.vectorize(max)(ob[l1:h1,l2:h2],tmp)
    s3=time.time()
    costmap[costmap==0]=costmap[costmap>0].min()
    costmap=(costmap-costmap.min())/(costmap.max()-costmap.min()) *10
    costmap=costmap**8
    costmap=(costmap-costmap.min())/(costmap.max()-costmap.min()) *10
    costmap[costmap==0]=costmap[costmap!=0].min()
    s4=time.time()
    # print(s4-s1,"s4 ")

    d=img.copy().astype(np.float)
    d[:]=9999999
    d[start]=0
    vis=[start]
    for ww in range(img.size):
        if len(vis)==0:
            return []
        tu=tuple(zip(*vis))
        min_ind=d[tu].argmin()
        p=vis[min_ind]
        del vis[min_ind]
        if p==end:
            break

        post=postpoints(p,img)
        for i in post:
            # if d[p]+math.sqrt((p[0]-i[0])**2+(p[1]-i[1])**2)+costmap[p]<d[i]:
            #     d[i]=d[p]+math.sqrt((p[0]-i[0])**2+(p[1]-i[1])**2)+costmap[p]
            #     vis[i]=0
            if d[p]+costmap[p]<d[i]:
                d[i]=d[p]+costmap[p]
                vis.append(i)

    s5 = time.time()
    # print(s5-s4, "s5 ")
    point=end
    path=[point]

    while point!=start:
        post=postpoints(point,img)
        for i in post:
            # if abs(d[i]+math.sqrt((point[0]-i[0])**2+(point[1]-i[1])**2)+costmap[i]-d[point])<0.01:
            if abs(d[i] + costmap[i] - d[point]) < 1e-13:
                point=i

                path.append(point)
                break

    s6 = time.time()
    # print(s6-s5, "s6 ")
    return path
