import matplotlib.pyplot as plt
import numpy as np
import random
import math

def circle(x,y,r,xl=[],yl=[]):
    xl.append([x + r * math.cos(np.deg2rad(d)) for d in deg])
    yl.append([y + r * math.sin(np.deg2rad(d)) for d in deg])

def inside_circle(a,b,r,x,y):
    if( (a-x)*(a-x)+(b-y)*(b-y) <= r*r  ):
        return(True)
    else:
        return(False)

def is_valid(p):
    x=p[0]
    y=p[1]
    
    if( x>=R or y>=C or inside_circle(4.5,3,2,x,y) or inside_circle(3,12,2,x,y) or inside_circle(15,15,3,x,y) ):
        return(False)
    else:
        return(True)
    
def euclidean_distance(p,q):
    return( (p[0]-q[0])*(p[0]-q[0])+(p[1]-q[1])*(p[1]-q[1]) )

def find_angle(p,q):
    hyp=q[1]-p[1]
    base=q[0]-p[0]
    theta=math.atan2(hyp,base)
    
    return(theta)

def nearest(p_rand,tree):
    p_near=[-1,-1]
    temp=math.inf
    branch=-1
    
    for i in range(len(tree)):
        p_n=[tree[i][0],tree[i][1]]
        dist=euclidean_distance(p_rand,p_n)
        
        if(dist<temp):
            p_near=[tree[i][0],tree[i][1]]
            temp=dist
            branch=i
            
    return(p_near,branch)

def extend(p,q):
    if(euclidean_distance(p,q)>=thresh_hold):
        theta=find_angle(p,q)
        new_point=parametric_coordinates(p,theta)
    else:
        new_point=q
        
    return(new_point)

def parametric_coordinates(p,theta):
    x=p[0]+(step*math.cos(theta))
    y=p[1]+(step*math.sin(theta))

    return([x,y])

def find_path(tree0,tree1):
    print("Path Found")
    path0=[]
    ni=tree0[-1][2]
    path0.append([tree0[-1][0],tree0[-1][1]])
    while(ni!=delimiter[0]):
        path0.append([tree0[ni][0],tree0[ni][1]])
        ni=tree0[ni][2]
    path0.append([tree0[0][0],tree0[0][1]])

    path1=[]
    ni=tree1[-1][2]
    path1.append([tree1[-1][0],tree0[-1][1]])
    while(ni!=delimiter[0]):
        path1.append([tree1[ni][0],tree1[ni][1]])
        ni=tree1[ni][2]
    path1.append([tree1[0][0],tree1[0][1]])
    
    return(path0,path1)

def checkpath(p,q):
    if(euclidean_distance(p,q) < thresh_hold):
        return(is_valid(p))
    
    theta=find_angle(p,q)
    while( (euclidean_distance(p,q) > thresh_hold) and (is_valid(p)) ):
        p=parametric_coordinates(p,theta)
        
    return(is_valid(p))












def pruning(path):
    new_path=[path[0]]
    i=0
    j=1
    
    while(i+2<len(path)):
        if(checkpath(path[i],path[j])):
            if(euclidean_distance(path[j], path[-1]) < thresh_hold):
                new_path.append(path[-1])
                break
            j+=1
        else:
            new_path.append(path[j-1])
            i=j-1

    n_x=[]
    n_y=[]
    for pt in new_path:
        n_x.append(pt[0])
        n_y.append(pt[1])
        
    return(n_x,n_y)

def rrt_connect():
    
    tree0 = list()
    tree0.append(start+delimiter)
    
    tree1 = list()
    tree1.append(goal + delimiter)
    
    flag = False
    max_iters = 5000
    count = 0
    
    while(count < max_iters):
        
        p_rand = [random.randint(1,R-1),random.randint(1,C-1)]
        p_near0,row0 = nearest(p_rand,tree0)  
        p_new0 = extend(p_near0,p_rand)  

        if(is_valid(p_new0)==False):
            count+=1
            if(len(tree1)<len(tree0)):
                temp=tree0
                tree0=tree1
                tree1=temp
            continue
        else:
            tree0.append([p_new0[0],p_new0[1],row0])
            
            p_near1,row1 = nearest(p_new0,tree1)  
            p_new1 = extend(p_near1,p_new0) 

            if(is_valid(p_new1)==False):
                count+=1
                if(len(tree1)<len(tree0)):
                    temp=tree0
                    tree0=tree1
                    tree1=temp
                continue
            else:
                tree1.append([p_new1[0], p_new1[1], row1])
                p_new_new1 = extend(p_new1, p_new0)  # p_new1朝p_new0生长

                if not is_valid(p_new_new1):
                    count += 1
                    if len(tree1) < len(tree0):
                        tree0, tree1 = tree1, tree0
                    continue
                else:
                    tree1.append([p_new_new1[0], p_new_new1[1], len(tree1)-1])
                    p_new1 = p_new_new1

                    while euclidean_distance(p_new1, p_new0) > thresh_hold:
                        p_new_new1 = extend(p_new1, p_new0)  # p_new1朝p_new0生长
                        if not is_valid(p_new_new1):
                            count += 1
                            break
                        else:
                            tree1.append([p_new_new1[0], p_new_new1[1], len(tree1)-1])
                            p_new1 = p_new_new1

                    if euclidean_distance(p_new1, p_new0) <= thresh_hold:
                        tree1.append([p_new0[0], p_new0[1], len(tree1)-1])
                        flag=True
                        break

            if(len(tree1)<len(tree0)):
                temp=tree0
                tree0=tree1
                tree1=temp
                
    if(flag==False):
        print("Path not Found")
    else:
        return(tree0,tree1)

    
if __name__ == '__main__':

    R = 30
    C = 30

    delimiter = [-1]

    deg = list(range(0, 360, 2))
    deg.append(0)
    
    start = [1, 1]
    goal = [20, 20]
    step = 1
    thresh_hold = 1
    
    fig,ax=plt.subplots()

    # Obstacle 1
    xl=[]
    yl=[]
    circle(4.5,3,2,xl,yl)
    ax.fill(xl[0],yl[0],"-k",zorder=5)

    # Obstacle 2
    xl=[]
    yl=[]
    circle(3,12,2,xl,yl)
    ax.fill(xl[0],yl[0],"-k",zorder=5)

    # Obstacle 3
    xl=[]
    yl=[]
    circle(15,15,3,xl,yl)
    ax.fill(xl[0],yl[0],"-k",zorder=5)
    plt.axis([0,30,0,30])

    if(is_valid(start)==False or is_valid(goal)==False):
        print("Path not found")
    else:
        tree0,tree1=rrt_connect()

        for pt in tree0:
            plt.scatter(pt[0],pt[1],color='lightcoral',s=5)
            
        for pt in tree1:
            plt.scatter(pt[0],pt[1],color='lightsteelblue',s=5)
            
        path0,path1=find_path(tree0,tree1)

        if(path0[-1]==start):
            path=path0[::-1][:-1]+path1
        else:
            path=path1[::-1][:-1]+path0

        n_x,n_y=pruning(path)
        plt.plot(n_x,n_y,color="green")

        x0=[]
        y0=[]
        for pt in path0:
            x0.append(pt[0])
            y0.append(pt[1])
        plt.plot(x0,y0,color="darkred")

        x1=[]
        y1=[]
        for pt in path1:
            x1.append(pt[0])
            y1.append(pt[1])
        plt.plot(x1,y1,color="darkblue")

        plt.grid(True)
        plt.show()
