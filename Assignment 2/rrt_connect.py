import matplotlib.pyplot as plt
import numpy as np
import random
import math

# Function for making circles/obstacles
def circle(x,y,r,xl=[],yl=[]):
    xl.append([x + r * math.cos(np.deg2rad(d)) for d in deg])
    yl.append([y + r * math.sin(np.deg2rad(d)) for d in deg])
    
# Function which checks whether or not the given point lies inside or outside of the circle
def inside_circle(a,b,r,x,y):
    if( (a-x)*(a-x)+(b-y)*(b-y) <= r*r  ):
        return(True)
    else:
        return(False)

# Function which determines the coordinates of the highlighted path
def clear_path(path):
    new_path=[path[0]]
    i=0
    j=1
    
    while(i+2<len(path)):
        if(checkpath(path[i],path[j])==False):
            i=j-1
            new_path.append(path[j-1])
        elif(checkpath(path[i],path[j])==True):
            if( euclidean_distance(path[j-1],path[-1]) >= thresh_hold ):
                j+=1
            elif(euclidean_distance(path[j],path[-1]) < thresh_hold):
                new_path.append(path[-1])
                break
            
    n_x=[]
    n_y=[]
    for pt in new_path:
        n_x.append(pt[0])
        n_y.append(pt[1])
        
    return(n_x,n_y)

# Function which calculates the euclidean distance betwene two points    
def euclidean_distance(p,q):
    return( (p[0]-q[0])*(p[0]-q[0])+(p[1]-q[1])*(p[1]-q[1]) )

# Function which finds the angle between two points (when both the points are hypothetically joined to the origin)
def find_angle(p,q):
    hyp=q[1]-p[1]
    base=q[0]-p[0]
    theta=math.atan2(hyp,base)
    
    return(theta)

# Function which joins the P_near point of the tree with the random point, satisfying the threshold condition
def expand(p,q):
    if(euclidean_distance(p,q)>=thresh_hold):                   # If the distance between points P_near and P_new is greater than "Delta", then P_new would be
        theta=find_angle(p,q)                                   # a point which is at a distance of "Step Size" from point P_near along the direction of point P_new
        p_new=parametric_coordinates(p,theta)
    else:
        p_new=q
        
    return(p_new)

# Function which finds a point which is at a distance of "Step Size" from point P along the direction of point Q
def parametric_coordinates(p,theta):
    x=p[0]+(step*math.cos(theta))
    y=p[1]+(step*math.sin(theta))

    return([x,y])

# Function which find the nearest point of the tree with the random point
# Here, "level" depicts the nearest node from the corresponding node
def nearest(p_rand,tree):
    p_near=[-1,-1]
    temp=math.inf
    level=-1                                                    # As the root node has no nearest node so it is given [-1,-1] default value
    
    for i in range(len(tree)):                                  # This "for" loop tends to find the node in a tree which is at the minimum distance from the random point
        p_n=[tree[i][0],tree[i][1]]
        dist=euclidean_distance(p_rand,p_n)
        
        if(dist<temp):
            p_near=[tree[i][0],tree[i][1]]
            temp=dist
            level=i
            
    return(p_near,level)

# Function which adds all the nodes of the tree in a list [nodes's x_coordinate,nodes's y_coordinate]
def find_path(tree1,tree2):
    print("Path Found")
    
    path1=[]
    ni=tree1[-1][2]                                             # The level of the last node of tree1
    
    path1.append([tree1[-1][0],tree1[-1][1]])
    while(ni!=delimiter[0]):                                    # Adding all the points as long as the last node arrives or the tree ends
        path1.append([tree1[ni][0],tree1[ni][1]])
        ni=tree1[ni][2]
    path1.append([tree1[0][0],tree1[0][1]])

    # Same process for the other tree
    path2=[]                                                    # The level of the last node of tree1
    ni=tree2[-1][2]
    
    path2.append([tree2[-1][0],tree1[-1][1]])
    while(ni!=delimiter[0]):                                    # Adding all the points as long as the last node arrives or the tree ends
        path2.append([tree2[ni][0],tree2[ni][1]])
        ni=tree2[ni][2]
    path2.append([tree2[0][0],tree2[0][1]])
    
    return(path1,path2)

# Function which checks whether the path from point p to point q is fesiable or without any obstacle, also satisfying the threshold condition
def checkpath(p,q):
    if(euclidean_distance(p,q)<thresh_hold):                    # Returns true if the path is valid and also satisfies the threshold condition
        return(is_valid(p))
    
    theta=find_angle(p,q)
    while( (euclidean_distance(p,q) > thresh_hold) and (is_valid(p)) ):     # Recursively checks and finds the required valid point if any can be found or not
        p=parametric_coordinates(p,theta)
        
    return(is_valid(p))

# Function which checks whether a point is valid/feasible on the map or not
def is_valid(p):
    x=p[0]
    y=p[1]
    
    if( x<0 or x>=R or y<0 or y>=C or inside_circle(4.5,3,2,x,y) or inside_circle(3,12,2,x,y) or inside_circle(15,15,3,x,y) ):
        return(False)
    else:
        return(True)

# The function which performs the planning process of the bi-directional RRT
def bi_rrt():
    
    tree1=list()                                                # Tree with start point as root
    tree1.append(start+delimiter)
    
    tree2=list()                                                # Tree with end point as root
    tree2.append(goal+delimiter)
    
    flag=False              
    max_iters=5000                                              # max_iters will control how many times at max the random_node is being calculated which satisfies the thereshold (delta)
    count=0
    
    while(count<max_iters):
        
        p_rand=[random.randint(1,R-1),random.randint(1,C-1)]    # A feasible random point within the map
        p_near0,level0=nearest(p_rand,tree1)                    # This is the P_near point
        p_new0=expand(p_near0,p_rand)                           # Connecting the P_near and the random point, which will generate the P_new point

        if(is_valid(p_new0)==False):                            # If the P_new point is not fesiable or lying on the obstace, then increment the node count by 1
            count+=1
            if(len(tree2)<len(tree1)):                          # Checking for the smaller tree in order to add next new node in the smaller tree
                temp=tree1
                tree1=tree2
                tree2=temp
            continue
        else:
            tree1.append([p_new0[0],p_new0[1],level0])          # Adding the new node in the start tree as root (or goal tree as root) 
            
            p_near1,level1=nearest(p_new0,tree2)                # Now the previous new_node becomes the random node for the bigger tree, and the same process of finding the P_new node is applied here as well
            p_new1=expand(p_near1,p_new0)                       # Connecting the new P_near and the new random point, which will generate the new P_new point

            if(is_valid(p_new1)==False):                        # Same logic as above
                count+=1
                if(len(tree2)<len(tree1)):
                    temp=tree1
                    tree1=tree2
                    tree2=temp
                continue
            else:
                tree2.append([p_new1[0],p_new1[1],level1])      # Adding the new node in the goal tree as root (or start tree as root)

                # So far we have addend nodes in both the trees, now we will try to connect these two trees
                if(thresh_hold>=euclidean_distance(p_new1,p_new0)):
                    tree2.append([p_new0[0],p_new0[1],len(tree2)-1])    # As soon as the distance between two trees becomes <= Delta, we connect both the trees
                    flag=True                                           # by joining the points with a line. Also, as both the trees are not connected,
                    break                                               # we break the loop. Stoping expanding any further
                
    if(flag==False):                                            # If path is not found, print "No Path Found" else, returning both the trees
        print("Path not Found")
    else:
        return(tree1,tree2)
    
    
if __name__ == '__main__':

    # For making circles
    deg=list(range(0,360,2))
    deg.append(0)
    delimiter=[-1]                                              # A delimiter node

    # Defining the initial conditions
    R=30
    C=30
    start=[1,1]
    goal=[20,20]
    thresh_hold=1                                               # Delta Size
    step=1                                                      # Step Size

    # Making the MAP
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
    
    # Ploting the simulation
    if(is_valid(start)==False or is_valid(goal)==False):
        print("Path not found")
    else:
        tree1,tree2=bi_rrt()                                    # Path planning phase or making the bi-directional tree

        for pt in tree1:
            plt.scatter(pt[0],pt[1],color='red',s=10)           # Tree spanned as start position as root
            
        for pt in tree2:
            plt.scatter(pt[0],pt[1],color='blue',s=10)          # Tree spanned as goal position as root
            
        path1,path2=find_path(tree1,tree2)                      # Finding the optimal path

        if(path1[-1]!=start):                                   # Joining both the paths
            path=path2[::-1][:-1]+path1
        else:
            path=path1[::-1][:-1]+path2

        n_x,n_y=clear_path(path)                                # Finding the clear path
        plt.plot(n_x,n_y,color="green")                         # Plotting the optimal path

        x0=[]
        y0=[]
        for pt in path1:
            x0.append(pt[0])
            y0.append(pt[1])
        plt.plot(x0,y0,color="red")                             # Plotting the path of tree 1

        x1=[]
        y1=[]
        for pt in path2:                                        # Plotting the path of tree 2
            x1.append(pt[0])
            y1.append(pt[1])
        plt.plot(x1,y1,color="blue")

        plt.grid(True)
        plt.show()                                              # Plotting the complete solution
