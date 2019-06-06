import cv2
import numpy as np
import math
import time
import matplotlib.pyplot as plt
########################################################################################################################
# Helper functions
# Class structures
class Node:
    def __init__(self):
        self.is_obstacle = False
        self.parent = None
        self.neighbours = {}
        self.on_path = False
        self.g_cost = 999999  # should be infinite
        self.rhs = 999999  # should be infinite

class Graph:
    # graph constructor that creates an empty dictionary
    # nodes = {(x,y):Node} where x,y are coordinates of node
    # open_list = {(x,y): key}
    def __init__(self):
        self.nodes = {}
        self.open_list = {}
        self.obstacle_space = set()
        self.current_path = []
        self.open_length = 0
        self.new_open_length = 0
        self.new_itr = 1
        self.x_replan = []
        self.y_replan = []
    # loop through image and create node object for each pixel
    def create_nodes(self):
        for x in range(-555, 555):
            for y in range(-505, 505):
                y = -y
                if (x,y) not in self.obstacle_space:
                    self.nodes[(x, y)] = Node()

    # for given pixel and find it's neighbours
    def calculate_neighbours(self, curr_node):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1.41
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -555 and y+1 < 505:
            if (x-1,y+1) not in self.open_list and self.nodes[(x-1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 505:
            if (x,y+1) not in self.open_list and self.nodes[(x,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 555 and y+1 < 505:
            if (x+1,y+1) not in self.open_list and self.nodes[(x+1,y+1)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -555 and y-1 >= -505:
            if (x-1, y-1) not in self.open_list and self.nodes[(x-1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -505:
            if (x, y-1) not in self.open_list and self.nodes[(x,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 555 and y-1 >= -505:
            if (x+1,y-1) not in self.open_list and self.nodes[(x+1,y-1)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -555:
            if (x-1,y) not in self.open_list and self.nodes[(x-1,y)].rhs == 999999:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 555:
            if (x+1,y) not in self.open_list and self.nodes[(x+1,y)].rhs == 999999:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    # Check if node is consistent
    def node_is_consistent(self,node):
        if self.nodes[node].g_cost == self.nodes[node].rhs:
            return True
        else:
            return False
    # Get heuristic distance
    def h(self,node,start):
        return np.sqrt((node[0] - start[0])**2 + (node[1] - start[1])**2)

    # Get key for sorting the open_list:
    def get_key(self,node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + self.h(node,start)
        return key

    # Get smallest element from the open_list:
    def get_smallest(self,open_list):
        smallest = 9999999;
        smallest_node = (-9999,-9999)
        for key, value in open_list.items():
            if open_list[key] < smallest:
                smallest = value
                smallest_node = key
        return smallest_node

    # D* Lite algorithm to find the shortest path
    def d_star_lite_algo(self, rob_x, rob_y, goal_x, goal_y,bg):
        x_plot = []
        y_plot = []
        bg[505 - rob_y, rob_x + 555] = (250,11,156)
        bg[505 - goal_y, goal_x + 555] = (250, 11, 156)
        ## Color declarations
        # define color for node in open list
        green = (60, 179, 113)
        # define color for the current node
        red = (0, 0, 250)
        # get coordinates for the start node
        start_node = (rob_x, rob_y)
        # get coordinates for the goal node
        goal_node = (goal_x, goal_y)
        # make cost of start node zero
        self.nodes[goal_node].rhs = 0
        self.open_list[goal_node] = 0
        self.open_length += 1
        curr_node = goal_node
        iteration = 1
        while not curr_node == start_node and not len(self.open_list) == 0:
            x_plot.append(iteration)
            y_plot.append(self.open_length)
            iteration += 1
            #print("Curr",curr_node)
            bg[505 - curr_node[1], curr_node[0] + 555] = red
            # make g_cost = rhs
            self.nodes[curr_node].g_cost = self.nodes[curr_node].rhs
            # remove curr_node from the open list
            del self.open_list[curr_node]
            self.open_length -= 1
            # get successors of the curr_node
            self.calculate_neighbours(curr_node)
            for n in self.nodes[curr_node].neighbours:
                bg[505 - n[1], n[0] + 555] = green
                self.nodes[n].parent = curr_node
                # rhs of successor = g of parent + path cost
                self.nodes[n].rhs = self.nodes[curr_node].g_cost + self.nodes[curr_node].neighbours[n]
                if not self.node_is_consistent(n):
                    self.open_list[n] = self.get_key(n, start_node)
                    self.open_length += 1
            curr_node = self.get_smallest(self.open_list)
        current_path = []
        while not self.nodes[curr_node].parent == None:
            bg[505 - curr_node[1], curr_node[0] + 555] = (250, 0, 0)
            current_path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
        current_path.append(curr_node)
        cv2.imshow("D star lite output", bg)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        plt.scatter(x_plot,y_plot,c='r')
        plt.show()
        print("Current",current_path)
        return current_path

    # new neighbours
    def new_calculate_neighbours(self, curr_node, open_list, current_path):
        x = curr_node[0]
        y = curr_node[1]
        dig = 1.41
        strght = 1
        if (x-1,y+1) not in self.obstacle_space and x-1 >= -555 and y+1 < 505:
            if (x-1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y+1) and (x-1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x-1,y+1)] = dig
        if (x,y+1) not in self.obstacle_space and y+1 < 505:
            if (x,y+1) not in open_list and not self.nodes[(x,y)].parent == (x,y+1) and (x,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x,y+1)] = strght
        if (x+1,y+1) not in self.obstacle_space and x+1 < 555 and y+1 < 505:
            if (x+1,y+1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y+1) and (x+1,y+1) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y+1)] = dig
        if (x-1,y-1) not in self.obstacle_space and x-1 >= -555 and y-1 >= -505:
            if (x-1, y-1) not in open_list and not self.nodes[(x,y)].parent == (x-1,y-1) and (x-1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1, y-1)] = dig
        if (x,y-1) not in self.obstacle_space and y-1 >= -505:
            if (x, y-1) not in open_list and not self.nodes[(x,y)].parent == (x,y-1) and (x,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x, y-1)] = strght
        if (x+1,y-1) not in self.obstacle_space and x+1 < 555 and y-1 >= -505:
            if (x+1,y-1) not in open_list and not self.nodes[(x,y)].parent == (x+1,y-1) and (x+1,y-1) not in current_path:
                self.nodes[(x, y)].neighbours[(x+1, y-1)] = dig
        if (x-1,y) not in self.obstacle_space and x-1 >= -555:
            if (x-1,y) not in open_list and not self.nodes[(x,y)].parent == (x-1,y) and (x-1,y) not in current_path:
                self.nodes[(x, y)].neighbours[(x-1,y)] = strght
        if (x+1,y) not in self.obstacle_space and x+1 < 555:
            if (x+1,y) not in open_list and not self.nodes[(x,y)].parent == (x+1,y) and (x+1,y) not in current_path:
                self.nodes[(x,y)].neighbours[(x+1,y)] = strght
    # get new key for replanning
    def get_new_key(self, node, start):
        key = min(self.nodes[node].g_cost, self.nodes[node].rhs) + self.h(node, start)
        return key

    # replanning the path from current start point
    def replan(self,rob_x, rob_y, goal_x, goal_y,current_path,bg):
        # get coordinates for the start node
        start_node = (rob_x, rob_y)
        # get coordinates for the goal node
        goal_node = (goal_x, goal_y)
        new_open_list = {}
        visited = []
        # make cost of start node zero
        self.nodes[goal_node].rhs = 0
        new_open_list[goal_node] = 0  # key needs to be written here
        self.new_open_length += 1
        curr_node = goal_node
        parent = self.nodes[goal_node].parent
        while not parent == None:
            visited.append(parent)
            parent = self.nodes[parent].parent
        while not curr_node == start_node and not len(new_open_list) == 0:
            self.x_replan.append(self.new_itr)
            self.y_replan.append(self.new_open_length)
            self.new_itr += 1
            bg[505 - curr_node[1], curr_node[0] + 555] = (0,0,250)
            visited.append(curr_node)
            # make g_cost = rhs
            self.nodes[curr_node].g_cost = self.nodes[curr_node].rhs
            # print("len",len(self.open_list))
            # first make all neighbour rhs = infinitys
            # Now recalculate for new path
            self.nodes[curr_node].neighbours = {}
            self.new_calculate_neighbours(curr_node, new_open_list, visited)
            for n in self.nodes[curr_node].neighbours:
                if n in visited:
                    continue
                bg[505 - n[1], n[0] + 555] = (60, 179, 113)
                self.nodes[n].parent = curr_node
                # rhs of successor = g of parent + path cost
                self.nodes[n].rhs = self.nodes[curr_node].g_cost + self.nodes[curr_node].neighbours[n]
                if not self.node_is_consistent(n):
                    new_open_list[n] = self.get_new_key(n, start_node)
                    self.new_open_length += 1
            # remove curr_node from the open list
            del new_open_list[curr_node]
            self.new_open_length -= 1
            curr_node = self.get_smallest(new_open_list)
        new_path = []
        count = 0
        while not self.nodes[curr_node].parent == None:
            #while not curr_node == goal_node:
            bg[505 - curr_node[1], curr_node[0] + 555] = (250, 0, 0)
            new_path.append(curr_node)
            curr_node = self.nodes[curr_node].parent
            count += 1
        new_path.append(curr_node)
        plt.ylabel('States in OPEN LIST')
        plt.xlabel('No of iterations')
        plt.plot(self.x_replan, self.y_replan, c='b')
        plt.show()
        return new_path

    # Travel across the received path
    def traverse(self,bg,current_path, y1=-30, y2=-30):
        self.obstacle_space.add((5, 5))
        bg[505 - 5, 5 + 555] = (0, 0, 0)
        for p in current_path:
            bg[505 - p[1], p[0] + 555] = (255, 255, 255)
            if self.nodes[p].parent in self.obstacle_space:
                print("Obstacle found at ", self.nodes[p].parent)
                parent = self.nodes[p].parent
                # get next available parent on path
                while parent in self.obstacle_space:
                    print("Obstacle found at ", parent)
                    parent = self.nodes[parent].parent
                new_path = self.replan(p[0], p[1], parent[0], parent[1],current_path,bg)
                # Dummy dynamic obstacle generation
                # travel again on new path
                self.traverse(bg, new_path,y1,y2)
                break
            cv2.imshow("Output", bg)
            cv2.waitKey(50)
        cv2.imshow("Output", bg)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
########################################################################################################################
def initiate():
    # scale the window size
    height = 1010
    width = 1110
    # create a background image
    bg = np.zeros((height,width,3),dtype=np.uint8)
    # Calculate Mikowski space
    r = 25
    print("Calculating Minowski Space. please wait")
    graph = Graph()
    obstacle_set_min = set()
    # iterate for each pixel and find out if it is an obstacle
    # if it is in the obstacle store it in the obstacle set
    graph.create_nodes()
    # define color for node in open list
    green = (60, 179, 113)
    # define color for unvisited node
    grey = (192, 192, 192)
    # define color for the nodes on the path
    blue = (250, 50, 50)
    # color for explored node
    black = (0, 0, 0)
    # apply appropriate colors
    for node in graph.nodes:
        x = node[0]
        y = node[1]
        # y = -y
        bg[505 - y, x + 555] = grey
    '''    
    for y in range(8, 13):
        if not y == 10:
            graph.obstacle_space.add((10, y))
            bg[505 - y, 10 + 555] = (0, 0, 0)
    '''
    graph.obstacle_space.add((5, 4))
    graph.obstacle_space.add((5, 6))
    bg[505 - 4, 5 + 555] = (0, 0, 0)
    bg[505 - 6, 5 + 555] = (0, 0, 0)
    #bg2 = cv2.resize(bg, (550, 505))
    #cv2.imshow("colored.png", bg)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    current_path = graph.d_star_lite_algo(0,0,10,10,bg)
    print("Current path: ",current_path)
    graph.traverse(bg,current_path,-30,-30)
initiate()