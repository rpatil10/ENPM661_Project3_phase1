import math
import heapq
import numpy as np
import cv2

# Declaring global variables
STEP_SIZE = 0
RADIUS = 0

# Blank workspace/canvas dimensions
workspace_height = 250
workspace_width = 400

y_flip = 250

# Create blank canvas
blank_canvas = np.zeros((workspace_height,workspace_width,3), np.uint8)

# Videowriter function to show the animation of the goal traversal
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('A_star.avi',fourcc,10,(400,250))

# Class to store node details
class Node:
    def __init__(self, x, y, final_x, final_y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost_to_come = float('inf')
        self.cost_to_go = math.sqrt((x-final_x)**2 + (y-final_y)**2)
        self.cost = None
        self.neighbours = {}
        self.parent = None
    # def __lt__(self,other):
    #     return self.cost < other.cost

# Class to obtain grid and compute A*
class Grid:
    
    def __init__(self, start, end, STEP_SIZE, RADIUS, workspace):
        self.visited = {}
        self.final_x = end.x
        self.final_y = end.y
        self.STEP_SIZE = STEP_SIZE
        self.RADIUS = RADIUS
        self.workspace = workspace
    
    def get_rounded_number(self,a):
        decimal = a - int(a)
        if decimal > 0.5:
            if decimal >= 0.75:
                return math.ceil(a)
            else:
                return int(a) + 0.5
        elif decimal < 0.5:
            if decimal < 0.25:
                return math.floor(a)
            else:
                return int(a) + 0.5
        return a
    
    # Function to generate new coordinates around current node
    def get_new_location(self, x, y, theta, delta_theta):
        new_theta = theta + delta_theta
        if new_theta > 0:
            new_theta = new_theta % 360
        elif new_theta < 0:
            new_theta = (new_theta + 360) % 360
        new_x = x + self.STEP_SIZE * math.cos((math.pi / 180) * new_theta)
        new_y = y + self.STEP_SIZE * math.sin((math.pi / 180) * new_theta)
        new_x = self.get_rounded_number(new_x)
        new_y = self.get_rounded_number(new_y)
        return new_x, new_y, new_theta
    
    def is_outside_playground(self, x, y):
        return True if x < self.RADIUS or y < self.RADIUS or x > 400-self.RADIUS or y > 250-self.RADIUS else False
    
    def is_inside_obstacle(self, x, y):
        if self.is_in_circle(x,y_flip-y) or self.is_in_hexagon(x,y_flip-y) or self.is_in_arrow_polygon(x,y_flip-y):
            return True
        else:
            False
    
    def is_in_circle(self, x, y):
        r = 40 + 10
        if (x-300)**2 + (y-65)**2 - r**2 >= 0:
            return False
        else:
            return True
        
    def is_in_hexagon(self ,x, y):
        line1 = x > 155
        line2 = y+(26/45)*x >= 1922/9
        line3 = y-(26/45)*x >= -158/9
        line4 = x < 245
        line5 = y+(26/45)*x <= 2858/9
        line6 = y-(26/45)*x <= 778/9
        if line1 and line2 and line3 and line4 and line5 and line6:
            return True
        else:
            return False
    
    def is_in_arrow_polygon(self, x, y):
        line1 = y + (6/7)*x < 152
        line2 = y - (16/5)*x > -219
        line3 = y - (85/69)*x < 36
        line4 = y + (25/79)*x > 66
        if (line1 or line2) and (line3 and line4):
            return True
        else:
            return False
    
    # Function to calculate coordinates of neighbouring nodes
    def get_neighbours_of_node(self, current_node):
        x, y, theta = current_node.x, current_node.y, current_node.theta
        neighbours = {}
        for delta_theta in range(-60, 90, 30):
            x, y, new_theta = self.get_new_location(x, y, theta, delta_theta)
            if(not self.is_outside_playground(x, y)) and (not self.is_inside_obstacle(x, y)):
                new_node = Node(x, y, self.final_x, self.final_y, new_theta)
                neighbours[new_node] = 1
        return neighbours
    
    def is_in_goal_area(self, x, y):
        if (x - self.final_x)**2 + (y - self.final_y)**2 - 100 <= 0:
            return True
        else:
            return False
    
    def compute_a_star_algo(self, start, end):
        print("Finding Path")
        priorityQueue = []
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):
            current_node = heapq.heappop(priorityQueue)
            current_node = current_node[1]
            if self.is_in_goal_area(current_node.x, current_node.y):
                print("Found path")
                return True
            current_distance = current_node.cost_to_come
            neighbours = self.get_neighbours_of_node(current_node)
            current_node.neighbours = neighbours
            for neighbour_node, new_distance in neighbours.items():
                neighbour_node.cost_to_come = current_distance + new_distance
                neighbour_node.cost = neighbour_node.cost_to_come + neighbour_node.cost_to_go
                neighbour_node.parent = current_node
                heapq.heappush(priorityQueue, (neighbour_node.cost, neighbour_node))
        print("Cannot find a path")
        return False
    
    def back_track(self, child):
        while child != None:
            path.append((child.x,child.y))
            child = child.parent
        return True
    
    def algo_visualization(self, start, end):
        self.visited = {}
        priorityQueue = []
        heapq.heappush(priorityQueue, (start.cost, start))
        cv2.circle(workspace, (start.x, y_flip-start.y),10, (0,255,0),-1)
        cv2.circle(workspace,(end.x,y_flip-end.y),10,(255,0,0),-1)
        while len(priorityQueue):
            current_node = heapq.heappop(priorityQueue)
            current_node = current_node[1]
            if self.is_in_goal_area(current_node.x,current_node.y):
                self.back_track(current_node)
                return
            current_distance = current_node.cost_to_come
            for neighbour_node, new_distance in current_node.neighbours.items():
                x = neighbour_node.x
                y = neighbour_node.y
                cv2.arrowedLine(workspace, (int(current_node.x),y_flip-int(current_node.y)), (int(x), y_flip-int(y)),(255,255,255), 1)
                cv2.imshow('Path',workspace)
                cv2.waitKey(100)
                out.write(workspace)
                heapq.heappush(priorityQueue, (neighbour_node.cost, neighbour_node))
        return
            

# Function to draw hexagon obstacle using half planes method
def draw_hexagon():
    os_h_1 = blank_canvas.copy()
    for i in range(workspace_width):
        if(235>=i):
            os_h_1[:,i] = [0,255,255]

    os_h_2 = blank_canvas.copy()
    for i in range(workspace_width):
        if(165<=i):
            os_h_2[:,i] = [0,255,255]

    os_h_3 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j-(4/7)*i >= -(30/7)):
                os_h_3[j,i] = [0,255,255]

    os_h_4 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j+(4/7)*i <= (2130/7)):
                os_h_4[j,i] = [0,255,255]

    os_h_5 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j-(4/7)*i <= (530/7)):
                os_h_5[j,i] = [0,255,255]

    os_h_6 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j+(4/7)*i >= (1570/7)):
                os_h_6[j,i] = [0,255,255]

    output_h_12 = cv2.bitwise_and(os_h_2, os_h_1, mask=None)
    output_h_123 = cv2.bitwise_and(os_h_3, output_h_12, mask=None)
    output_h_1234 = cv2.bitwise_and(os_h_4, output_h_123, mask=None)
    output_h_12345 = cv2.bitwise_and(os_h_5, output_h_1234, mask=None)
    output_h = cv2.bitwise_and(os_h_6, output_h_12345, mask=None)
    return output_h

# Function to draw arrow-shaped polygon obstacle using half planes method
def draw_arrow_polygon():
    os_a_1 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j-(16/5)*i > -186):
                os_a_1[j,i] = [0,255,255]

    os_a_2 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j+(6/7)*i < (970/7)):
                os_a_2[j,i] = [0,255,255]

    os_a_3 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j-(85/69)*i < (475/23)):
                os_a_3[j,i] = [0,255,255]

    os_a_4 = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if (j+(25/79)*i > (6035/79)):
                os_a_4[j,i] = [0,255,255]

    output_a_12 = cv2.bitwise_or(os_a_2, os_a_1, mask=None)
    output_a_34 = cv2.bitwise_and(os_a_4, os_a_3, mask=None)
    output_a = cv2.bitwise_and(output_a_34, output_a_12, mask=None)
    return output_a

# Function to draw circle obstacle using half planes method
def draw_circle():
    os_c = blank_canvas.copy()
    for i in range(workspace_width):
        for j in range(workspace_height):
            if ((i-300)**2 + (j-(y_flip-185))**2 <= (40)**2):
                    os_c[j,i] = [0,255,255]
    return os_c

# Function to merge all three obstacles using half planes method
def draw_all_obstacles():
    output_img_ah = cv2.bitwise_or(draw_arrow_polygon(), draw_hexagon(), mask=None)
    output_img_ahc = cv2.bitwise_or(draw_circle(), output_img_ah, mask=None)
    return output_img_ahc

def is_in_circle_for_inputs(x, y):
    r = 40 + 10
    if (x-300)**2 +(y-(y_flip-185))**2 - r**2 >= 0:
        return False
    else:
        print("Location inside circle")
        return True
        
    
def is_in_hexagon_for_inputs(x, y):
    line1 = x > 155
    line2 = y+(26/45)*x >= 1922/9
    line3 = y-(26/45)*x >= -158/9
    line4 = x < 245
    line5 = y+(26/45)*x <= 2858/9
    line6 = y-(26/45)*x <= 778/9
    if line1 and line2 and line3 and line4 and line5 and line6:
        print("Location inside hexagon")
        return True
    else:
        return False

def is_in_arrow_polygon_for_inputs(x, y):
    line1 = y + (6/7)*x < 152
    line2 = y - (16/5)*x > -219
    line3 = y - (85/69)*x < 36
    line4 = y + (25/79)*x > 66
    if (line1 or line2) and (line3 and line4):
        print("Location inside arrow polygon")
        return True
    else:
        return False
    
# Function to check whether given point/location/node lies in obstacle or not, while taking inputs from user
def is_inside_obstacle_for_inputs(x, y):
    if is_in_circle_for_inputs(x,y) or is_in_hexagon_for_inputs(x,y) or is_in_arrow_polygon_for_inputs(x,y):
        return True
    else:
        False
        
def is_outside_playground_for_inputs(x, y):
    return True if x < RADIUS or y < RADIUS or x > 400-RADIUS or y > 250-RADIUS else False
    
# Function to capture start point/location/node and goal point/location/node inputs from user 
def get_start_end_points():
    global STEP_SIZE 
    STEP_SIZE = int(input("Enter the step size of the robot:  "))
    global RADIUS
    RADIUS = int(input("Enter the radius of the robot:  "))
    # Function to capture start point/location/node input from user
    while True:
        start_location_x, start_location_y, start_location_theta = [int(i) for i in input("Enter start location point and orientation[x,y,theta](eg: if (10,10,30), then enter: 10 10 30): ").split()]
        # Condition to check whether start location is beyond workspace/canvas or not
        if is_outside_playground_for_inputs(start_location_x,start_location_y):
            print("Start location cannot be beyond workspace/canvas")
            continue
        start_location_y = y_flip - start_location_y
        start_location_node = (start_location_x, start_location_y, start_location_theta)
        # Condition to check whether start location is within obstacle or not
        if(is_inside_obstacle_for_inputs(start_location_node[0], start_location_node[1])):
            print("Start location cannot be within an obstacle")
            continue
        break
    # Function to capture goal point/location/node input from user
    while True:
        goal_location_x, goal_location_y, goal_location_theta = [int(i) for i in input("Enter goal location point and orientation[x,y,theta](eg: if (115,185,60), then enter: 115 185 60): ").split()]
        # Condition to check whether goal location is beyond workspace/canvas or not
        if is_outside_playground_for_inputs(goal_location_x,goal_location_y):
            print("Goal location cannot be beyond workspace/canvas")
            continue
        goal_location_y = y_flip - goal_location_y
        goal_location_node = (goal_location_x, goal_location_y, goal_location_theta)
        # Condition to check whether goal location is within obstacle or not
        if(is_inside_obstacle_for_inputs(goal_location_node[0], goal_location_node[1])):
            print("Goal location cannot be within an obstacle")
            continue
        break
    return start_location_x, start_location_y, start_location_theta, goal_location_x, goal_location_y, goal_location_theta, start_location_node, goal_location_node

# Function call to generate workspace with all obstacles
workspace = draw_all_obstacles()

# Get start and goal coordinates from user
start_location_x, start_location_y, start_location_theta, goal_location_x, goal_location_y, goal_location_theta, start_location_node, goal_location_node = get_start_end_points()
start_location_y = y_flip - start_location_y
start_location_node = (start_location_x, start_location_y, start_location_theta)
goal_location_y = y_flip - goal_location_y
goal_location_node = (goal_location_x, goal_location_y, goal_location_theta)

# Create start node using start location
start = Node(start_location_node[0],start_location_node[1],goal_location_node[0],goal_location_node[1],start_location_node[2])
start.cost_to_come = 0
# Create goal node using goal location
end = Node(goal_location_node[0],goal_location_node[1],goal_location_node[0],goal_location_node[1],0)
bot = Grid(start, end, STEP_SIZE, RADIUS, workspace)
path = []

if bot.compute_a_star_algo(start, end):
    exit_flag = False
    bot.algo_visualization(start,end)
    path.reverse()
else:
    exit_flag = True

# Loop condition to visualize backtrack 
while not exit_flag:
    for index in range(len(path)):
        x, y = path[index]
        if index != 0:
            cv2.line(workspace,(int(prev_x), y_flip-int(prev_y)),(int(x), y_flip-int(y)), (255, 0, 255),1 )
            cv2.imshow('Path',workspace)
            cv2.waitKey(100)
            out.write(workspace)
        prev_x = x
        prev_y = y
    exit_flag = True
cv2.imshow('Path',workspace)
cv2.waitKey(0)
cv2.imwrite("result.png", workspace)