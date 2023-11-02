import sys
import time
from map import Map
from gui import *
from utils import *
from robot_sim import *

MAX_NODES = 20000

def RRT(map):
    """ Builds RRT given a map
    """
    map.add_node(map.get_start())
    map_width, map_height = map.get_size()
    while (map.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use Map.node_generator() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node using map.add_path
        # Note: you do not need to call the add_node function, since add_path
        #       will add the node to the map as well.

        # generate a random node
        randoNode = Map.node_generator(map)
        # get nearest node to the random node from RRT
        allNodes = np.array(Map.get_nodes(map))
        closestNode = None
        closestDist = np.inf
        for curr in allNodes:
            disty = get_dist(curr, randoNode)
            if disty < closestDist:
                closestDist = disty
                closestNode = curr
        # limit the distance RRT can move
        limit = 35
        nodeLimited = Map.step_from_to(map, closestNode, randoNode, limit)
        # add a path from the nearest node to random node using map.add_path
        newNode = Map.add_path(map,closestNode, nodeLimited)
        ########################################################################
        time.sleep(0.01)
        if map.is_solved():
            break

    path = map.get_path()
    smoothed_path = map.get_smooth_path()

    if map.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", map.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


def robot_planning_with_exploration(robot, map):
    map.check_new_obstacle(robot, DDRobot.VISION_DISTANCE)
    RRT(map)
    path = map.get_smooth_path()
    #print(path)
    #print(path)
    #next_pos
    DDRobot(path[0].x, path[0].y, map)
    #while the current robot position is not at the goal:
    while get_dist(robot, map.get_goals()[0]) > 1:
        if get_dist(robot, map.get_goals()[0]) <= 1:
            return
        ########################################################################
        # TODO: please enter your code below.
        # Description of function provided in instructions. Potential pseudcode is below
        #print(next_pos)
        # Get the next node from the path
        next_pos = path.pop(0)
        #print(next_pos)
        #print(path)
        # drive the robot to next node in path. First turn to the appropriate angle, which
        # you can calculate using a trigonometric function
        theta = np.arctan2(next_pos.y - robot.y, next_pos.x - robot.x)
        #print(theta)
        robot.turn_in_place(theta - robot.theta)

        # while robot has not reached the next node in the path
        while get_dist(robot, next_pos) > 1:
            if get_dist(robot, map.get_goals()[0]) < 1:
                break
            # detect any visible obstacles and update cmap
            map.check_new_obstacle(robot, get_dist(robot, next_pos))
            # if new obstacles are detected, reset the cmap with the current robot location as 
            # the start node, re-plan using RRT to generate a new path
            if not (map.is_collision_with_obstacles((robot, next_pos))):
                robot.move_forward(get_dist(robot, next_pos))
                #print(robot)
            else:
                # reset the map
                map.reset(Node([robot.x, robot.y]))
                # map.check_new_obstacle(robot, DDRobot.VISION_DISTANCE)
                # RRT(map)
                # path = map.get_smooth_path()

                #Replan using RRT to generate a new path

                robot_planning_with_exploration(robot, map)
            # otherwise, drive straight towards the next node within vision distance

        # Hint: feel free to use function robot.turn_in_place(), robot.move_forward()
        # map.check_new_obstacle(), map.reset(), etc.

        
        ########################################################################


class RobotThread(threading.Thread):
    """Thread to run vector code separate from main thread
    """

    def __init__(self, robot, map ):
        threading.Thread.__init__(self, daemon=True)
        self.robot = robot
        self.map = map

    def run(self):
        robot_planning_with_exploration(self.robot, self.map)
        time.sleep(5)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self, map):
        threading.Thread.__init__(self, daemon=True)
        self.map = map

    def run(self):
        RRT(self.map)
        time.sleep(5)
        self.map.reset_paths()
        stopevent.set()


if __name__ == '__main__':
    global stopevent
    stopevent = threading.Event()
    exploration = False
    for i in range(0,len(sys.argv)): 
        #reads input whether we are running the exploration version or not
        if (sys.argv[i] == "-explore"):
            exploration = True

    map = Map("maps/map2.json", exploration)
    
    if exploration:
        r = DDRobot(map.get_start().x, map.get_start().y, map)
        robot_thread = RobotThread(robot=r, map=map)
        visualizer = Visualizer(map, r, stopevent, exploration)
        robot_thread.start()
        visualizer.start()
    else:
        rrt_thread = RRTThread(map=map)
        visualizer = Visualizer(map, None, stopevent, exploration)
        rrt_thread.start()
        visualizer.start()
