import json
import threading
from utils import *
#np.random.seed(1)

class Map:
    """Class representing a map for search algorithms.

        Features include: start location, goal location, obstacles, and path storage
        Configuration is loaded from js on file supplied at object creation
        Designed to be thread-safe

        Attributes:
        width -- width of map, in mm
        height -- height of map, in mm
    """

    def __init__(self, fname, exploration_mode=False):
        self.fname = fname
        with open(fname) as configfile:
            # Load dimensions from json file
            config = json.loads(configfile.read())
            self.width = config['width']
            self.height = config['height']

            # Initially empty private data, please access through functions below
            self._start = Node(tuple(config['start']))
            self._goals = [Node(tuple(coord)) for coord in config['goals']]
            self._obstacles = []
            self._nodes = []  # node in RRT
            self._node_paths = []  # edge in RRT
            self._solved = False
            self._smooth_path = []
            self._smoothed = False
            self._restarts = []

            # Read in obstacles
            for obstacle in config['obstacles']:
                self._obstacles.append([Node(tuple(coord)) for coord in obstacle])

            # For coordination with visualization
            self.lock = threading.Lock()
            self.updated = threading.Event()
            self.changes = []

            self._exploration_mode = exploration_mode
            self._explored_obstacles = [] # for part 2, keep track of obstacles the robot has explored

    def is_inbound(self, node):
        """Check if node is within legitimate range

            Arguments:
            node -- grid coordinates
        """
        if ((node.x >= 0) and (node.y >= 0) and (node.x < self.width - 0) and (node.y < self.height - 0)):
            return True
        else:
            return False

    def is_collision_with_obstacles(self, line_segment):
        """Check if a line segment intersects with any obstacles
        
            Arguments:
            line_segment -- a tuple of two node
        """
        obstacles = self._explored_obstacles if self._exploration_mode else self._obstacles
        line_start, line_end = line_segment
        for obstacle in obstacles:
            num_sides = len(obstacle)
            for idx in range(num_sides):
                side_start, side_end = obstacle[idx], obstacle[(idx + 1) % num_sides]
                if is_intersect(line_start, line_end, side_start, side_end):
                    return True
        return False

    def is_inside_obstacles(self, node, use_all_obstacles = False):
        """Check if a node is inside any obstacles
           Notice: we have bumped up the size of obstacles with a safety distance.
                   Therefore, we only need to check if the center of the robot is
                   inside obstacles.
        
            Arguments:
            node -- the query node
        """
        obstacles = self._explored_obstacles if self._exploration_mode and not use_all_obstacles else self._obstacles
        for obstacle in obstacles:
            num_sides = len(obstacle)
            is_inside = True
            for idx in range(num_sides):
                side_start, side_end = obstacle[idx], obstacle[(idx + 1) % num_sides]
                if get_orientation(side_start, side_end, node) == 2:
                    is_inside = False
                    break
            if is_inside:
                return True
        return False

    def get_size(self):
        """Return the size of grid
        """
        return self.width, self.height

    def get_nodes(self):
        """Return all nodes in RRT
        """
        return self._nodes

    def get_goals(self):
        """Return list of goals. You can assume there is only one goal.
        """
        return self._goals
    
    def get_restarts(self):
        return self._restarts

    def reset(self, node):
        """Reset the map by clearing the existing nodes and paths, 
           and set the new start to the node
        """
        self.set_start(node)
        self.reset_paths()
        self.add_restart()

    def add_restart(self):
        self._restarts.append(self.get_start())

    def get_num_nodes(self):
        """Return number of nodes in RRT
        """
        return len(self._nodes)

    def set_start(self, node):
        """Set the start cell

            Arguments:
            node -- grid coordinates of start cell
        """
        if self.is_inside_obstacles(node) or (not self.is_inbound(node)):
            print("start is not updated since your start is not legitimate\nplease try another one\n")
            return
        self.lock.acquire()
        self._start = Node((node.x, node.y))
        self.updated.set()
        self.changes.append('start')
        self.lock.release()

    def get_start(self):
        """Get start
        """
        return self._start

    def add_goal(self, node):
        """Add one more goal

            Arguments:
            node -- grid coordinates of goal cell
        """
        if self.is_inside_obstacles(node) or (not self.is_inbound(node)):
            print("goal is not added since your goal is not legitimate\nplease try another one\n")
            return
        self.lock.acquire()
        self._goals.append(node)
        self.updated.set()
        self.changes.append('goals')
        self.lock.release()

    def add_obstacle(self, nodes):
        """Add one more obstacles

            Arguments:
            nodes -- a  of four nodes denoting four corners of a rectangle obstacle, in clockwise order
        """

        self.lock.acquire()
        self._obstacles.append(nodes)
        self.updated.set()
        self.changes.append('obstacles')
        self.lock.release()

    def get_random_valid_node(self):
        """Get one random node which is inbound and avoids obstacles
        """
        return self._node_generator(self)

    def add_node(self, node):
        """Add one node to RRT
        """
        self.lock.acquire()
        self._nodes.append(node)
        self.updated.set()
        self.changes.append('nodes')
        self.lock.release()

    def add_path(self, start_node, end_node):
        """Add one edge to RRT, and add the end_node to nodes. If end_node is
           the goal or close to goal mark problem as solved.

            Arguments:
            start_node -- start node of the path
            end_node -- end node of the path
        """
        if self.is_collision_with_obstacles((start_node, end_node)):
            return
        self.lock.acquire()
        end_node.parent = start_node
        self._nodes.append(end_node)
        self._node_paths.append((start_node, end_node))

        for goal in self._goals:
            if end_node == goal:
                self._solved = True
                break
            if get_dist(goal, end_node) < 15 and (not self.is_collision_with_obstacles((end_node, goal))):
                goal.parent = end_node
                self._nodes.append(goal)
                self._node_paths.append((end_node, goal))
                self._solved = True
                break

        self.updated.set()
        self.changes.extend(['node_paths', 'nodes', 'solved' if self._solved else None])
        self.lock.release()
    

    def is_solved(self):
        """Return whether a solution has been found
        """
        return self._solved

    def is_solution_valid(self):
        """Check if a valid has been found
        """
        # add a check which iterates over all restart nodes 
        # and checks if they are not colliding
        for node in self.get_restarts():
            if self.is_inside_obstacles(node):
                print("get restarts")
                return False
        # check the final path returned for RRT
        if not self._solved:
            print("not solved")
            return False
        cur = None
        for goal in self._goals:
            cur = goal
            while cur.parent is not None:
                cur = cur.parent
            if cur == self._start:
                print("soliution")
                return True
        print("not start")
        return False

    def step_from_to(self, node0, node1, limit=45):
        ########################################################################
        # TODO: please enter your code below.
        # 1. If distance between two nodes is less than limit, return node1
        # 2. Otherwise, return a node in the direction from node0 to node1 whose
        #    distance to node0 is limit. Recall that each iteration we can move
        #    limit units at most
        # 3. Hint: please consider using np.arctan2 function to get vector angle
        # 4. Note: remember always return a Node object
        ############################################################################
        x0 = node0.x
        y0 = node0.y
        #print(node1)
        x1 = node1.x
        y1 = node1.y
        disty = get_dist(node1, node0)
        # if distance is less than the limit, return the second ndoe
        if (disty < limit):
            return node1
        # if not, 
        theta = np.arctan2(y1 - y0, x1 - x0)
        x = x0 + limit * np.cos(theta)
        y = y0 + limit * np.sin(theta)
        return Node((x, y))
        #############################################################################
        

    
    def node_generator(self):
        #rand_node = None
        ############################################################################
        # TODO: please enter your code below.
        # 1. Use Map width and height to get a uniformly distributed random node
        # 2. Use Map.is_inbound and Map.is_inside_obstacles to determine the
        #    legitimacy of the random node.
        # 3. Note: return a node at the goal with 5% chance.
        # remember to always return a Node object
        ############################################################################
        width = self.width
        height = self.height
        prob_goal = .05
        if np.random.random() <= prob_goal:
            rand_node = self.get_goals()[0]
            #print("GOAL")
            bad = False
        else:
            bad = True
            while bad:
                # generate random coordinates
                x = int(np.random.uniform(0, width))
                y = int(np.random.uniform(0, height))
                rando = Node([x, y])
                if self.is_inbound(rando) and not self.is_inside_obstacles(rando):
                    rand_node = rando
                    bad = False
        return rand_node
        ############################################################################
    
    def get_smooth_path(self):
        if self._smoothed:
            return self._smooth_path[:]
        self.lock.acquire()
        self._smooth_path = self.compute_smooth_path(self.get_path())
        self._smoothed = True
        self.updated.set()
        self.changes.append('smoothed')
        self.lock.release()
        return self._smooth_path[:]

    def compute_smooth_path(self, path):
        """ Return a smoothed path given the original unsmoothed path.

            Arguments:
            path -- original unsmoothed path
        """
        ############################################################################
        # TODO: please enter your code below.
        if len(path) <= 2:
            return path
        smooth = path
        #print(len(path))
        #print(smooth)
        e = 0
        for e in range(1):
            temp_smooth = [path[0]]
            #print(temp_smooth)
            x = 1
            while x < (len(smooth) - 3):
                no1 = smooth[x]
                no2 = smooth[x + 1]
                no3 = smooth[x + 2]
            # check if there are obstacles
                if not self.is_collision_with_obstacles((no1, no3)):
                    #temp_smooth.append(no1)
                    temp_smooth.append(no3)
                    x = x + 2
                    #x += 1
                else:
                    #temp_smooth.append(no1)
                    temp_smooth.append(no2)
                    temp_smooth.append(no3)
                    x = x + 2
                    #newNode = Map.add_path(self,no1, no3)

            temp_smooth.append(path[-1])
            
            #print(len(temp_smooth))
            smooth = temp_smooth
        #print(len(smooth))
        #print("\n end")

        # Return a smoothed path by reducing the number of nodes in the path, 
        # while maintaining collision free.

        # is_collision_with_obstacles(self, line_segment)
        # if len(path) <= 2:
        #     return path
        # for i in range(100): 
        #     # repeatedly pick 2 nodes at random, check if they can be connected by a straight line without a collision.
        #     index1 = np.random.randint(1, len(path) - 1)
        #     index2 = np.random.randint(1, len(path) - 1)
        #     while index1 == index2:
        #         index2 = np.random.randint(0, len(path) - 1)
        #     if index1 > index2:
        #         node1 = path[index2]
        #         print(node1)
        #         node2 = path[index1]
        #     else:
        #         node1 = path[index1]
        #         node2 = path[index2]
        #     # check if they can be connected by a straight line
        #     collision = self.is_collision_with_obstacles((node1, node2))
        #     # if so, add to path and remove the nodes in between the two nodes
        #     # if no collisions, snip it
        #     if not collision:
        #         #node2.parent = node1
        #         newNode = Map.add_path(self, node1, node2)
        # path = self.get_path()
        return smooth
        ############################################################################

    def get_path(self):
        
        final_path = None
        
        while final_path is None:
            path = []
            cur = None
            for goal in self._goals:
                cur = goal
                while cur.parent is not None:
                    path.append(cur)
                    cur = cur.parent
                if cur == self._start:
                    path.append(cur)
                    break
            final_path = path[::-1]
        
        return final_path

    def is_solved(self):
        """Return whether a solution has been found
        """
        return self._solved

    def is_solution_valid(self):
        """Check if a valid has been found
        """
        if not self._solved:
            return False
        cur = None
        for goal in self._goals:
            cur = goal
            while cur.parent is not None:
                cur = cur.parent
            if cur == self._start:
                return True
        return False

    def reset_paths(self):
        """Reset the grid so that RRT can run again
        """
        self.clear_solved()
        self.clear_nodes()
        self.clear_node_paths()
        self.clear_smooth_path()

    def clear_smooth_path(self):
        """Clear solved state
        """
        self.lock.acquire()
        self._smoothed = False
        self._smooth_path = []
        self.updated.set()
        self.lock.release()

    def clear_solved(self):
        """Clear solved state
        """
        self.lock.acquire()
        self._solved = False
        for goal in self._goals:
            goal.parent = None
        self.updated.set()
        self.changes.append('solved')
        self.lock.release()

    def clear_nodes(self):
        """Clear all nodes in RRT
        """
        self.lock.acquire()
        self._nodes = []
        self.updated.set()
        self.changes.append('nodes')
        self.lock.release()

    def clear_node_paths(self):
        """Clear all edges in RRT
        """
        self.lock.acquire()
        self._node_paths = []
        self.updated.set()
        self.changes.append('node_paths')
        self.lock.release()

    def clear_goals(self):
        """Clear all goals
        """
        self.lock.acquire()
        self._goals = []
        self.updated.set()
        self.changes.append('goals')
        self.lock.release()

    def clear_obstacles(self):
        """Clear all obstacle
        """
        self.lock.acquire()
        self._obstacles = []
        self.updated.set()
        self.changes.append('obstacles')
        self.lock.release()

    def check_new_obstacle(self, robot, vision_distance: float):
        """Check if new obstacles are observed
        """
        self.lock.acquire()
        has_new_obstacle = False
        for obstacle in self._obstacles:
            if obstacle in self._explored_obstacles:
                continue
            if self.distance_to_obstacle(robot, obstacle) <= vision_distance:
                self._explored_obstacles.append(obstacle)
                self.changes.append('obstacles')
                has_new_obstacle = True
        self.lock.release()
        return has_new_obstacle

    def distance_to_obstacle(self, robot, obstacle):
        """Return distance from robot to the obstacle
        """
        x, y = robot.x, robot.y
        x1, y1 = obstacle[0].x, obstacle[0].y
        x2, y2 = obstacle[2].x, obstacle[2].y
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)

        distances = []
        if x1 < x < x2 and y1 < y < y2: 
            return 0
        if x1 < x < x2:
            distances.append(min(abs(y - y1), abs(y - y2)))
        if y1 < y < y2:
            distances.append(min(abs(x - x1), abs(x - x2)))
        for corner in obstacle:
            bx, by = corner.x, corner.y
            distances.append(((x - bx) ** 2 + (y - by) ** 2) ** 0.5)
        return min(distances)
