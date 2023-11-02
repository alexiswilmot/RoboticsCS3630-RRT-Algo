from rrt import *
import json
import os
from robot_sim import DDRobot 
import pickle

def is_path_collision_free(map, smooth_path):
    if len(smooth_path) > 0:
        for i in range(1, len(smooth_path)):
            if map.is_collision_with_obstacles(((smooth_path[i-1], smooth_path[i]))):
                print("Collided")
                return False
    
    if (smooth_path[0].x, smooth_path[0].y) != (map.get_start().x, map.get_start().y):
        print("Start point not matching")
        return False
    
    if get_dist(smooth_path[-1], map.get_goals()[0]) > 1:
        print("Goal point not matching")
        return False
    
    return True
    
def test_path_smoothing(map, path):
    smooth_path = map.compute_smooth_path(path)
    if is_path_collision_free(map, smooth_path):
        return True, len(smooth_path)
    return False, None


def is_solution_valid_from_path(map, path):
    
    if len(path) > 0:
        for i in range(1, len(path)):
            node0 = Node(tuple(path[i-1]))
            node1 = Node(tuple(path[i]))
            if map.is_collision_with_obstacles(((node0, node1))):
                print("Collided")
                return False
    
    with open(map.fname) as configfile:
        # Load dimensions from json file
        config = json.loads(configfile.read())
        start_point = Node(tuple(config['start']))

    if (path[0][0], path[0][1]) != (start_point.x, start_point.y):
        print("Start point not matching")
        return False
    
    if get_dist(Node(tuple(path[-1])), map.get_goals()[0]) > 1:
        print("Goal point not matching")
        return False
    
    return True



class GradingThread():
    """RRT grading thread
    """
    def __init__(self, maps, path_files=None):
        self.maps = maps
        self.path_files = path_files

    def run(self):
        print("Grader running...\n")
        points = 0
        total = len(self.maps) * 10

        for map in self.maps:
            cmap = Map(map)
            RRT(cmap)
            if (cmap.is_solution_valid()):
                points += 10
            print(map + ": " + str(points) + "/" + str(total) + " points")

        print("\nScore = " + str(points) + "/" + str(total) + "\n")
    
    def check_path_smoothing(self):
        print("Grader running...\n")
        points = 0
        total = len(self.maps) * 5
        node_count_dict = {}
        node_count_dict["maps/map7.json"] = 12


        for map, path_file in zip(self.maps, self.path_files):
            path_file = path_file
            map_file = map
            with open(path_file, "rb") as file:
                loaded_path = pickle.load(file)
            cmap = Map(map_file)
            min_path_length = sys.maxsize

            for i in range(5):
                isValid, count = test_path_smoothing(cmap, loaded_path)
                if isValid:
                    min_path_length = min(min_path_length, count)
            
            if min_path_length <= node_count_dict[map]:
                points += 5
            
            map_name = os.path.basename(map_file)
            print(map_name + ": " + str(points) + "/" + str(total) + " points")
        
        print("\nScore = " + str(points) + "/" + str(total) + "\n")
        return points



    def check_robot_trajectory(self):
        points = 0
        total = len(self.maps) * 12.5
        exploration = True

        for map in self.maps:
            print(map)
            cmap = Map(map, exploration)
            r = DDRobot(cmap.get_start().x, cmap.get_start().y, cmap)
            robot_planning_with_exploration(r, cmap)
            if (is_solution_valid_from_path(cmap, r.trajectory)):
                points += 12.5
            print(map + ": " + str(points) + "/" + str(total) + " points")
        
        print("\nScore = " + str(points) + "/" + str(total) + "\n")


if __name__ == "__main__":
    tests = {}
    if len(sys.argv) > 1:
        try:
            # Test to run
            test = str(sys.argv[1]) # Give {test1, test2, test3}
        except:
            print("Error opening test file, please give the test option {test1, test2, test3}")
            raise
    else:
        print("correct usage: python3 autograder.py <testfile>")
        exit()

    if test == "test1":
        maps = ["maps/map1.json", "maps/map2.json", "maps/map3.json"]
        grader = GradingThread(maps)
        grader.run()
    
    if test == "test2":
        maps = ["maps/map7.json"]
        path_files = ["paths/tests_path7.pkl"]
        grader = GradingThread(maps, path_files=path_files)
        grader.check_path_smoothing()

    if test=="test3":
        maps = ["maps/map1.json", "maps/map2.json", "maps/map3.json"]
        grader = GradingThread(maps)
        grader.check_robot_trajectory()
