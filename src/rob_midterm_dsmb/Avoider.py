#!/usr/bin/env python3
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
# Source [3]: https://github.com/Rad-hi/Obstacle-Avoidance-ROS
# Modified to only consider the left, front, and right regions 

from yaml import DirectiveToken
from rob_midterm_dsmb.msg import Solution_direction

class Avoider():

    # Weights for choosing direction
    DIRECTION_WEIGHT = 0.3
    OBSTACLE_WEIGHT = 1.0

    # Husky's FOV is 270 degrees starting from left going clockwise (720 indices in the scan, though!)    
    HUSKY_ALL_REGION_ORDER = [
        "left_L", "left_C", "left_R",
        "front_L", "front_C", "front_R",
        "right_L", "right_C", "right_R",
        "back_L", "back_C", "back_R"
    ]
    # We are not using any scan for the back direction classes
    HUSKY_SCANNERS_REGION_ORDER = HUSKY_ALL_REGION_ORDER[0:9]

    # The cost for deviation from front_c to a particular region for each region
    REGIONS_CHAGNE_COST = {
        "front_C": 0/6, "front_L": 1/6, "front_R": -1/6,
        "left_R": 2/6, "left_C": 3/6, "left_L": 4/6,
        "back_R": 5/6, "back_C": 6/6, "back_L": -5/6,
        "right_R": -4/6, "right_C": -3/6, "right_L": -2/6, 
    }


    def __init__(self, vel_obj, obstacle_threshold=7, region_size=80, normal_lin_vel=0.7, trans_lin_vel=0.2, trans_ang_vel=1.0):
        self.vel_obj = vel_obj
        self.first_scan_received = False
        self.OBSTACLE_DIST_THRESH = obstacle_threshold # obstacles further away than this are ignored in Laser_Scan_Per_Region
        self.REGION_SIZE = region_size          # count of indices of one scan in a region
        self.NORMAL_LIN_VEL = normal_lin_vel
        self.TRANS_LIN_VEL = trans_lin_vel
        self.TRANS_ANG_VEL = trans_ang_vel
        # Contains the minimum distance for each region
        self.Regions_Min_Distances =  {
            "front_L": self.OBSTACLE_DIST_THRESH, "front_C": self.OBSTACLE_DIST_THRESH, "front_R": self.OBSTACLE_DIST_THRESH,
            "left_L": self.OBSTACLE_DIST_THRESH, "left_C": self.OBSTACLE_DIST_THRESH, "left_R": self.OBSTACLE_DIST_THRESH,
            "right_L": self.OBSTACLE_DIST_THRESH, "right_C": self.OBSTACLE_DIST_THRESH, "right_R": self.OBSTACLE_DIST_THRESH,
            "back_L": self.OBSTACLE_DIST_THRESH, "back_C": self.OBSTACLE_DIST_THRESH, "back_R": self.OBSTACLE_DIST_THRESH
        }

    def has_received_first_scan(self):
        return self.first_scan_received

    def update_regions_with_new_scan(self, smart_scan):
        # Insipred by [2] but adapted to the structure of the Husky's scan results.
        msg = smart_scan.scan
        destination_distance = smart_scan.distance
        
        # Keeps track of the distance measures for each region
        Laser_Scan_Per_Region = {
            "front_L": [], "front_C": [], "front_R": [],
            "left_L": [], "left_C": [], "left_R": [],
            "right_L": [], "right_C": [], "right_R": []
        }

        for i, region in enumerate(self.HUSKY_SCANNERS_REGION_ORDER):
            for scanned_distance in msg.ranges[self.REGION_SIZE*i : self.REGION_SIZE*(i+1)]:
                if scanned_distance != 'inf' and scanned_distance < self.OBSTACLE_DIST_THRESH and scanned_distance <= destination_distance:
                    Laser_Scan_Per_Region[region].append(scanned_distance)
            
            self.Regions_Min_Distances[region] = min(Laser_Scan_Per_Region[region]) if len(Laser_Scan_Per_Region[region]) > 0 else self.OBSTACLE_DIST_THRESH
        
        self.first_scan_received = True
        print("Regions updated with new scan")
        return

    def find_solution_direction(self, goal="front_C"):
        direction, cost = self._choose_direction(goal)
        solution = Solution_direction()
        solution.direction = direction
        solution.cost = cost
        return solution

    def _choose_direction(self, goal="front_C"):
        # The alogirthm idea is taken from [2]. We rewrote this method (originally called _clearance_test)
        # make it more understandable. Also, we now accept a goal as a parameter.
        print("All regions' minimum distance: ", self.Regions_Min_Distances)

        #
        # Let every region's cost be influenced by its neighbors' obstacle cose
        #
        print("All regions' combined costs:")
        minimum = 10e6
        iterator = iter(self.HUSKY_ALL_REGION_ORDER) 
        previous, current, following = self.HUSKY_ALL_REGION_ORDER[-1], next(iterator), next(iterator)   

        # Use the "minimum == 10e6" to identify the first step
        while current != self.HUSKY_ALL_REGION_ORDER[0] or minimum == 10e6: 
            spread_obstacle_cost = (0.2 * self.obstacle_distance_cost_function(previous)) \
            + (0.6 * self.obstacle_distance_cost_function(current)) \
            + (0.2 * self.obstacle_distance_cost_function(following))
            
            change_in_direction_cost = self.direction_cost_function(current, goal)

            combined_cost = self.combined_cost_function(change_in_direction_cost, spread_obstacle_cost)

            print("->  Cost for {region} is {combined_cost:0.5f}({spread_obstacle_cost:0.4f}[{left_c:0.3f}/{center_c:0.3f}/{right_c:0.3f}], {direction_cost:0.5f}) ({previous},{current},{next})"
                .format(region=current, combined_cost=combined_cost, direction_cost=self.DIRECTION_WEIGHT*change_in_direction_cost,
                 previous=previous, current=current, next=following, spread_obstacle_cost=spread_obstacle_cost,
                 left_c=0.2*self.obstacle_distance_cost_function(previous),
                 center_c=0.6*self.obstacle_distance_cost_function(current),
                 right_c=0.2*self.obstacle_distance_cost_function(following)))

            if combined_cost < minimum:
                minimum = combined_cost
                best_region = current
            
            # Move on to the next region
            previous, current, following = current, following, next(iterator, self.HUSKY_ALL_REGION_ORDER[0])

        print("Best Region: ", best_region, " Cost: ", minimum)
        return best_region, minimum

    # Max cost is 1.0; Min cost is 0
    def direction_cost_function(self, region, goal="front_C"):
        return abs(self.REGIONS_CHAGNE_COST[region] - self.REGIONS_CHAGNE_COST[goal])

    # Max cost is 1.0; Min cost is 0
    def obstacle_distance_cost_function(self, region):
        minDist = self.Regions_Min_Distances[region]
        return 15**((self.OBSTACLE_DIST_THRESH - minDist)/(self.OBSTACLE_DIST_THRESH)-1)

    # Max cost is (OBSTACLE_WEIGHT + DIRECTION_WEIGHT); Min cost is 0
    def combined_cost_function(self, direction_cost, obstacle_distance_cost):
        return self.OBSTACLE_WEIGHT * obstacle_distance_cost + self.DIRECTION_WEIGHT * direction_cost