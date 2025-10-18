from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """
        Generates a curved path that continues straight after passing its target.

        The logic is:
        1. Determine a single target point from the visible cones.
        2. Generate the path step-by-step, curving towards the target.
        3. **Once the target is behind the current path point, stop turning and
           continue generating the path in a straight line.**
        """
        # --- Constants ---
        PATH_LENGTH = 15.0
        STEP_SIZE = 0.5
        TRACK_WIDTH = 3.5
        TURN_AGGRESSION = 0.9
        
        num_points = int(PATH_LENGTH / STEP_SIZE)
        
        # 1. Separate cones by color
        yellow_cones = []
        blue_cones = []
        for cone in self.cones:
            if cone.color == 0:
                yellow_cones.append(cone)
            elif cone.color == 1:
                blue_cones.append(cone)

        # 2. Calculate the center point (centroid) for each side
        yellow_center: Tuple[float, float] | None = None
        if yellow_cones:
            avg_x = sum(c.x for c in yellow_cones) / len(yellow_cones)
            avg_y = sum(c.y for c in yellow_cones) / len(yellow_cones)
            yellow_center = (avg_x, avg_y)

        blue_center: Tuple[float, float] | None = None
        if blue_cones:
            avg_x = sum(c.x for c in blue_cones) / len(blue_cones)
            avg_y = sum(c.y for c in blue_cones) / len(blue_cones)
            blue_center = (avg_x, avg_y)

        # 3. Determine a single target point to aim for
        target_point: Tuple[float, float] | None = None
        if blue_center and yellow_center:
            target_x = (blue_center[0] + yellow_center[0]) / 2
            target_y = (blue_center[1] + yellow_center[1]) / 2
            target_point = (target_x, target_y)
        elif blue_center:
            vec_x = blue_center[0] - self.car_pose.x
            vec_y = blue_center[1] - self.car_pose.y
            right_vec_x, right_vec_y = vec_y, -vec_x
            mag = math.hypot(right_vec_x, right_vec_y)
            if mag > 0:
                offset_x = (right_vec_x / mag) * (TRACK_WIDTH / 2)
                offset_y = (right_vec_y / mag) * (TRACK_WIDTH / 2)
                target_point = (blue_center[0] + offset_x, blue_center[1] + offset_y)
        elif yellow_center:
            vec_x = yellow_center[0] - self.car_pose.x
            vec_y = yellow_center[1] - self.car_pose.y
            left_vec_x, left_vec_y = -vec_y, vec_x
            mag = math.hypot(left_vec_x, left_vec_y)
            if mag > 0:
                offset_x = (left_vec_x / mag) * (TRACK_WIDTH / 2)
                offset_y = (left_vec_y / mag) * (TRACK_WIDTH / 2)
                target_point = (yellow_center[0] + offset_x, yellow_center[1] + offset_y)
        
        # 4. Generate the path step-by-step
        cx, cy, yaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
        
        path: Path2D = [(cx, cy)]
        
        current_x, current_y = cx, cy
        current_dx, current_dy = math.cos(yaw), math.sin(yaw)
        
        target_is_passed = False # Flag to lock the direction

        for _ in range(1, num_points):
            if target_point and not target_is_passed:
                vec_to_target_x = target_point[0] - current_x
                vec_to_target_y = target_point[1] - current_y
                
                # *** KEY CHANGE IS HERE ***
                # Check if the target is behind the current direction using a dot product.
                # If the dot product is negative, the angle is > 90 degrees, so we've passed it.
                dot_product = current_dx * vec_to_target_x + current_dy * vec_to_target_y
                if dot_product < 0:
                    target_is_passed = True

                # Only adjust the curve if we haven't passed the target yet
                if not target_is_passed:
                    mag = math.hypot(vec_to_target_x, vec_to_target_y)
                    if mag > 0:
                        vec_to_target_x /= mag
                        vec_to_target_y /= mag
                        
                        current_dx = (1 - TURN_AGGRESSION) * current_dx + TURN_AGGRESSION * vec_to_target_x
                        current_dy = (1 - TURN_AGGRESSION) * current_dy + TURN_AGGRESSION * vec_to_target_y

                        new_mag = math.hypot(current_dx, current_dy)
                        if new_mag > 0:
                            current_dx /= new_mag
                            current_dy /= new_mag
            
            # Move one step in the current (now fixed, if target is passed) direction
            next_x = current_x + current_dx * STEP_SIZE
            next_y = current_y + current_dy * STEP_SIZE
            path.append((next_x, next_y))
            
            current_x, current_y = next_x, next_y
            
        return path