from __future__ import print_function

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance, get_location_in_distance_from_wp
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      Idle,
                                                                      WaypointFollower,
                                                                      ActorTransformSetter,
                                                                      StopVehicle,
                                                                      SyncArrival,
                                                                      ActorSink)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance, InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToVehicle)

import numpy as np

class OccludedCrossingJunction(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=120):
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        self.timeout = timeout

        self._last_vehicle_distance = 12
        self._interval_between_vehicles = 7.5
        self._distance_from_junction = 6
        self._occluder_left_distance = 5
        self._occluder_front_distance = 12
        self._bg_distance_threshold = 7
        self._destory_actors_threshold = 8
        self._occluder_distance_threshold = 6
        self._occluder_max_running_time = 15

        self._bg_vehicle_speed = 5
        self._occluder_speed = 2
        self._ego_vehicle_driven_distance = 30
        self._blocker_waypoints = []

        super(OccludedCrossingJunction, self).__init__("CuttingInStationaryObject",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

        self._traffic_light = CarlaDataProvider.get_next_traffic_light_by_location(config.trigger_points[0].location)

        if self._traffic_light is not None:
            self._traffic_light.set_state(carla.TrafficLightState.Green)
            self._traffic_light.set_green_time(self.timeout)


    def find_straight_waypoint(self, base_waypoint, waypoint_list):
        base_yaw = base_waypoint.transform.rotation.yaw
        min_error = 360
        min_id = None
        for index, waypoint in enumerate(waypoint_list):
            waypoint = waypoint.next(10)[0]
            yaw = waypoint.transform.rotation.yaw
            diff = abs(yaw - base_yaw) % 360
            diff = min(diff, 360 - diff)
            if diff < min_error:
                min_error = diff
                min_id = index
        return waypoint_list[min_id]

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint = self._reference_waypoint
        wp_next = waypoint.get_left_lane()
        wp_next, _ = get_waypoint_in_distance(wp_next, self._last_vehicle_distance)
        while not wp_next.is_intersection:
            _, traveled_distance = get_waypoint_in_distance(wp_next, self._distance_from_junction)
            if traveled_distance < self._distance_from_junction:
                break

            blocker_temp_wp = carla.Transform(carla.Location(wp_next.transform.location.x,
                                                                  wp_next.transform.location.y,
                                                                  wp_next.transform.location.z - 500),
                                                           wp_next.transform.rotation)
            blocker = CarlaDataProvider.request_new_actor('vehicle.carlamotors.carlacola', blocker_temp_wp)
            if blocker is None:
                break
            blocker.set_simulate_physics(enabled=False)
            self.other_actors.append(blocker)
            self._blocker_waypoints.append(wp_next)
            wp_next, traveled_distance = get_waypoint_in_distance(wp_next, self._interval_between_vehicles)
            if traveled_distance < self._interval_between_vehicles / 2:
                break


        # occluder_rotation
        self._occluder_waypoint = self._wmap.get_waypoint(config.other_actors[0].transform.location)
        occluder_location = carla.Location(config.other_actors[0].transform.location.x,
                                           config.other_actors[0].transform.location.y,
                                           config.other_actors[0].transform.location.z - 500)
        occluder_transform = carla.Transform(occluder_location, self._occluder_waypoint.transform.rotation)
        occluder = CarlaDataProvider.request_new_actor('vehicle.audi.tt', occluder_transform)
        occluder.set_simulate_physics(enabled=False)

        self.other_actors.append(occluder)

    def _get_collision_location(self, waypoint1, waypoint2):
        w1_loc = [waypoint1.transform.location.x, waypoint1.transform.location.y]
        w2_loc = [waypoint2.transform.location.x, waypoint2.transform.location.y]

        w1_next = self.find_straight_waypoint(waypoint1, waypoint1.next(1.0))
        w1_next_loc = [w1_next.transform.location.x, w1_next.transform.location.y]
        w1_delta = [w1_next_loc[0] - w1_loc[0], w1_next_loc[1] - w1_loc[1]]
        w2_next = self.find_straight_waypoint(waypoint2, waypoint2.next(1.0))
        w2_next_loc = [w2_next.transform.location.x, w2_next.transform.location.y]
        w2_delta = [w2_next_loc[0] - w2_loc[0], w2_next_loc[1] - w2_loc[1]]

        A = [[w1_delta[0], -w2_delta[0]], [w1_delta[1], -w2_delta[1]]]
        b = [w2_loc[0] - w1_loc[0], w2_loc[1] - w1_loc[1]]
        x = np.linalg.solve(A, b)
        collision_location = carla.Location(w1_loc[0] + x[0] * w1_delta[0], w1_loc[1] + x[0] * w1_delta[1], waypoint1.transform.location.z)
        return collision_location

    def _create_behavior(self):
        """
        After invoking this scenario, vehicle will for the user
        controlled vehicle to enter trigger distance region,
        the vehicle starts crossing the road from the
        stationary vehicles before the junction once the condition meets,
        then after 120 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence()

        sink_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sink_parallel.add_child(Idle(0.5))
        sink_parallel.add_child(ActorSink(self._occluder_waypoint.transform.location, self._destory_actors_threshold))
        for i in range(len(self._blocker_waypoints)):
            sink_parallel.add_child(ActorSink(self._blocker_waypoints[i].transform.location, self._destory_actors_threshold))
        sequence.add_child(sink_parallel)

        start_occluder_transform = ActorTransformSetter(self.other_actors[-1], self._occluder_waypoint.transform)
        sequence.add_child(start_occluder_transform)
        for i in range(len(self._blocker_waypoints)):
            sequence.add_child(ActorTransformSetter(self.other_actors[i], self._blocker_waypoints[i].transform))

        collision_location = self._get_collision_location(self._reference_waypoint, self._occluder_waypoint)
        collision_waypoint = self._wmap.get_waypoint(collision_location)

        bg_stop_location = self._get_collision_location(self._blocker_waypoints[-1], self._occluder_waypoint)
        runningcondition = py_trees.composites.Parallel("running all the vehicles", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        bg = py_trees.composites.Sequence()
        bg_runningcondition = py_trees.composites.Parallel("running the background vehicles",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(len(self._blocker_waypoints)):
            bg_runningcondition.add_child(KeepVelocity(self.other_actors[i], self._bg_vehicle_speed))
        bg_runningcondition.add_child(InTriggerDistanceToLocation(self.other_actors[-2], bg_stop_location, self._bg_distance_threshold))

        bg_stopcondition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(len(self._blocker_waypoints)):
            bg_stopcondition.add_child(StopVehicle(self.other_actors[i], 1.0))

        bg.add_child(bg_runningcondition)
        bg.add_child(bg_stopcondition)

        runningcondition.add_child(bg)
        occluder_runningcondition = py_trees.composites.Parallel("running the occluder vehicles",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        occluder_runningcondition.add_child(SyncArrival(self.other_actors[-1], self.ego_vehicles[0], collision_location, gain=1))
        occluder_runningcondition.add_child(InTriggerDistanceToLocation(self.other_actors[-1], collision_location, self._occluder_distance_threshold))
        occluder_runningcondition.add_child(Idle(self._occluder_max_running_time))
        runningcondition.add_child(occluder_runningcondition)

        sequence.add_child(runningcondition)
        keep_velocity_parallel = py_trees.composites.Parallel("keep velocity for the occluder",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_parallel.add_child(KeepVelocity(self.other_actors[-1], self._occluder_speed))
        keep_velocity_parallel.add_child(DriveDistance(self.ego_vehicles[0], self._ego_vehicle_driven_distance))
        sequence.add_child(keep_velocity_parallel)

        for i in range(len(self.other_actors)):
            sequence.add_child(ActorDestroy(self.other_actors[i]))

        return sequence


    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

