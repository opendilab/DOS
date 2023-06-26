
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
                                                                      ActorTransformSetter,
                                                                      ActorSink)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance)

class CuttingInStationaryObject(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self._wmap = CarlaDataProvider.get_map()
        print(config)
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        self.timeout = timeout
        self.first_vehicle_distance = 15.75
        self.second_vehicle_distance = 23
        self.pedestrian_distance = 19.25
        self._destory_actors_threshold = 8
        self._num_lane_changes = 0
        self._adversary_type = 'walker.*'
        self._adversary_speed = 3.0  # Speed of the adversary [m/s]
        self._reaction_time = 0.8  # Time the agent has to react to avoid the collision [s]
        self._collision_wp = None

        self._reaction_ratio = 0.12  # The higehr the number of lane changes, the smaller the reaction time
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(CuttingInStationaryObject, self).__init__("CuttingInStationaryObject",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

        self._traffic_light = CarlaDataProvider.get_next_traffic_light_by_location(config.trigger_points[0].location)

        if self._traffic_light is not None:
            self._traffic_light.set_state(carla.TrafficLightState.Green)
            self._traffic_light.set_green_time(self.timeout)


    def _get_sidewalk_transform(self, waypoint, offset):
        """
        Processes the waypoint transform to find a suitable spawning one at the sidewalk.
        It first rotates the transform so that it is pointing towards the road and then moves a
        bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
        """

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += offset['yaw']

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            right_vector = waypoint.transform.get_right_vector()
            offset_dist = waypoint.lane_width * offset["k"]
            offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += offset['z']

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint = self._reference_waypoint
        wp_next = waypoint.get_right_lane()

        self.wp_first_vehicle, _ = get_waypoint_in_distance(wp_next, self.first_vehicle_distance)
        self.wp_second_vehicle, _ = get_waypoint_in_distance(wp_next, self.second_vehicle_distance)
        self.wp_pedestrian, _ = get_waypoint_in_distance(wp_next, self.pedestrian_distance)
        self._collision_wp, _ = get_waypoint_in_distance(self._reference_waypoint, self.pedestrian_distance)

        sidewalk_waypoint = self.wp_pedestrian
        while sidewalk_waypoint.lane_type != carla.LaneType.Sidewalk:
            right_wp = sidewalk_waypoint.get_right_lane()
            if right_wp is None:
                break
            sidewalk_waypoint = right_wp
            self._num_lane_changes += 1


        offset = {'yaw': 270, 'z': 0.5, 'k': 1.0}
        self._adversary_transform = self._get_sidewalk_transform(sidewalk_waypoint, offset)

        adversary_temp_transform = carla.Transform(carla.Location(self.wp_pedestrian.transform.location.x,
                                                                  self.wp_pedestrian.transform.location.y,
                                                                  self.wp_pedestrian.transform.location.z - 500),
                                                   self.wp_pedestrian.transform.rotation)

        adversary = CarlaDataProvider.request_new_actor(self._adversary_type, adversary_temp_transform)
        first_vehicle_temp_transform = carla.Transform(carla.Location(self.wp_first_vehicle.transform.location.x,
                                                                  self.wp_first_vehicle.transform.location.y,
                                                                  self.wp_first_vehicle.transform.location.z - 500),
                                                   self.wp_first_vehicle.transform.rotation)

        second_vehicle_temp_transform = carla.Transform(carla.Location(self.wp_second_vehicle.transform.location.x,
                                                                  self.wp_second_vehicle.transform.location.y,
                                                                  self.wp_second_vehicle.transform.location.z - 500),
                                                   self.wp_second_vehicle.transform.rotation)

        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.carlamotors.carlacola', first_vehicle_temp_transform)
        second_vehicle = CarlaDataProvider.request_new_actor('vehicle.carlamotors.carlacola', second_vehicle_temp_transform)

        adversary.set_simulate_physics(enabled=False)
        first_vehicle.set_simulate_physics(enabled=False)
        second_vehicle.set_simulate_physics(enabled=False)

        self.other_actors.append(adversary)
        self.other_actors.append(first_vehicle)
        self.other_actors.append(second_vehicle)


    def _create_behavior(self):
        """
        After invoking this scenario, pedestrian will for the user
        controlled vehicle to enter trigger distance region,
        the pedestrian starts crossing the road from the
        stationary vehicles once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence()

        collision_location = self._collision_wp.transform.location
        collision_distance = collision_location.distance(self._adversary_transform.location)
        collision_duration = collision_distance / self._adversary_speed
        reaction_time = self._reaction_time - self._reaction_ratio * self._num_lane_changes
        collision_time_trigger = collision_duration + reaction_time

        sink_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sink_parallel.add_child(Idle(0.5))
        sink_parallel.add_child(ActorSink(self._adversary_transform.location, self._destory_actors_threshold))
        sink_parallel.add_child(ActorSink(self.wp_first_vehicle.transform.location, self._destory_actors_threshold))
        sink_parallel.add_child(ActorSink(self.wp_second_vehicle.transform.location, self._destory_actors_threshold))
        sequence.add_child(sink_parallel)

        start_pedestrian_transform = ActorTransformSetter(self.other_actors[0], self._adversary_transform)
        start_first_vehicle_transform = ActorTransformSetter(self.other_actors[1], self.wp_first_vehicle.transform)
        start_second_vehicle_transform = ActorTransformSetter(self.other_actors[2], self.wp_second_vehicle.transform)

        sequence.add_child(start_pedestrian_transform)
        sequence.add_child(start_first_vehicle_transform)
        sequence.add_child(start_second_vehicle_transform)

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")

        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], collision_time_trigger, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary
        speed_duration = 2.0 * collision_duration
        speed_distance = 2.0 * collision_distance
        sequence.add_child(KeepVelocity(
            self.other_actors[0], self._adversary_speed,
            duration=speed_duration, distance=speed_distance, name="AdversaryCrossing"))

        # Remove everything
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyBlocker"))
        sequence.add_child(ActorDestroy(self.other_actors[2], name="DestroyBlocker"))

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
