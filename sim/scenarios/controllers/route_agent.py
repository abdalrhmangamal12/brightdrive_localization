"""Driver for our car, loaded by ScenarioRunner through the .xosc:
  <Property name="module" value=".../controllers/route_agent.py"/>
It follows the route waypoints of the .xosc (AssignRouteAction) at the .xosc target speed (SpeedAction), using
CARLA's BasicAgent, which also brakes for cars ahead (ScenarioRunner's own lane follower does not).
ScenarioRunner turns the file name route_agent into the class name RouteAgent, so keep the two in step.

Waypoint lists arrive through update_waypoints(). The first one is the route. A later, short one is a maneuver
(a LaneChangeAction: ScenarioRunner sends ~5 m of the current lane, the lane change, ~10 m of the new lane):
the car drives it, then rejoins the route at the route waypoint nearest to the maneuver's end. A later, long
one (ScenarioRunner's "keep following the new lane" list after a lane change) is ignored: the route continues.
"""
# CARLA's ready-made driving agent and the road-option type its plans use.
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption
# ScenarioRunner's base class for drivers, and its access to the CARLA map.
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

# A waypoint list with at most this many entries, arriving while a route is active, is a maneuver.
# ScenarioRunner's lane-change plans are 2 m apart: 60 entries = 120 m. Routes are hundreds of entries.
MANEUVER_MAX_WAYPOINTS = 60


class RouteAgent(BasicControl):
    """ScenarioRunner calls: __init__ once, update_target_speed / update_waypoints when the .xosc says so,
    run_step every tick, reset at the end."""

    def __init__(self, actor, args=None):
        # The base class stores the car as self._actor and sets _waypoints, _target_speed, ...
        super().__init__(actor)
        # The <Properties> of the controller in the .xosc arrive here as strings.
        args = args or {}
        obey_lights = args.get('obey_traffic_lights', 'false') == 'true'
        # BasicAgent: target speed in km/h (set every step below), uses the map ScenarioRunner already loaded.
        self._agent = BasicAgent(actor, target_speed=0.0, map_inst=CarlaDataProvider.get_map(),
                                 opt_dict={'ignore_traffic_lights': not obey_lights,
                                           'ignore_stop_signs': not obey_lights})
        # The route as the agent's plan (road waypoint, how to drive), kept to rejoin it after a maneuver.
        self._route = None

    def _to_plan(self, transforms):
        """carla.Transforms -> the agent's plan: road waypoints with a "how to drive" tag."""
        town_map = CarlaDataProvider.get_map()
        return [(town_map.get_waypoint(t.location), RoadOption.LANEFOLLOW) for t in transforms]

    def _rejoin_index(self, location):
        """Index of the route waypoint nearest to `location`, so the route can continue from there."""
        return min(range(len(self._route)),
                   key=lambda i: self._route[i][0].transform.location.distance(location))

    def run_step(self):
        # New waypoints from the .xosc: the route, a maneuver, or ScenarioRunner's follow-up list.
        if self._waypoints_updated:
            self._waypoints_updated = False
            if self._route is None:
                self._route = self._to_plan(self._waypoints)
                self._agent.set_global_plan(self._route, stop_waypoint_creation=True, clean_queue=True)
            elif not self._waypoints:
                # ScenarioRunner could not plan the maneuver here (it prints "couldn't perform the expected lane
                # change": e.g. no lane to change into at the change point) and sends an empty list: the route goes on.
                print('route_agent: empty maneuver (ScenarioRunner could not plan it here): the route continues',
                      flush=True)
            elif len(self._waypoints) <= MANEUVER_MAX_WAYPOINTS:
                # The maneuver, then the rest of the route from the route point nearest to the maneuver's end.
                maneuver = self._to_plan(self._waypoints)
                rest = self._route[self._rejoin_index(self._waypoints[-1].location) + 1:]
                self._agent.set_global_plan(maneuver + rest, stop_waypoint_creation=True, clean_queue=True)
                print(f'route_agent: maneuver of {len(maneuver)} waypoints, then the route from index '
                      f'{len(self._route) - len(rest)}', flush=True)
            # else: a long list while the route is active = ScenarioRunner's "follow the new lane": ignored.
        # The .xosc speed is m/s, the agent wants km/h.
        self._agent.set_target_speed(self._target_speed * 3.6)
        # One step of driving: the agent returns throttle, steering and brake; apply them to the car.
        self._actor.apply_control(self._agent.run_step())
        # Tell ScenarioRunner when the route is finished.
        self._reached_goal = self._agent.done()

    def reset(self):
        # End of the scenario: let go of the car.
        self._agent = None
