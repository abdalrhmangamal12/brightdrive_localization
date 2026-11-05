"""Driver for traffic cars, loaded by ScenarioRunner through the .xosc:
  <Property name="module" value=".../controllers/tm_autopilot.py"/>
It hands the car to CARLA's Traffic Manager, which drives it around the town, keeping lanes and distances.
(ScenarioRunner 0.9.15 has a carla_autopilot module for this, but it cannot be loaded: its class name does not
match its file name. ScenarioRunner turns the file name tm_autopilot into the class name TmAutopilot.)
"""
# ScenarioRunner's base class for drivers, and the Traffic Manager port it uses.
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class TmAutopilot(BasicControl):
    def __init__(self, actor, args=None):
        # The base class stores the car as self._actor.
        super().__init__(actor)
        # Switch on the autopilot, on the same Traffic Manager ScenarioRunner set to synchronous mode.
        self._actor.set_autopilot(True, CarlaDataProvider.get_traffic_manager_port())

    def run_step(self):
        # Nothing to do: the Traffic Manager drives the car on every tick.
        pass

    def reset(self):
        # End of the scenario: switch the autopilot off if the car still exists.
        if self._actor is not None and self._actor.is_alive:
            self._actor.set_autopilot(False, CarlaDataProvider.get_traffic_manager_port())
