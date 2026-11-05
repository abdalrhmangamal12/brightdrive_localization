#!/usr/bin/env python3
"""Write one OpenSCENARIO file per scenario in config/sim.yaml.

  python3 sim/scenarios/make_scenarios.py        (host or container; CARLA is not needed, only PyYAML)

Reads   config/sim.yaml, config/sensors.yaml (car model + role_name), scenarios/routes/*.yaml (make_route.py)
Writes  sim/scenarios/xosc/<scenario>.xosc     (generated: edit sim.yaml, then run this again)
Each file has parameters you can change at run time without regenerating (ScenarioRunner
--openscenarioparams, or sim/scripts/run.sh ... --param Speed:12):
  Speed       target speed of our car on straights, m/s
  TurnSpeed   target speed where the speed profile says "slow" (turns), m/s
  TimeLimit   stop after this many simulated seconds (100000 = no limit)
  StartDelay  seconds the car stands still before it drives (the estimator's still window for IMU init)
What the .xosc does NOT carry (the bridge, the GNSS gate and the initial-pose publisher read it from
sim.yaml at run time): GNSS noise, the GNSS outage window, the initial-pose offsets.
"""
# Paths relative to this script; maths for headings; random for traffic choices; yaml for the configs.
import math
import random
import sys
from pathlib import Path

import yaml

# sim/ folder (this file is sim/scenarios/make_scenarios.py).
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / 'config'))
import settings  # noqa: E402  (sim/config/settings.py)

# Where ScenarioRunner (inside the container) finds the two drivers: the repo's mount point in the sim container.
CONTROLLERS = '/ws/src/brightdrive_localization/sim/scenarios/controllers'
# A speed-profile / lane-change trigger fires when the car is this close to its route point (waypoints are 5 m apart).
TRIGGER_TOLERANCE_M = 6.0
# ... and must have driven at least (route metres - this) before: rules out an earlier pass of the same road.
TRIGGER_GUARD_M = 80.0
# Car models used for traffic (4-wheel cars of CARLA 0.9.15), taken in turn.
TRAFFIC_MODELS = ['vehicle.tesla.model3', 'vehicle.audi.a2', 'vehicle.toyota.prius', 'vehicle.nissan.micra',
                  'vehicle.mini.cooper_s_2021', 'vehicle.mercedes.coupe_2020', 'vehicle.citroen.c3',
                  'vehicle.ford.mustang', 'vehicle.seat.leon', 'vehicle.mercedes.sprinter']
# Weather presets as OpenSCENARIO values. ScenarioRunner turns them into CARLA weather:
# cloudiness = 100 - 100 * sun intensity; sun altitude = elevation (rad); fog below 1000 m visual range is dense
# fog from that distance on; rain intensity 0.9 = 90 % rain and a wet road.
WEATHER = {
    'clear': {'intensity': 0.85, 'elevation': 1.31, 'visual_range': 100000.0, 'rain': 0.0, 'clock': '12:00:00'},
    'rain': {'intensity': 0.20, 'elevation': 1.31, 'visual_range': 100000.0, 'rain': 0.9, 'clock': '12:00:00'},
    'fog': {'intensity': 0.40, 'elevation': 1.31, 'visual_range': 50.0, 'rain': 0.0, 'clock': '12:00:00'},
    'night': {'intensity': 0.90, 'elevation': -0.40, 'visual_range': 100000.0, 'rain': 0.0, 'clock': '23:00:00'},
}


def world_position(x, y, z, yaw_deg):
    """An OpenSCENARIO position (ROS-style frame, heading in radians)."""
    return f'<WorldPosition x="{x:.2f}" y="{y:.2f}" z="{z:.2f}" h="{math.radians(yaw_deg):.4f}"/>'


def vehicle(name, model, ego):
    """One car in <Entities>. ScenarioRunner uses `name` as the CARLA role_name; type marks our car."""
    kind = 'ego_vehicle' if ego else 'simulation'
    return f'''    <ScenarioObject name="{name}">
      <Vehicle name="{model}" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="69.444" maxAcceleration="200" maxDeceleration="10.0"/>
        <BoundingBox><Center x="1.5" y="0.0" z="0.9"/><Dimensions width="2.1" length="4.5" height="1.8"/></BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
        <Properties><Property name="type" value="{kind}"/></Properties>
      </Vehicle>
    </ScenarioObject>'''


def controller(module_file, properties=None):
    """A ControllerAction: which driver module drives the car (plus its settings as Properties)."""
    props = ''.join(f'<Property name="{k}" value="{v}"/>' for k, v in (properties or {}).items())
    return f'''          <PrivateAction>
            <ControllerAction>
              <AssignControllerAction>
                <Controller name="{Path(module_file).stem}">
                  <Properties><Property name="module" value="{CONTROLLERS}/{module_file}"/>{props}</Properties>
                </Controller>
              </AssignControllerAction>
              <OverrideControllerValueAction>
                <Throttle value="0" active="false"/><Brake value="0" active="false"/>
                <Clutch value="0" active="false"/><ParkingBrake value="0" active="false"/>
                <SteeringWheel value="0" active="false"/><Gear number="0" active="false"/>
              </OverrideControllerValueAction>
            </ControllerAction>
          </PrivateAction>'''


def pick_traffic(spawns, route, start, count, max_dist_m, seed=7):
    """`count` spawn points near the route (within max_dist_m), at least 30 m from our start, 20 m from each
    other."""
    def dist_to_route(p):
        return min(math.hypot(p[0] - w[0], p[1] - w[1]) for w in route)
    near = [s for s in spawns if dist_to_route(s) < max_dist_m and math.hypot(s[0] - start[0], s[1] - start[1]) > 30.0]
    # Same random order every time (seed): the same scenario file every time.
    random.Random(seed).shuffle(near)
    chosen = []
    for s in near:
        # Enough cars already (also stops at once when count is 0).
        if len(chosen) >= count:
            break
        if all(math.hypot(s[0] - c[0], s[1] - c[1]) > 20.0 for c in chosen):
            chosen.append(s)
    return chosen


def speed_action(target):
    """A step to `target` (an OpenSCENARIO expression such as $Speed)."""
    return f'''<PrivateAction><LongitudinalAction><SpeedAction>
                  <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
                  <SpeedActionTarget><AbsoluteTargetSpeed value="{target}"/></SpeedActionTarget>
                </SpeedAction></LongitudinalAction></PrivateAction>'''


def position_trigger(name, hero_name, route_wps, at_m):
    """Start when our car comes within TRIGGER_TOLERANCE_M of the route point `at_m` metres along the route AND
    has already driven at least `at_m` - TRIGGER_GUARD_M metres (both conditions in one ConditionGroup = AND).
    Route metres, not driven metres: the driven distance lags the route by tens of metres (corners are cut, a
    lane change adds length), so a distance condition alone fires late. The distance guard is needed because the
    main route passes some roads twice (the extra block: the lanes of 747-816 m and 1200-1250 m are 3.8 m apart),
    and position alone fired on the first pass."""
    cum = [0.0]
    for a, b in zip(route_wps, route_wps[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    k = min(range(len(cum)), key=lambda i: abs(cum[i] - at_m))
    w = route_wps[k]
    guard = max(at_m - TRIGGER_GUARD_M, 0.0)
    return f'''<StartTrigger><ConditionGroup>
              <Condition name="{name}" delay="0" conditionEdge="rising">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{hero_name}"/></TriggeringEntities>
                  <EntityCondition><ReachPositionCondition tolerance="{TRIGGER_TOLERANCE_M}">
                    <Position>{world_position(*w)}</Position>
                  </ReachPositionCondition></EntityCondition>
                </ByEntityCondition>
              </Condition>
              <Condition name="{name}_driven" delay="0" conditionEdge="rising">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{hero_name}"/></TriggeringEntities>
                  <EntityCondition><TraveledDistanceCondition value="{guard:.1f}"/></EntityCondition>
                </ByEntityCondition>
              </Condition>
            </ConditionGroup></StartTrigger>'''


def maneuver(name, hero_name, event_name, action_xml, trigger_xml):
    """One maneuver with one event: an action and its start trigger. Each extra action gets its own maneuver,
    so it runs beside the route (events inside one maneuver would replace each other)."""
    return f'''          <Maneuver name="{name}">
            <Event name="{event_name}" priority="parallel">
              <Action name="{event_name}">
                {action_xml}
              </Action>
              {trigger_xml}
            </Event>
          </Maneuver>'''


def extra_maneuvers(s, hero_name, route_wps):
    """The speed profile and the lane changes of the scenario, as maneuvers. Distances are route metres from
    the route's first waypoint (the numbers on reserved_reverse.png / in the README lane table)."""
    out = []
    # speed_profile: [[at_m, fast|slow], ...]. "fast" = $Speed, "slow" = $TurnSpeed. The entry at 0 m is the
    # start speed (StartDriving already uses $Speed, so it is skipped unless it says slow).
    for i, (at_m, which) in enumerate(s.get('speed_profile') or []):
        target = {'fast': '$Speed', 'slow': '$TurnSpeed'}[which]
        if at_m <= 0 and which == 'fast':
            continue
        out.append(maneuver(f'SpeedProfile{i:02d}', hero_name, f'Speed{i:02d}_{which}_at_{at_m:.0f}m',
                            speed_action(target), position_trigger(f'At{at_m:.0f}m', hero_name, route_wps, at_m)))
    # lane_changes: [[at_m, left|right, over_m], ...]: a lane change over `over_m` metres (short = sudden).
    for i, (at_m, side, over_m) in enumerate(s.get('lane_changes') or []):
        rel = {'left': 1, 'right': -1}[side]
        action = f'''<PrivateAction><LateralAction><LaneChangeAction>
                  <LaneChangeActionDynamics dynamicsShape="sinusoidal" value="{over_m:.1f}" dynamicsDimension="distance"/>
                  <LaneChangeTarget><RelativeTargetLane entityRef="{hero_name}" value="{rel}"/></LaneChangeTarget>
                </LaneChangeAction></LateralAction></PrivateAction>'''
        out.append(maneuver(f'LaneChange{i:02d}', hero_name, f'LaneChange{i:02d}_{side}_at_{at_m:.0f}m',
                            action, position_trigger(f'At{at_m:.0f}m', hero_name, route_wps, at_m)))
    return '\n'.join(out)


def make_xosc(name, s, cfg, sensors):
    """The whole .xosc text for scenario `name` with settings `s` (defaults already applied)."""
    # The route file: <route>.yaml, or <route>_reverse.yaml for the reverse drive of a closed loop.
    route_file = SIM / 'scenarios' / 'routes' / f'{s["route"]}{"_reverse" if s["reverse"] else ""}.yaml'
    route = yaml.safe_load(open(route_file))
    wps, length, closed = route['waypoints'], route['length_m'], route.get('closed', True)
    # Start pose = one route waypoint. Closed loop: the route continues from the next waypoint, around the loop
    # `laps` times, and ends back at the start. Open route: from the next waypoint to the last one.
    k = s['start_at'] % len(wps)
    start = wps[k]
    if closed:
        drive = (wps[k + 1:] + wps[:k + 1]) * s['laps']
        drive_m = length * s['laps']
    else:
        drive = wps[k + 1:]
        drive_m = length * (len(wps) - 1 - k) / (len(wps) - 1)
    waypoints = '\n'.join(f'                <Waypoint routeStrategy="shortest"><Position>{world_position(*w)}'
                          f'</Position></Waypoint>' for w in drive)
    # Traffic: cars on spawn points near the route, each driven by the Traffic Manager.
    spawns = yaml.safe_load(open(SIM / 'scenarios' / 'routes' / 'spawn_points.yaml'))['spawn_points']
    traffic = pick_traffic(spawns, wps, start, s['traffic'], s['traffic_max_dist_m'])
    hero_name, hero_model = sensors['hero']['role_name'], sensors['hero']['blueprint']
    entities = [vehicle(hero_name, hero_model, ego=True)]
    traffic_init = []
    for i, p in enumerate(traffic):
        car = f'traffic_{i + 1:02d}'
        entities.append(vehicle(car, TRAFFIC_MODELS[i % len(TRAFFIC_MODELS)], ego=False))
        traffic_init.append(f'''        <Private entityRef="{car}">
          <PrivateAction><TeleportAction><Position>{world_position(p[0], p[1], p[2] + 0.3, p[3])}</Position></TeleportAction></PrivateAction>
{controller('tm_autopilot.py')}
        </Private>''')
    w = WEATHER[s['weather']]
    time_limit = s['time_limit_s'] if s['time_limit_s'] > 0 else 100000
    outage = (f'GNSS outage {s["gnss_outage_start_s"]}-{s["gnss_outage_start_s"] + s["gnss_outage_duration_s"]} s'
              if s['gnss_outage_duration_s'] > 0 else 'no GNSS outage')
    init = (f'initial pose offset {s["init_offset_m"]} m / {s["init_heading_offset_deg"]} deg'
            if s.get('init_offset_m') is not None else 'no initial pose published')
    nl = '\n'
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by sim/scenarios/make_scenarios.py from sim/config/sim.yaml: edit those.
     Scenario "{name}": route {route_file.stem} ({length:.0f} m{", closed loop" if closed else ", open"}) x {s["laps"]},
     start at waypoint {k}, still for {s["start_delay_s"]} s, speed {s["speed"]} m/s (turns {s["turn_speed"]} m/s where
     the speed profile says slow), weather {s["weather"]}, {len(traffic)} traffic cars, {len(s.get("lane_changes") or [])} lane changes.
     Read at run time from sim.yaml, not from here: GNSS noise {s["gnss_noise_m"]} m / bias {s["gnss_bias_m"]} m
     (bridge), {outage} (gnss_gate), {init} (initial-pose publisher). -->
<OpenSCENARIO>
  <!-- No "CARLA:" at the start of description: ScenarioRunner then reads positions in the OpenSCENARIO frame
       (y left, like ROS) and flips them for CARLA. With "CARLA:" it would take them as CARLA (y right) values. -->
  <FileHeader revMajor="1" revMinor="0" date="2026-11-01T12:00:00" description="brightdrive {name}" author="brightdrive_localization"/>
  <ParameterDeclarations>
    <ParameterDeclaration name="Speed" parameterType="double" value="{s["speed"]}"/>
    <ParameterDeclaration name="TurnSpeed" parameterType="double" value="{s["turn_speed"]}"/>
    <ParameterDeclaration name="TimeLimit" parameterType="double" value="{time_limit}"/>
    <ParameterDeclaration name="StartDelay" parameterType="double" value="{s["start_delay_s"]}"/>
  </ParameterDeclarations>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="{cfg["town"]}"/>
    <SceneGraphFile filepath=""/>
  </RoadNetwork>
  <Entities>
{nl.join(entities)}
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <GlobalAction>
          <EnvironmentAction>
            <Environment name="{s["weather"]}">
              <TimeOfDay animation="false" dateTime="2026-11-01T{w["clock"]}"/>
              <Weather cloudState="free">
                <Sun intensity="{w["intensity"]}" azimuth="0" elevation="{w["elevation"]}"/>
                <Fog visualRange="{w["visual_range"]}"/>
                <Precipitation precipitationType="{"rain" if w["rain"] > 0 else "dry"}" intensity="{w["rain"]}"/>
              </Weather>
              <RoadCondition frictionScaleFactor="1.0"/>
            </Environment>
          </EnvironmentAction>
        </GlobalAction>
        <Private entityRef="{hero_name}">
          <PrivateAction><TeleportAction><Position>{world_position(start[0], start[1], start[2] + 0.5, start[3])}</Position></TeleportAction></PrivateAction>
{controller('route_agent.py', {'obey_traffic_lights': str(s['obey_traffic_lights']).lower()})}
        </Private>
{nl.join(traffic_init)}
      </Actions>
    </Init>
    <Story name="Drive">
      <Act name="DriveTheRoute">
        <!-- Speed and route are given here, not in <Init>: ScenarioRunner runs a car's Init actions one after the
             other and waits for each, and "reach the speed" would wait for a route that comes after it. -->
        <ManeuverGroup name="HeroDrives" maximumExecutionCount="1">
          <Actors selectTriggeringEntities="false"><EntityRef entityRef="{hero_name}"/></Actors>
          <Maneuver name="Drive">
            <Event name="StartDriving" priority="overwrite">
              <!-- The two actions of one event run at the same time. The car stands still until StartDelay. -->
              <Action name="SetSpeed">
                {speed_action('$Speed')}
              </Action>
              <Action name="FollowRoute">
                <PrivateAction><RoutingAction><AssignRouteAction>
                  <Route name="{route_file.stem}" closed="false">
{waypoints}
                  </Route>
                </AssignRouteAction></RoutingAction></PrivateAction>
              </Action>
              <StartTrigger><ConditionGroup><Condition name="AfterStartDelay" delay="0" conditionEdge="rising">
                <ByValueCondition><SimulationTimeCondition value="$StartDelay" rule="greaterThan"/></ByValueCondition>
              </Condition></ConditionGroup></StartTrigger>
            </Event>
          </Maneuver>
{extra_maneuvers(s, hero_name, wps)}
        </ManeuverGroup>
        <StartTrigger><ConditionGroup><Condition name="AtStart" delay="0" conditionEdge="rising">
          <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
        </Condition></ConditionGroup></StartTrigger>
        <StopTrigger>
          <ConditionGroup><Condition name="RouteDone" delay="0" conditionEdge="rising">
            <ByEntityCondition>
              <TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="{hero_name}"/></TriggeringEntities>
              <EntityCondition><TraveledDistanceCondition value="{max(drive_m - 15.0, 10.0):.1f}"/></EntityCondition>
            </ByEntityCondition>
          </Condition></ConditionGroup>
          <ConditionGroup><Condition name="TimeUp" delay="0" conditionEdge="rising">
            <ByValueCondition><SimulationTimeCondition value="$TimeLimit" rule="greaterThan"/></ByValueCondition>
          </Condition></ConditionGroup>
        </StopTrigger>
      </Act>
    </Story>
    <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>
'''


def main():
    cfg = settings.config()
    sensors = yaml.safe_load(open(SIM / 'config' / 'sensors.yaml'))
    out = SIM / 'scenarios' / 'xosc'
    out.mkdir(parents=True, exist_ok=True)
    required, extras = settings.names(cfg)
    # Files of scenarios that no longer exist in the YAML are removed: the folder mirrors the YAML.
    for old in out.glob('*.xosc'):
        if old.stem not in required + extras:
            old.unlink()
            print(f'{old.stem:18s} removed (not in sim.yaml)')
    for name in required + extras:
        (out / f'{name}.xosc').write_text(make_xosc(name, settings.load(name, cfg), cfg, sensors))
        print(f'{name:18s} -> {out / (name + ".xosc")}{"   (extra)" if name in extras else ""}')


if __name__ == '__main__':
    main()
