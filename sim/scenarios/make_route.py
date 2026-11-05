#!/usr/bin/env python3
"""Turn the few route points in config/sim.yaml into full routes along the roads, and draw the town with a
coordinate grid and the routes, so you can choose and check the points. Needs the server (sim container):

  docker compose exec sim python3 -u /ws/src/brightdrive_localization/sim/scenarios/make_route.py

Writes (generated, do not edit by hand):
  sim/scenarios/routes/<route>.yaml           waypoints every 5 m, length, in the ROS map frame
  sim/scenarios/routes/<route>_reverse.yaml   the same loop driven the other way (its own lanes);
                                              only for closed routes
  sim/scenarios/routes/spawn_points.yaml      the town's spawn points (where traffic cars can start)
  sim/scenarios/routes/<town>.png             roads + 50 m grid + the routes and their numbered points
  sim/scenarios/routes/reserved_reverse.png   the main route with its reserved section (the waypoint
                                                     range `reserved_reverse` of the route) drawn in black

A route is a closed loop (back to its first point) unless it says `closed: false`: then it ends at its last
point and no _reverse file is made (the reverse_reuse scenario is such an open route).
"""
# File paths relative to this script, so it runs from any folder.
from pathlib import Path

# CARLA's Python API, and CARLA's route planner (finds the road path between two places).
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
# numpy for coordinates, PIL to draw the picture, yaml to read and write config files.
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

# sim/ folder (this file is sim/scenarios/make_route.py).
SIM = Path(__file__).resolve().parents[1]
# Keep one waypoint every 5 m: dense enough to follow, small enough to read.
SPACING_M = 5.0
# Picture scale: pixels per metre.
PX_PER_M = 2.0
# One colour per route, in the order they appear in sim.yaml.
COLOURS = [(220, 60, 50), (40, 110, 220), (30, 160, 90), (200, 130, 0)]


def font(size):
    """A readable font if the system has it, else PIL's small built-in one."""
    try:
        return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def snap(town_map, point, toward, across_median=False):
    """CARLA location of the lane centre nearest to `point` (ROS x, y) whose driving direction points toward
    `toward` (the next point). Without this, a point on a two-way road may land on the opposite lane.
    Candidates: the nearest driving lane and its two neighbours. With `across_median` (a route's
    `snap_across_median: true`), when none of them drives toward `toward` (a road with a median:
    get_left_lane() never crosses it), the lanes up to 3 lanes (10.5 m) to either side are searched too.
    Off by default: the main route was planned without it and its shape is what every scenario uses."""
    # Direction to the next point, in CARLA axes (ROS y is CARLA's -y).
    bearing = np.array([toward[0] - point[0], -(toward[1] - point[1])], dtype=float)
    bearing /= max(np.linalg.norm(bearing), 1e-6)

    def agreement(lane):
        f = lane.transform.get_forward_vector()
        return float(np.dot([f.x, f.y], bearing))

    # Nearest driving lane (ROS y is CARLA's -y), then its neighbours.
    wp = town_map.get_waypoint(carla.Location(x=point[0], y=-point[1]), project_to_road=True,
                               lane_type=carla.LaneType.Driving)
    lanes = [lane for lane in (wp, wp.get_left_lane(), wp.get_right_lane())
             if lane is not None and lane.lane_type == carla.LaneType.Driving]
    best = max(lanes, key=agreement)
    if agreement(best) > 0.3 or not across_median:
        return best.transform.location
    # Nothing nearby drives that way: look across the median.
    side = np.array([-bearing[1], bearing[0]])
    across = {}
    for k in range(-3, 4):
        loc = carla.Location(x=float(point[0] + k * 3.5 * side[0]), y=float(-point[1] + k * 3.5 * side[1]))
        lane = town_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if lane is not None:
            across[(lane.road_id, lane.lane_id)] = lane
    best = max(across.values(), key=agreement)
    print(f'  point {point}: no lane next to it drives toward {toward}; '
          f'using road {best.road_id} lane {best.lane_id} across the median')
    return best.transform.location


def plan_route(grp, town_map, points, closed=True, across_median=False):
    """Road path through the points (ROS x, y), and back to the first one if `closed`.
    Returns (N, 4) x, y, z, yaw in ROS."""
    path = []
    # Snap every chosen point onto the lane that drives toward the following point (the last point of an open
    # route: toward itself from the one before, i.e. keep the direction of the last leg).
    nxt = points[1:] + ([points[0]] if closed else [points[-1]])
    prv = [points[-1]] + points[:-1]
    lanes = [snap(town_map, p, q if q != p else (2 * p[0] - r[0], 2 * p[1] - r[1]), across_median)
             for p, q, r in zip(points, nxt, prv)]
    # Each leg: point i -> point i+1, and (closed loop) the last point -> the first point.
    legs = list(zip(lanes, lanes[1:] + ([lanes[0]] if closed else [])))
    for a, b in legs:
        leg = grp.trace_route(a, b)
        # Each element is (waypoint on the road, how to take the next junction); keep the road poses.
        for waypoint, _ in leg:
            t = waypoint.transform
            # Back to ROS: flip y and yaw.
            path.append((t.location.x, -t.location.y, t.location.z, -t.rotation.yaw))
    path = np.array(path)
    # Thin out to one waypoint every SPACING_M metres along the path.
    keep, travelled = [0], 0.0
    for i in range(1, len(path)):
        travelled += np.linalg.norm(path[i, :2] - path[i - 1, :2])
        if travelled >= SPACING_M:
            keep.append(i)
            travelled = 0.0
    return path[keep]


def main():
    # Read the routes and the town from the shared config.
    cfg = yaml.safe_load(open(SIM / 'config' / 'sim.yaml'))
    out_dir = SIM / 'scenarios' / 'routes'
    out_dir.mkdir(parents=True, exist_ok=True)

    # Connect; load the town only if a different one is loaded now.
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world = client.get_world()
    if world.get_map().name.split('/')[-1] != cfg['town']:
        world = client.load_world(cfg['town'])
    town_map = world.get_map()
    # The planner works on a graph of the town's lanes, sampled every 2 m.
    grp = GlobalRoutePlanner(town_map, 2.0)

    # Plan every route that has at least 2 points and save its waypoints.
    routes = {}
    for name, r in cfg['routes'].items():
        if len(r['points']) < 2:
            print(f'route {name}: fewer than 2 points, skipped')
            continue
        closed = r.get('closed', True)
        # Forward: the points in the order written. Reverse (closed loops only): the same points backwards,
        # planned again so the car uses the lanes of the other direction (reversing the waypoint list would
        # drive against traffic).
        variants = [('', r['points'])] + ([('_reverse', r['points'][::-1])] if closed else [])
        for suffix, points in variants:
            wp = plan_route(grp, town_map, [tuple(p) for p in points], closed, r.get('snap_across_median', False))
            length = float(np.linalg.norm(np.diff(wp[:, :2], axis=0), axis=1).sum())
            routes[name + suffix] = (points, wp, length)
            # Saved file: town, length and the waypoints as [x, y, z, yaw_deg] lists.
            yaml.safe_dump({'town': cfg['town'], 'length_m': round(length, 1), 'spacing_m': SPACING_M,
                            'closed': closed,
                            'waypoints': [[round(float(v), 2) for v in row] for row in wp]},
                           open(out_dir / f'{name}{suffix}.yaml', 'w'), default_flow_style=None, sort_keys=False)
            print(f'route {name}{suffix}: {len(points)} points -> {len(wp)} waypoints, {length:.0f} m')

    # The town's spawn points in the ROS map frame: make_scenarios.py starts traffic cars on them.
    spawns = [[round(t.location.x, 2), round(-t.location.y, 2), round(t.location.z, 2), round(-t.rotation.yaw, 1)]
              for t in town_map.get_spawn_points()]
    yaml.safe_dump({'town': cfg['town'], 'spawn_points': spawns}, open(out_dir / 'spawn_points.yaml', 'w'),
                   default_flow_style=None)

    # ---- the pictures: roads, grid, routes; then the reserved section ----
    # All road centre lines every 2 m, in ROS coordinates.
    roads = np.array([(w.transform.location.x, -w.transform.location.y) for w in town_map.generate_waypoints(2.0)])
    draw_routes(cfg, routes, roads, out_dir / f'{cfg["town"]}.png')
    draw_reserved(cfg, routes, roads, out_dir / 'reserved_reverse.png')


def draw_routes(cfg, routes, roads, out_file, reserved=None):
    """The town picture. `reserved` = (route name, i0, i1): that waypoint range is drawn thick and black."""
    lo, hi = roads.min(0) - 30, roads.max(0) + 30
    W, H = ((hi - lo) * PX_PER_M).astype(int)
    img = Image.new('RGB', (W, H + 60), (250, 250, 248))
    d = ImageDraw.Draw(img)

    def px(x, y):
        """ROS map metres -> picture pixels (x to the right, y up)."""
        return (x - lo[0]) * PX_PER_M, 60 + (hi[1] - y) * PX_PER_M

    # Grid every 50 m, labelled with the coordinate to type into sim.yaml.
    for gx in range(int(np.ceil(lo[0] / 50) * 50), int(hi[0]), 50):
        d.line([px(gx, lo[1]), px(gx, hi[1])], fill=(215, 215, 215), width=1)
        d.text((px(gx, hi[1])[0] + 2, 62), f'x={gx}', fill=(120, 120, 120), font=font(13))
    for gy in range(int(np.ceil(lo[1] / 50) * 50), int(hi[1]), 50):
        d.line([px(lo[0], gy), px(hi[0], gy)], fill=(215, 215, 215), width=1)
        d.text((2, px(lo[0], gy)[1] + 2), f'y={gy}', fill=(120, 120, 120), font=font(13))
    # Roads as grey dots.
    for x, y in roads:
        cx, cy = px(x, y)
        d.ellipse([cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5], fill=(150, 150, 150))
    # Each route: a coloured line (closed back to its start if it is a loop), its chosen points numbered 1, 2, 3 ...
    for k, (name, (points, wp, _)) in enumerate(routes.items()):
        col = COLOURS[k % len(COLOURS)]
        closed = cfg['routes'].get(name.replace('_reverse', ''), {}).get('closed', True)
        d.line([px(x, y) for x, y in wp[:, :2]] + ([px(*wp[0, :2])] if closed else []), fill=col, width=4)
        for i, (x, y) in enumerate(points, 1):
            cx, cy = px(x, y)
            d.ellipse([cx - 11, cy - 11, cx + 11, cy + 11], fill=col)
            d.text((cx - 5, cy - 9), str(i), fill=(255, 255, 255), font=font(15))
    legend = '   '.join(f'{name}: {length:.0f} m' for name, (_, _, length) in routes.items()) or 'no routes yet'
    title = f'{cfg["town"]} roads (ROS map frame: x right, y up), grid 50 m.   Routes: {legend}'
    if reserved is not None:
        name, i0, i1 = reserved
        wp = routes[name][1]
        d.line([px(x, y) for x, y in wp[i0:i1 + 1, :2]], fill=(0, 0, 0), width=9)
        for i in (i0, i1):
            cx, cy = px(*wp[i, :2])
            d.text((cx + 12, cy - 8), f'wp {i}', fill=(0, 0, 0), font=font(16))
        seg = float(np.linalg.norm(np.diff(wp[i0:i1 + 1, :2], axis=0), axis=1).sum())
        title = (f'{cfg["town"]}: reserved section = route {name} waypoints {i0}..{i1} ({seg:.0f} m, black); '
                 f'mapped forward only, reverse_reuse drives it backwards')
    d.text((10, 10), title, fill=(30, 30, 30), font=font(20))
    img.save(out_file)
    print(f'picture: {out_file} ({W}x{H + 60})')


def draw_reserved(cfg, routes, roads, out_file):
    """The main route with its reserved section (sim.yaml: routes.<name>.reserved_reverse: [i0, i1])."""
    for name, r in cfg['routes'].items():
        if 'reserved_reverse' in r and name in routes:
            i0, i1 = r['reserved_reverse']
            draw_routes(cfg, {name: routes[name]}, roads, out_file, reserved=(name, i0, i1))
            return
    print('no reserved_reverse range in sim.yaml: reserved_reverse.png not drawn')


if __name__ == '__main__':
    main()
