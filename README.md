# BrightDrive — Localization & Mapping

CARLA 0.9.15 · Town03 · ROS 2 Humble · C++

| Component | Status | Folder |
|---|---|---|
| Error-state Kalman filter (IMU + GNSS), pure C++ | in progress | [`eskf/`](eskf/README.md) |

## Simulation

CARLA 0.9.15 server and a ROS 2 Humble client, both in Docker: `compose.yaml` (all services), `compose.viz.yaml`
(optional window), `.env` (`UID`/`GID`, `ROS_DOMAIN_ID`, `BAGS_DIR`, `MAPS_DIR`), `docker/`. Needs Docker Compose v2,
the NVIDIA container runtime (`docker info | grep -i runtimes` lists `nvidia`), an NVIDIA driver (580.x here) and
the server image (`docker pull carlasim/carla:0.9.15`, 21 GB).

| Service | What |
|---|---|
| `carla-server` | `carlasim/carla:0.9.15` pinned by digest, GPU through the `nvidia` runtime, host network, `-RenderOffScreen -quality-level=Epic`, healthcheck on the RPC port 2000 |
| `ros` | built from `docker/Dockerfile`: Humble + numpy from apt + the `carla==0.9.15` wheel. Host network + IPC, runs as your uid/gid, repo at `/ws/src/brightdrive_localization`, bags at `/ws/bags`, idle (`sleep infinity`) until you exec into it |

Every command below runs from the repo root.

### Build

```
docker compose build ros
```

### Start

```
docker compose up -d ros         # starts carla-server first and waits until it is healthy
docker compose ps                # both Up, carla-server (healthy)
```

The server shows `(health: starting)` until the RPC port answers: 5 s on this laptop (RTX 4060 8 GB) with warm
shader caches, up to 3 minutes after a reboot or a fresh pull, while Town10HD_Opt loads.

### Exec and check

```
docker compose exec ros bash                                   # a shell in the client container
docker compose exec ros python3 /ws/src/brightdrive_localization/sim/scripts/check_client.py
```

`sim/scripts/check_client.py`: client and server versions must match, loads Town03 and prints the load time,
100 synchronous ticks at 0.01 s with the real-time factor, restores asynchronous mode in a `finally`.
`exec` skips the image entrypoint: `source /opt/ros/humble/setup.bash` in that shell before `ros2` commands.

Server-side checks:

```
docker compose logs --tail 20 carla-server          # UE4 version line, no "Signal 11"
bash -c 'echo > /dev/tcp/127.0.0.1/2000' && echo "port 2000 open"
nvidia-smi | grep CarlaUE4                          # the server holds the GPU
```

### Stop

```
docker compose down          # all containers; or: docker compose stop ros
```

### Windowed (to look at the town)

```
docker compose -f compose.yaml -f compose.viz.yaml up -d carla-server
```

`compose.viz.yaml` overrides the command (no `-RenderOffScreen`, a 1280x720 window) and passes the host display
through. Not loaded by default. If the window does not appear: `xhost +SI:localuser:$USER`, then restart.

### Known behaviour

- `-quality-level=Low` segfaults on `load_world('Town03')` on this machine (exit 139); Epic is the setting that works.
- `restart: "no"`: a crashed server stays visible as `Exited (139)` in `docker compose ps -a`. Read
  `docker compose logs carla-server`, then `docker compose up -d carla-server` again.
- The server keeps running between runs; stop it only when you are done (a restart costs the startup time).
