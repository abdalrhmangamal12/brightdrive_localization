# BrightDrive — Localization & Mapping

CARLA 0.9.15 · Town03 · ROS 2 Humble · C++

| Component | Status | Folder |
|---|---|---|
| Error-state Kalman filter (IMU + GNSS), pure C++ | in progress | [`eskf/`](eskf/README.md) |

## Simulation

CARLA 0.9.15 server and a ROS 2 Humble client, both in Docker: `compose.yaml` (all services), `compose.viz.yaml`
(optional window), `.env` (`UID`/`GID`, `BD_ROS_DOMAIN_ID`, `BAGS_DIR`, `MAPS_DIR`), `docker/`. ROS containers share `x-ros-env` in
`compose.yaml`: domain 42, localhost only, Fast DDS with the profile in `docker/fastdds_shm.xml`. Needs Docker Compose v2,
the NVIDIA container runtime (`docker info | grep -i runtimes` lists `nvidia`), an NVIDIA driver (580.x here) and
the server image (`docker pull carlasim/carla:0.9.15`, 21 GB).

| Service | What |
|---|---|
| `carla-server` | `carlasim/carla:0.9.15` pinned by digest, GPU through the `nvidia` runtime, host network, `-RenderOffScreen -quality-level=Epic`, starts in Town03 (`docker/UserEngine.ini`), healthcheck on the RPC port 2000 |
| `sim` | `docker/Dockerfile` stage `sim`: Humble + numpy from apt + the `carla==0.9.15` wheel. Host network + IPC, runs as your uid/gid, repo at `/ws/src/brightdrive_localization`, bags at `/bags`, idle (`sleep infinity`) until you exec into it |

Every command below runs from the repo root.

### Build

```
docker compose build sim
```

### Start

```
docker compose up -d sim         # starts carla-server first and waits until it is healthy
docker compose ps                # both Up, carla-server (healthy)
```

The server shows `(health: starting)` until the RPC port answers: about 5 s on this laptop (RTX 4060 8 GB) with warm
shader caches, longer after a reboot or a fresh pull. It starts in Town03: `docker/UserEngine.ini` is mounted as
`CarlaUE4/Config/UserEngine.ini` and sets `GameDefaultMap` (a map on the command line is ignored by UE4 Shipping
builds). Startup VRAM peaks around 3.2 GB instead of 6 GB with the default Town10HD_Opt.

### Exec and check

```
docker compose exec sim bash                                   # a shell in the client container, Humble sourced
docker compose exec sim python3 /ws/src/brightdrive_localization/sim/scripts/check_client.py
```

`sim/scripts/check_client.py`: client and server versions must match, the server must be in Town03 (loaded only
if it is not, with the load time),
100 synchronous ticks at 0.01 s with the real-time factor, restores asynchronous mode in a `finally`.
The shell has Humble sourced (`/etc/bash.bashrc` in the image).

Server-side checks:

```
docker compose logs --tail 20 carla-server          # UE4 version line, no "Signal 11"
bash -c 'echo > /dev/tcp/127.0.0.1/2000' && echo "port 2000 open"
nvidia-smi | grep CarlaUE4                          # the server holds the GPU
```

### Stop

```
docker compose down          # all containers; or: docker compose stop sim
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
  `docker compose logs carla-server`, then `docker compose up -d carla-server` again. Seen once 16 s after start
  with no client, while Town10HD_Opt was still the startup map and the desktop sat on the NVIDIA GPU (VRAM peak
  6.9 of 8 GB); not seen since Town03 at start and `prime-select on-demand`.
- The server keeps running between runs; stop it only when you are done (a restart costs the startup time).
