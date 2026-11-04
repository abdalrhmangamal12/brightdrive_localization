# sim — the CARLA 0.9.15 side

Bring-up step 1: only the CARLA server. The ROS 2 Humble container, the bridge and the configs come in later steps.

| File | Purpose |
|---|---|
| `docker/compose.yaml` | service `carla-server`: `carlasim/carla:0.9.15` pinned by digest, GPU through the `nvidia` runtime, host network, `-RenderOffScreen -quality-level=Epic`, healthcheck on the RPC port 2000 |
| `docker/.env` | uid/gid of the container user and `BAGS_DIR` (used from the ROS container on) |
| `docker/compose.viz.yaml` | optional override: windowed server on the host display, not loaded by default |

Needs: Docker Compose v2, the NVIDIA container runtime (`docker info | grep -i runtimes` lists `nvidia`), an NVIDIA driver (580.x here), and the image (`docker pull carlasim/carla:0.9.15`, 21 GB).

## Start

```
cd brightdrive_localization/sim/docker
docker compose up -d carla-server
```

The container shows `(health: starting)` until the RPC port answers, then `(healthy)`: 5 s on this laptop
(RTX 4060 8 GB) with warm shader caches, up to 3 minutes after a reboot or a fresh pull, while Town10HD_Opt loads.

## Check

```
docker compose ps                                   # STATUS ... (healthy)
docker compose logs --tail 20 carla-server          # UE4 version line, no "Signal 11"
bash -c 'echo > /dev/tcp/127.0.0.1/2000' && echo "port 2000 open"
nvidia-smi | grep CarlaUE4                          # the server holds the GPU
```

## Stop

```
docker compose down          # or: docker compose stop carla-server
```

## Windowed (to look at the town)

```
docker compose -f compose.yaml -f compose.viz.yaml up -d carla-server
```

`docker/compose.viz.yaml` overrides the command (no `-RenderOffScreen`, a 1280x720 window) and passes the host
display through. Not loaded by default. If the window does not appear: `xhost +SI:localuser:$USER`, then restart.

## Known behaviour

- `-quality-level=Low` segfaults on `load_world('Town03')` on this machine (exit 139); Epic is the setting that works.
- `restart: "no"`: a crashed server stays visible as `Exited (139)` in `docker compose ps -a`. Read
  `docker compose logs carla-server`, then `docker compose up -d carla-server` again.
- The server keeps running between scenario runs; stop it only when you are done (a restart costs the startup time).
