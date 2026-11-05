#!/usr/bin/env python3
"""Start every run from an empty world: remove the cars, sensors and people a killed run left behind.

A run stopped hard (terminal closed, docker killed) cannot clean up, so its car with role_name=hero stays in the
world and the next run's bridge would attach to it instead of the new car. run_inside.sh calls this first.

  docker compose exec sim python3 -u /ws/src/brightdrive_localization/sim/scripts/clean_world.py
"""
# CARLA API.
import carla


def main():
    # Connect to the CARLA server.
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    # A killed ScenarioRunner can leave the world in synchronous mode: nothing moves until someone ticks.
    # Back to normal (the server runs on its own); ScenarioRunner switches it on again when it starts.
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    # Everything that moves or measures. Roads, buildings and traffic lights are not actors of these types.
    leftovers = [a for a in world.get_actors()
                 if a.type_id.startswith(('vehicle.', 'sensor.', 'walker.', 'controller.'))]
    # Sensors first (they hang on cars), then the rest; one batch call to the server.
    leftovers.sort(key=lambda a: not a.type_id.startswith('sensor.'))
    client.apply_batch_sync([carla.command.DestroyActor(a) for a in leftovers])
    print(f'clean world: removed {len(leftovers)} leftover actors', flush=True)


if __name__ == '__main__':
    main()
