import time
import random
import carla
import argparse
import logging


def jitter_transform(base_xyz, base_yaw_deg, sigma_xy_m, sigma_yaw_deg, sigma_z_m=0.0):
    x, y, z = base_xyz
    x += random.gauss(0.0, sigma_xy_m)
    y += random.gauss(0.0, sigma_xy_m)
    z += random.gauss(0.0, sigma_z_m)
    yaw = base_yaw_deg + random.gauss(0.0, sigma_yaw_deg)
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))


def spawn_two_cars(world, sigma_xy_m=0.5, sigma_yaw_deg=5.0, max_attempts=20, seed=None):
    """Spawns exactly 2 vehicles with Gaussian jitter around fixed base spawn points."""
    if seed is not None:
        random.seed(seed)

    # pick ANY 2 vehicles you want; these are two "cars/vans" from your list
    base_spawn_data = [
        ("vehicle.volkswagen.t2", (-46.0, 60.2, 0.5), -90),
        ("vehicle.mercedes.sprinter", (-45.5, 55.0, 0.5), -90),
    ]

    blueprint_library = world.get_blueprint_library()
    spawned = []

    for model, base_xyz, base_yaw in base_spawn_data:
        bp = blueprint_library.find(model)
        if not bp:
            logging.warning(f"Blueprint {model} not found; skipping.")
            continue

        actor = None
        for _ in range(max_attempts):
            tf = jitter_transform(base_xyz, base_yaw, sigma_xy_m, sigma_yaw_deg)
            actor = world.try_spawn_actor(bp, tf)  # returns None if collision / invalid spawn
            if actor is not None:
                spawned.append(actor)
                break

        if actor is None:
            logging.warning(
                f"Failed to spawn {model} after {max_attempts} attempts "
                f"(try smaller sigma or different base point)."
            )

    return spawned


def main():
    argparser = argparse.ArgumentParser(description="CARLA 2-car spawner with Gaussian jitter")
    argparser.add_argument("--host", default="127.0.0.1")
    argparser.add_argument("--port", type=int, default=2000)
    argparser.add_argument("--sigma_xy", type=float, default=0.5, help="meters (std dev) for x/y jitter")
    argparser.add_argument("--sigma_yaw", type=float, default=5.0, help="degrees (std dev) for yaw jitter")
    argparser.add_argument("--max_attempts", type=int, default=20)
    argparser.add_argument("--seed", type=int, default=None)
    args = argparser.parse_args()

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    obstacles = []
    try:
        world = client.get_world()
        obstacles = spawn_two_cars(
            world,
            sigma_xy_m=args.sigma_xy,
            sigma_yaw_deg=args.sigma_yaw,
            max_attempts=args.max_attempts,
            seed=args.seed,
        )
        logging.info(f"Spawned {len(obstacles)} vehicles.")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt. Cleaning up...")
    finally:
        for a in obstacles:
            try:
                a.destroy()
            except Exception:
                pass
        logging.info("Cleanup completed. Exiting.")


if __name__ == "__main__":
    main()
