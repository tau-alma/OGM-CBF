import time
import random
import carla
import argparse
import logging


def jitter_transform(base_xyz, base_yaw_deg, sigma_x_m, sigma_y_m, sigma_yaw_deg, sigma_z_m=0.0):
    x, y, z = base_xyz
    x += random.gauss(0.0, sigma_x_m)
    y += random.gauss(0.0, sigma_y_m)
    z += random.gauss(0.0, sigma_z_m)
    yaw = base_yaw_deg + random.gauss(0.0, sigma_yaw_deg)
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))


def spawn_two_cars(
    world,
    default_sigma_x_m=0.5,
    default_sigma_y_m=0.5,
    default_sigma_yaw_deg=5.0,
    max_attempts=20,
    seed=None,
):
    """Spawns vehicles with per-obstacle Gaussian jitter (sigma_x, sigma_y, sigma_yaw)."""
    if seed is not None:
        random.seed(seed)

    # per obstacle: (model, (x,y,z), base_yaw_deg, sigma_x_m, sigma_y_m, sigma_yaw_deg)
    # set any sigma_* to None => uses defaults from CLI
    base_spawn_data = [
        ("vehicle.dodge.charger_police_2020", (-40.75, 95.0, 0.5), 45.0, 0.0, 0.0, 0.0),
        ("vehicle.dodge.charger_police_2020", (-45.5, 82.0, 0.5), -90, 0.0, 0.0, 0.0),
        ("vehicle.dodge.charger_police_2020", (-47.0, 90.0, 0.5), -90, 0.0, 0.0, 0.0),  # uses defaults
    ]

    blueprint_library = world.get_blueprint_library()
    spawned = []

    for model, base_xyz, base_yaw, sx, sy, syaw in base_spawn_data:
        sx = default_sigma_x_m if sx is None else sx
        sy = default_sigma_y_m if sy is None else sy
        syaw = default_sigma_yaw_deg if syaw is None else syaw

        bp = blueprint_library.find(model)
        if not bp:
            logging.warning(f"Blueprint {model} not found; skipping.")
            continue

        actor = None
        for _ in range(max_attempts):
            tf = jitter_transform(base_xyz, base_yaw, sx, sy, syaw)
            actor = world.try_spawn_actor(bp, tf)
            if actor is not None:
                spawned.append(actor)
                break

        if actor is None:
            logging.warning(
                f"Failed to spawn {model} after {max_attempts} attempts "
                f"(try smaller sigmas or different base point)."
            )

    return spawned


def main():
    argparser = argparse.ArgumentParser(description="CARLA spawner with per-obstacle anisotropic Gaussian jitter")
    argparser.add_argument("--host", default="127.0.0.1")
    argparser.add_argument("--port", type=int, default=2000)
    argparser.add_argument("--sigma_x", type=float, default=0.5, help="meters std dev for x jitter (default)")
    argparser.add_argument("--sigma_y", type=float, default=0.5, help="meters std dev for y jitter (default)")
    argparser.add_argument("--sigma_yaw", type=float, default=5.0, help="degrees std dev for yaw jitter (default)")
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
            default_sigma_x_m=args.sigma_x,
            default_sigma_y_m=args.sigma_y,
            default_sigma_yaw_deg=args.sigma_yaw,
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
