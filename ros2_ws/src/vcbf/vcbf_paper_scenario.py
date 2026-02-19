#!/usr/bin/env python3
import carla
import signal
import sys

VEHICLES = [
    ("vehicle.ford.ambulance",              (-41.0,   94.0, 0.5),  -90.0),
    ("vehicle.carlamotors.carlacola",       (-44.5,   70.0, 0.5),  -90.0),
    ("vehicle.dodge.charger_police_2020",   (-38.0,   63.0, 0.5),  210.0),
    ("vehicle.volkswagen.t2",               (-37.5,   73.5, 0.5),  -45.0),
    ("vehicle.mercedes.sprinter",           (-45.0,   81.0, 0.5),  -90.0),
    ("vehicle.carlamotors.firetruck",       (-48.5,   86.0, 0.5),   90.0),
    ("vehicle.tesla.cybertruck",            (-40.25,  55.0, 0.5),  -90.0),
    ("vehicle.nissan.patrol",               (-48.5,   60.0, 0.5),   90.0),
]

spawned = []
world = None

def cleanup(*_):
    global spawned
    print("\nCleaning up actors...")
    try:
        for a in spawned:
            if a is not None and a.is_alive:
                a.destroy()
        print("Done.")
    except Exception as e:
        print(f"Cleanup error: {e}")
    sys.exit(0)

def spawn_vehicles(world, vehicles=VEHICLES, autopilot=False):
    bp_lib = world.get_blueprint_library()
    out = []
    for bp_id, (x, y, z), yaw in vehicles:
        bp = bp_lib.find(bp_id)
        tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            print(f"[FAIL] {bp_id}")
            continue
        if autopilot:
            actor.set_autopilot(True)
        out.append(actor)
        print(f"[OK] {bp_id} -> id={actor.id}")
    return out

def main(host="127.0.0.1", port=2000, timeout=5.0, autopilot=False):
    global spawned, world

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()

    # ensure Ctrl+C triggers cleanup
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    spawned = spawn_vehicles(world, autopilot=autopilot)
    print(f"Spawned {len(spawned)}/{len(VEHICLES)}. Press Ctrl+C to destroy.")

    # keep process alive
    signal.pause()

if __name__ == "__main__":
    main()
