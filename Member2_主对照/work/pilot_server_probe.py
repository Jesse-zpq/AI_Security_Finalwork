from pathlib import Path
import sys

import carla

port = int(sys.argv[1])
client = carla.Client('127.0.0.1', port)
client.set_timeout(20.0)
for label, read in [('version', client.get_server_version), ('maps', client.get_available_maps),
                    ('map', lambda: client.get_world().get_map().name)]:
    try:
        print(label, read(), flush=True)
    except Exception as error:
        print(label, repr(error), flush=True)
for filename in sys.argv[2:]:
    try:
        world = client.generate_opendrive_world(Path(filename).read_text(encoding='utf-8'))
        print(filename, world.get_map().name, flush=True)
    except Exception as error:
        print(filename, repr(error), flush=True)
