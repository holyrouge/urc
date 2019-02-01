import json
from pprint import pprint

import params
import importlib
import sys
import time
from status import Status


if __name__ == "__main__":

    config = params.config_path
    print("using node config file at " + config)

    with open(config) as f:
        node_data = json.load(f)

    pprint(node_data)

    loaded_nodes = []

    for entry in node_data['nodes']:
        _tmp = importlib.import_module(entry['path'])
        c = getattr(_tmp, entry['node'])
        loaded_nodes.append(c(entry['config']))
        print("loaded node: " + entry['id'])

    for node in loaded_nodes:
        print("launching node: " + node.__class__.__name__)
        node.start()
        print(node.__class__.__name__ + " started")

    while True:
        try:
            time.sleep(0.0001)
        except KeyboardInterrupt:
            print("Closing down...")
            for node in loaded_nodes:
                node.stop()
                node.join()
            sys.exit()

        for node in loaded_nodes:
            if node.status == Status.CRASHED:
                print(node.id + " has crashed, attempting to restart")
                conf = ""
                for entry in node_data['nodes']:
                    if entry['id'] == node.id:
                        conf = entry['config']
                node.__init__(conf)
                node.start()
                print("restarted node: " + node.id)



