import json
from pprint import pprint

import params
import importlib




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

