import json
from pprint import pprint

import pubsub
import params

if __name__ == "__main__":

    config = params.config_path
    print("using node config file at " + config)

    with open(config) as f:
        node_data = json.load(f)

    pprint(node_data)
    
    loaded_nodes = []

    for entry in node_data['nodes']:
        _tmp = __import__(entry['path'])
        loaded_nodes.append(_tmp.object)
        print("loaded node: " + entry['id'])

    for node in loaded_nodes:
        print("launching node: " + node.__class__.__name__)
        node.start()
        print(node.__class__.__name__ + " started")

    try:
        pubsub.p.psubscribe("*")

        while True:
            msg = pubsub.p.get_message()
            if msg:
                print(msg)
    except KeyboardInterrupt:
        pass
