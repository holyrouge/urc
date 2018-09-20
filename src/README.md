#Source Directory

Individual nodes are stored in 'node_name/' and loaded in 'main' from a 
JSON config file. 

To make a new node, create a directory for your node with an __init__.py file.
In your node, be sure to extend the 'node' class provided here.

#Getting Started

You will need to have redis-server already installed to run any of the code. 
Remote instances of redis-server can probably work as well.
To install any dependencies, run `pip install -r requirements.txt` in the `src` directory.
#Contributing

Make sure to keep your own node in its own file to keep the repo organized.

