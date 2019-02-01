#Creating Nodes With Our Fancy Framework(still have not come up with a name for it)

The first step is to make sure you have zeromq installed, easily done by running 
`pip3 install requirements.txt`. Make sure you are using Python 3 for this as none 
of this code was tested with Python 2.

Once you have the dependencies, create a Python package in the src folder of the 
project. You will need to create an empty file called `__init__.py` in this folder.
It would be best if you named the folder after your node.

Once that is done, create a file `[node_name].py` and `[node_name]_config.json`.

In your Python file, import the `Node` class and create a class that extends it.
The most important methods for you to create are the constructor and a method called 
`loop()`. Those are the main ways your node will be initialized and run. Please note
that the constructor for the `Node` class requires a path to a config file, that is
the JSON file you just made.

In your JSON file, you can put all kinds of data that your node needs to configure 
itself with on startup. The node will automatically load eerything in your config 
file so you are **strongly** encouraged to put any constants in their in case they
need to be changed.

Your node's config file is required to have an id field, as well as a topic field.
The id should be a string and the topics should be an array of objects representing
any connections your node needs to make with other.

Further documentation is on the way, and the `Node` class is pretty well documented.

Happy coding!
