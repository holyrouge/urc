# from test.node5 import test1
# from test.node6 import test2
#
# x = test1("test/test1config.json")
# y = test2("test/test2config.json")
#
# x.start()
# y.start()

from test.node7 import test3
from test.node8 import test4

a = test3("test/test3config.json")
b = test4("test/test4config.json")

a.start()
b.start()

