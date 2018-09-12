from sub1.sub1 import Sub1
from sub2.sub2 import Sub2
from sub3.sub3 import Sub3

import threading

if __name__ == "__main__":
    t1 = Sub1()
    t2 = Sub2()
    t3 = Sub3()
    t1.start()
    t2.start()
    t3.start()
    print(threading.enumerate())