from sub1.sub1 import Sub1
from sub2.sub2 import Sub2
from sub3.sub3 import Sub3

import pubsub

if __name__ == "__main__":
    try:
        pubsub.p.psubscribe("*")

        t1 = Sub1()
        t2 = Sub2()
        t3 = Sub3()
        t1.start()
        t2.start()
        t3.start()
        
        while True:
            msg = pubsub.p.get_message()
            if msg:
                print(msg)
    except KeyboardInterrupt:
        pass