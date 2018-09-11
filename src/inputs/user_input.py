import redis
import inputs
from inputs import get_gamepad
from inputs import get_key

# initialize redis-py client with defaults
client = redis.StrictRedis(host='localhost', port=6379, db=0)


while True:
 
  # takes gamepad and keyboard input and publishes them to their respective
  # channels
  #events = get_gamepad()
  #for event in events:
    #msg = str(event.ev_type) + ' ' + str(event.code) + ' ' + str(event.state)
    #print(msg)
    #client.publish('gamepad-channel', msg)

  events = get_key()
  for event in events:
    msg = str(event.ev_type) + ' ' + str(event.code) + ' ' + str(event.state)
    print(msg)
    client.publish('keyboard-channel', msg)
      
