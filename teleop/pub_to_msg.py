import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Vector3_
import cyclonedds.idl.types as types



ChannelFactoryInitialize()

pub = ChannelPublisher("pinch_val", Vector3_)
pub.Init()



def LowStateHandler(msg: Vector3_):
    print(msg)


sub = ChannelSubscriber("pinch_val", Vector3_)
sub.Init(LowStateHandler, 10)

# def publish_reset_category(category: int,publisher): # Scene Reset signal
#     msg = String_(data=str(category))
#     publisher.Write(msg)
#     logger_mp.info(f"published reset category: {category}")



for i in range(30):
    val = 10.0
    ded = 0.0
    msg = Vector3_(x=types.float64(val), y=types.float64(0.0), z=types.float64(0.0))
    # msg.data = "Hello world. time:" + str(time.time())

    if pub.Write(msg, 0.5):
        print("publish success. msg:", msg)
    else:
        print("publish error.")

    time.sleep(1)

pub.Close()