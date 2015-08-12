#!/usr/bin/python
import rospy
from tape import listen, init, play
from std_msgs.msg import String
import rosplan_interface as plans


@plans.action_receiver
def recieve_message(msg, sender=None, loc=None):
    res = listen()
    if not res:
        raise Exception
    plans.add_instance('message', msg, String(res))
    plans.add_predicate('hasmessage', msg)


@plans.action_receiver
def delivermessage(msg, loc):
    filename = plans.get_instance('message', msg, String)
    res = play(filename)
    if not res:
        raise Exception
    plans.add_predicate('hasdeliveredmessage', msg, loc)

if __name__ == "__main__":
    rospy.init_node('tape_recorder')
    init()
    plans.init_kb()
    plans.start_actions()
    rospy.spin()
