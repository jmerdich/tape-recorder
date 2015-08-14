#!/usr/bin/python
import rospy
from tape import listen, init, play
from std_msgs.msg import String
import rosplan_interface as plans


@plans.planner_action
def recieve_message(msg, sender=None, loc=None):
    res = listen()
    if not res:
        raise Exception
    plans.add_instance('message', msg, String(res))
    plans.add_predicate('hasmessage', msg)


@plans.planner_action
def delivermessage(msg, loc):
    print "started %s" % msg
    try:
        filename = plans.get_instance('message', msg, String)[0].data
    except Exception as e:
        print e
    print "got instance" + str(filename)
    try:
        res = play(filename)
    except Exception as e:
        print e
    print res
    if not res:
        print "file not found %s" % filename 
        raise Exception
    plans.add_predicate('hasdeliveredmessage', x=msg, y=loc)

if __name__ == "__main__":
    rospy.init_node('tape_recorder')
    init()
    plans.init_kb()
    plans.start_actions()
    rospy.spin()
