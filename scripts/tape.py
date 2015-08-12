#!/usr/bin/python

import roslib
roslib.load_manifest('tape_recorder')

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from tape_recorder.srv import PlayFile, PlayFileResponse

import os
import os.path
import subprocess
from datetime import datetime
lastfile = None


def listen(content):
    global lastfile
    filename = os.path.abspath(
        os.path.join(rec_dir,
                     datetime.strftime(datetime.now(),
                                       "recording_%y-%m-%d_%H-%M-%S.wav"))
    )
    rospy.loginfo("Started recording to %s" % filename)
    if subprocess.check_call(sox_cmd.format(device=rec_device,
                                            file=filename,
                                            silence=silence), shell=True):
        return TriggerResponse(success=False, message="Failed")
    rospy.loginfo("Recording %s completed successfully" % filename)
    lastfile = filename
    return TriggerResponse(success=True, message=filename)


def play(content):
    if not content.infile and lastfile:
        filename = lastfile
    else:
        filename = os.path.abspath(content.infile)
    rospy.loginfo("Started playback of %s" % filename)
    if not os.path.isfile(filename):
        return PlayFileResponse(success=False, message="File not found")
    if subprocess.check_call(play_cmd.format(file=filename),
                             shell=True):
        return PlayFileResponse(success=False, message="Playback failed")
    rospy.loginfo("Playing %s completed successfully" % filename)
    return PlayFileResponse(success=True, message="Success")


if __name__ == '__main__':
    rospy.init_node('tape_recorder')
    trigger = rospy.Service("~listen", Trigger, listen)
    confidence = rospy.Service("~play", PlayFile, play)
    rec_device = rospy.get_param("~rec_device", "alsa default")
    silence = rospy.get_param("~silence", "0 1 00:00:01.0 8%")
    rec_dir = os.path.abspath(rospy.get_param(
        "~recording_dir",
        os.path.abspath(os.path.join(os.environ['HOME'],
                        "recordings"))))
    if not os.path.isdir(rec_dir):
        os.makedirs(rec_dir)
    sox_cmd = rospy.get_param("~rec_command",
                              'sox -q -t {device} {file} vad silence {silence}')
    play_cmd = rospy.get_param("~play_command", 'play -q {file}')
    rospy.loginfo("Tape recorder ready.")
    rospy.spin()
