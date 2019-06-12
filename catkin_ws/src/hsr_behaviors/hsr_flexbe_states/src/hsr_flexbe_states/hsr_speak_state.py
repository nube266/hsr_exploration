#!/usr/bin/env python
#
# Active Intelligent Systems Laboratory
# Toyohashi University of Technology
# 
# Shigemichi Matsuzaki
#
import rospy
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyPublisher
from tmc_msgs.msg import Voice

class hsr_SpeakState(EventState):
    '''
    Publishes a talk request (tmc_msgs/Voic) message on a given topic name.
    -- sentence      string      The sentence that the robot speaks
    -- topic         string      The topic on which should be published.
    -- interrupting  Bool        Whether a new message is spoken by canceling the currently spoken one (no more used?)
    -- queueing      Bool        Whether incoming messages are queued when other message is processed (no more used?)
    -- language      int         The language that the robot uses (0: Japanese, 1: English)

    <= done                     Done publishing.
    '''

    def __init__(self, sentence, topic='/talk_request', interrupting=False, queueing=False, language=1):
        '''
        Constructor
        '''
        super(hsr_SpeakState, self).__init__(outcomes=['done'])

        self._topic = topic
        message = Voice()
        message.interrupting = interrupting
        message.queueing = queueing
        message.language = language
        message.sentence = sentence
        self._message = message
        self._pub = ProxyPublisher({self._topic: Voice})

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        self._pub.publish(self._topic, self._message)
