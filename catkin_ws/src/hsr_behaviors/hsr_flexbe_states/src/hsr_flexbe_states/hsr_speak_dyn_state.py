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

class hsr_SpeakDynState(EventState):
    '''
    Publishes a talk request (tmc_msgs/Voice) message on a given topic name.
    -- sentence      string      The sentence that the robot speaks (+ for variable word. e.g.) Bring me +)
    -- topic         string      The topic on which should be published.
    -- interrupting  Bool        Whether a new message is spoken by canceling the currently spoken one (no more used?)
    -- queueing      Bool        Whether incoming messages are queued when other message is processed (no more used?)
    -- language      int         The language that the robot uses (0: Japanese, 1: English)

    ># variable      string      The word that substitutes '+' in the sentence

    <= done                     Done publishing.
    '''

    def __init__(self, sentence="It's +", sentence_when_empty="I couldn't recognize it", topic='/talk_request', interrupting=False, queueing=False, language=1):
        '''
        Constructor
        '''
        super(hsr_SpeakDynState, self).__init__(input_keys=['variable'], outcomes=['done', 'empty'])

        self._topic = topic
        self._sentence = sentence
        self._sentence_when_empty = sentence_when_empty
        message = Voice()
        message.interrupting = interrupting
        message.queueing = queueing
        message.language = language
        self._message = message
        self._pub = ProxyPublisher({self._topic: Voice})

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
        real_sentence  = ''
        self._outcome = 'done'
        # If variable is not empty
        if userdata.variable != '':
            split_sentence = self._sentence.split('+')
            real_sentence = split_sentence.pop(0)
            for s in split_sentence:
                real_sentence = real_sentence + ' ' + userdata.variable + ' ' + s
        # If it's empty, say something else
        else:
            real_sentence = self._sentence_when_empty
            self._outcome = 'empty'

        self._message.sentence = real_sentence
        self._pub.publish(self._topic, self._message)
