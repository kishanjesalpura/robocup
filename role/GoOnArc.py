from enum import Enum
import behavior
import _GoOnArc_
try:
    _GoToPoint_ = reload(_GoToPoint_)
except:
    import _GoToPoint_
import rospy
from utils.functions import *
from utils.config import *


DISTANCE_THRESH = 10

class GoOnArc(behavior.Behavior):
    """docstring for GoOnArc"""
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        #formality
        setup = 1
        #ringa ringa roses
        round_n_round = 2


    def __init__(self,continuous=False):

        #initiating behaviour init
        super(GoOnArc,self).__init__()

        #defining class varibles
        self.name = "GoOnArc"
        self.center = None  #A vector2d for center of circle
        self.target = None
        self.radius = None
        self.behavior_failed = False
        self.DISTANCE_THRESH = DISTANCE_THRESH

        #adding states to fsm
        self.add_state(GoOnArc.State.setup,
            behavior.Behavior.State.running)
        self.add_state(GoOnArc.State.round_n_round,
            behavior.Behavior.State.running)

        #adding trnsitions to fsm
        self.add_transition(behavior.Behavior.State.start,
            GoOnArc.State.setup,lambda: True,'immediately')

        self.add_transition(GoOnArc.State.setup,
            GoOnArc.State.round_n_round,lambda: self.checklist(),'setup')

        self.add_transition(GoOnArc.State.round_n_round,
            behavior.Behavior.State.completed,lambda:self.at_target(),'complete')

        self.add_transition(GoOnArc.State.setup,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        self.add_transition(GoOnArc.State.round_n_round,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')


    def add_center(self,center):
        self.center = center
        bot_pos = self.kub.get_pos()
        bot_pos = Vector2D(bot_pos.x,bot_pos.y)
        self.radius = bot_pos.dist(self.center)

    def add_kub(self,kub):
        self.kub = kub

    def checklist(self):
        center_check = (self.center is None)
        target_check = (self.target is None)
        if center_check or target_check:
            print("something missing out of center or target")
            return False
        return True


    def at_target(self):
        bot_pos = self.kub.get_pos()
        bot_pos = Vector2D(bot_pos.x, bot_pos.y)
        return dist(self.target,bot_pos) < self.DISTANCE_THRESH

    #setup state run functions
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        _GoOnArc_.init(self.kub,self.target,self.theta)
        pass

    def on_exit_setup(self):
        pass

    #round_n_round run functions
    def on_enter_drive(self):
        pass

    def terminate(self):
        super().terminate()

    def execute_drive(self):
        print("Execute drive")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        generatingfunction = _GoOnArc_.execute(start_time,self.DISTANCE_THRESH)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target = gf

            if not vicinity_points(self.target,target):
                self.behavior_failed = True
                break
        self.new_point = self.kub.get_pos()

    def on_exit_drive(self):
        pass
