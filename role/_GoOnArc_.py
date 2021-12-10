from kubs import kubs, cmd_node
from velocity import run
from velocity import run_w
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils.geometry import Vector2D
from utils import config, functions
import krssg_ssl_msgs.srv as srv
import math
import numpy as np

kub = None
start_time = None
TARGET = None
CENTER = None
RADIUS = None
FLAG_move = False
rotate = False
vx_end = 0
vy_end = 0


rospy.wait_for_service("bsServer",)
getState = rospy.ServiceProxy('bsServer',srv.bsServer)

#try:
#    prev_state = getState(prev_state).stateB
#except rospy.ServiceException, e:
#    print("Error ", e)

def init(_kub,target,center,rotate):
    global kub,TARGET,RADIUS,CENTER,FLAG_move,FIRST_CALL
    kub = _kub
    TARGET = Vector2D()
    CENTER = Vector2D()
    CENTER.x = center.x
    CENTER.y = center.y
    rotate = rotate
    TARGET.x = target.x
    TARGET.y = target.y
    FLAG_move = False
    FIRST_CALL = True

def velocity_planner(bot_pos):
    #PD controller
    consts = np.array([3.5,0.003])
    theta1 = atan2(bot_pos.y-CENTER.y,bot_pos.x-CENTER.x)
    theta2 = atan2(TARGET.y-CENTER.y,TARGET.x-CENTER.x)
    theta = theta2-theta1
    pd = np.array([[theta*RADIUS],[kub.get_vel().magnitute]])
    vel = pd.dot(consts)
    if vel>config.MAX_BOT_SPEED:
        vel = config.MAX_BOT_SPEED
    vx = vel*math.sin(theta1)
    vy = vel*math.cos(theta1)
    return vx,vy

def execute(startTime,DIST_THRESH,avoid_ball=False):
    global getState,TARGET, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub,prev_state,vx_end,vy_end

    while not (FLAG_move):


        try:
            kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)



        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            [vx, vy] = velocity_planner(kub.get_pos())

            if not vw:
                vw = 0

            if not vx and not vy:
                vx,vy = vx_end,vy_end
            else:
                vx_end,vy_end = vx,vy

            if dist(kub.state.homePos[kub.kubs_id], TARGET)<DIST_THRESH :
                kub.move(0,0)
                FLAG_move = True
            else:
                kub.move(vx, vy)

            kub.execute()
            yield kub,TARGET



    kub.execute()

    yield kub,TARGET
