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
RADIUS = None
CENTER = None
TARGET = None
FLAG_move = False
vx_end = 0
vy_end = 0


rospy.wait_for_service("bsServer",)
getState = rospy.ServiceProxy('bsServer',srv.bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

def init(_kub,center,radius,target):
    global kub,RADIUS,CENTER,FIRST_CALL,FLLAG_move,TARGET
    kub = _kub
    TARGET = Vector2D()
    TARGET.x = target.x
    TARGET.y = target.y
    CENTER = Vector2D()
    CENTER.x = center.x
    CENTER.y = center.y
    RADIUS = radius
    FLAG_move = False

def execute_inside_circle():
    normal_dir = math.atan2(self.kub.get_pos().y - self.circle.center.y),(self.kub.get_pos().x - self.circle.center.x)
    if self.kub.state.ballVel.x == 0 and self.kub.state.ballVel.y == 0:
        veldir = math.atan2(self.kub.state.ballPos.y-self.kub.get_pos().y,self.kub.state.ballPos.x-self.kub.get_pos().x)    
    else:
        veldir = math.atan2(self.kub.state.ballVel.y, self.kub.state.ballVel.x)
    point=getPointBehindTheBall(self.kub.state.ballPos,veldir)
    #theta = math.atan2(point.y-self.kub.get_pos().y, point.x-self.kub.get_pos().x)
    magnitude=V_CLOSE + dist(point,self.kub.get_pos())(MAX_BOT_SPEED/4-V_CLOSE)/(D_CLOSE)
    l = dist(self.circle.center,self.kub.get_pos())
    if self.circle.center == self.circle1.center:
        relvel_dir = normal_dir + (math.pi*l)/(2*self.circle.radius)
    if self.circle.center == self.circle2.center:
        relvel_dir = normal_dir - (math.pi*l)/(2*self.circle.radius)
    relvel = Vector2D(magnitude*math.cos(relvel_dir),magnitude*math.sin(relvel_dir))
    FinalVel=relvel+self.kub.state.ballVel
    return FinalVel.x, FinalVel.y

def execute_outside_circle():
    if kub.state.ballVel.x == 0 and kub.state.ballVel.y == 0:
        veldir = math.atan2(kub.state.ballPos.y-kub.get_pos().y,kub.state.ballPos.x-kub.get_pos().x)    
    else:
        veldir = math.atan2(kub.state.ballVel.y, kub.state.ballVel.x)
    point=getPointBehindTheBall(kub.state.ballPos,veldir)
    magnitude=V_CLOSE + dist(point,kub.get_pos())*(MAX_BOT_SPEED/4-V_CLOSE)/(D_CLOSE)
    l = dist(CENTER,kub.get_pos())
    beta = math.asin(RADIUS/l)
    theta = math.atan(kub.get_pos().y - CENTER.y)/(kub.get_pos().x - CENTER.x)
    angle1 = theta + beta
    angle2 = theta - beta
    multiplier1 = 1
    multiplier2 = 1
    kub_pos = Vector2D(kub.get_pos().x,kub.get_pos().y)
    if dist(kub_pos + Vector2D(BOT_RADIUS*math.cos(angle1),BOT_RADIUS*math.sin(angle1)),CENTER) > dist(kub_pos - Vector2D(BOT_RADIUS*math.cos(angle1),BOT_RADIUS*math.sin(angle1)),CENTER):
        multiplier1 = -1
    if dist(kub_pos + Vector2D(BOT_RADIUS*math.cos(angle2),BOT_RADIUS*math.sin(angle2)),CENTER) > dist(kub_pos - Vector2D(BOT_RADIUS*math.cos(angle2),BOT_RADIUS*math.sin(angle2)),CENTER):
        multiplier2 = -1
    if (math.cos(veldir)*math.cos(angle1) + math.sin(veldir)*math.sin(angle1))*multiplier1 < (math.cos(veldir)*math.cos(angle2) + math.sin(veldir)*math.sin(angle2))*multiplier2:
        cosangle = math.cos(angle1)*multiplier1
        sinangle = math.sin(angle1)*multiplier1
    else:
        cosangle = math.cos(angle2)*multiplier2
        sinangle = math.sin(angle2)*multiplier2
    relvel=Vector2D(magnitude*cosangle,magnitude*sinangle)
    FinalVel=relvel+kub.state.ballVel
    vx=FinalVel.x
    vy=FinalVel.y
    return vx,vy

def excecute_inside_alpha():
    if kub.state.ballVel.x == 0 and kub.state.ballVel.y == 0:
        veldir = math.atan2(kub.state.ballPos.y-kub.get_pos().y,kub.state.ballPos.x-kub.get_pos().x)    
    else:
        veldir = math.atan2(kub.state.ballVel.y, kub.state.ballVel.x)
    point=getPointBehindTheBall(kub.state.ballPos,veldir)
    kub_pos = kub.get_pos()
    magnitude=V_CLOSE + dist(point,kub.get_pos())*(MAX_BOT_SPEED/4-V_CLOSE)/(D_CLOSE)
    theta = math.atan2(point.y-kub_pos.y, point.x-kub_pos.x)
    vx = magnitude*math.cos(theta)
    vy = magnitude*math.sin(theta)
    return vx,vy
    
def execute(strategy, startTime,DIST_THRESH,avoid_ball=False):
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
            
            strategy = 'execute_' + strategy
            
            velocity_planner = getattr(strategy)

            vx, vy = velocity_planner()

            #if not vw:
            #    vw = 0

            if not vx and not vy:
                vx,vy = vx_end,vy_end
            else:
                vx_end,vy_end = vx,vy

            if functions.dist(kub.state.homePos[kub.kubs_id],TARGET)<DIST_THRESH :
                kub.move(0,0)
                FLAG_move = True
            else:
                kub.move(vx, vy)

            kub.execute()
            yield kub,TARGET



    kub.execute()

    yield kub,TARGET
