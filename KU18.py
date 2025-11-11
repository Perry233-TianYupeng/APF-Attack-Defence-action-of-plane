#空战仿真的一些通用函数


#本文件参照可以直接通过端口控制平台飞机的最大速度，油门，航向，倾角，过载，航迹倾角的仿真平台进行设计，如需使用请根据使用的平台实际指令进行修改
#本文件使用场景为敌我双方4v4空战，其中主程序通过调用create_action_cmd函数与平台传参，每次调用该函数时，会传入己方一架飞机的info和output_cmd
#主程序会按照我方1到4号机的顺序轮流访问飞机，每一轮次传入的指令只对当前访问的飞机进行操控
#以下函数均按照该逻辑进行编写

import math
import numpy as np
import sys



class RD:
    """包含弧度与角度转换，经纬度与距离转换等函数，用于处理位置信息"""
    def __init__(self):
        self.rfactor=57.32484
    def r2d(self,r):#弧度转角度，不做范围统一
        return r*self.rfactor
    def d2r(self,d):#角度转弧度，不做范围统一
        return d/self.rfactor
    def superd2r(self,d):#角度转弧度，并处理为-pi-+pi
        d=d%360
        return self.d2r(d) if d<=180 else self.d2r(d-360)
    def superr2d(self,r):#弧度转角度，并处理为0-360
        return self.r2d(r) if r>=0 else self.r2d(r)+360
    def LongituteDis(self,LonDiffer,Lat):
        """计算经度差对应的距离,单位为米"""
        return LonDiffer*(math.pi/180)*6378137.0*math.cos(self.d2r(Lat))
    def LatitudeDis(self,LatDiffer):
        """计算纬度差对应的距离,单位为米"""
        return LatDiffer*(math.pi/180)*6378137.0
    def GetPosition(self,info,DroneID):
        """获取DroneID对应无人机的位置信息，返回值为角度制的经纬度和高度"""
        position=(self.r2d(info.Longtitude),self.r2d(info.Latitude),info.Altitude)
        return position

#后续内容中使用到RD类中的函数，故创建RDer对象    
RDer=RD()
      
global flag
global TurnDirection #请初学者思考一下，这个变量名称为什么没有变蓝？
flag=1 #蛇形机动标志位，用于改变转向顺逆时针
TurnDirection=[0,0,0,0] #每架飞机的转向顺逆时针，1顺时针，-1逆时针

# 以上对于TurnDirection的global声明是个标准的错误示范哦！
# 在此补充一些关于global的小知识：
# global关键字用于在函数内部声明全局变量，使得函数内部对全局变量的修改能够影响函数外部的全局变量
# 在函数内部声明全局变量时，如果函数内部没有对全局变量进行修改，则不需要使用global关键字
# 细心的人可能发现若在VScode中打开这段代码，变量flag变为了蓝色，但列表TurnDirection却仍未白色，并且若在其他函数中使用flag还需在函数内声明global，而列表TurnDirection则不需要
# 初学者在使用global关键字时，可能经常碰到这种情况
# 这是因为在python中，赋值语句的逻辑是：将变量名绑定到一个对象上，而不是修改对象本身，例如：x=1，y=x，x=2，此时y的值仍然是1，因为y绑定的是x第一次赋值时的对象，而不是x本身，
# 但x的值变为了2，因为此时的x不再是赋值给y的x，而是一个新的对象，所以y的值不会变，这类型对象我们称之为不可变对象，因此，我们若是想修改这类变量，则需要使用global关键字
# 而列表，字典，元组等对象，我们称之为可变对象，因为我们可以修改它们的值，而不需要使用global关键字，例如：x=[1,2,3],y=x,x[0]=2，此时y的值变为了[2,2,3]，因为y绑定的是x本身，而不是x第一次赋值时的对象

class JDDZ:
    """机动动作类,需要提前依次传入`output_cmd`,`info`和'DroneID'"""
    def __init__(self,output_cmd,info,DroneID):
        self.output_cmd=output_cmd
        self.info=info
        self.DroneID=DroneID

    def PingFei(self,Deg,Spd,Thrust=100): 
        """平飞,传入参数:平飞方向`Deg`,预设速度`Spd`,油门(如果不传入默认`100`)"""
        self.output_cmd.sPlaneControl.CmdID = 1
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
    
    def JiaJianSu(self,Deg,Spd,Thrust=120):
        """加减速,传入参数:方向`Deg`,预设速度`Spd`,油门(如果不传入默认`120`)"""
        self.output_cmd.sPlaneControl.CmdID = 2
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust 
    
    def PaSheng(self,Deg,Spd,Alt,Thrust1=120,Thrust2=100):
        """最速爬升到`Alt`高度后沿着`Deg`方向平飞,`Thrust1`为最速爬升时的推力(默认`120`),`Thrust2`为平飞时的推力(默认`100`)"""
        self.output_cmd.sPlaneControl.CmdID = 3
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust1
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust1
        if self.info.Altitude>=Alt:
            self.output_cmd=self.PingFei(Deg,Spd,Thrust2)
    
    def FuChong(self,Spd,Alt,PitchDeg,Deg,Thrust):
        """俯冲,传入参数:预设速度`Spd`,俯冲目标海拔`Alt`,航迹倾角`PitchDeg`(**-90~+90**),俯冲方向`Deg`,油门"""
        self.output_cmd.sPlaneControl.CmdID = 7
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.CmdPitchDeg = PitchDeg
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust 
        self.output_cmd.sPlaneControl.CmdAlt=Alt
    def ZhuanWan(self,Phi,Deg,Ny,Spd,TurnMode,Thrust=120):
        """转弯,传入参数:滚转角`Phi`,转弯方向`Deg`,过载`Ny`,预设速度`Spd`,转弯模式(`1`绕小角度,`2`绕大角度),油门(默认`120`)"""
        global flag2
        global TurnDirection
        Deg=Deg%360
        self.output_cmd.sPlaneControl.CmdID = 6
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg
        self.output_cmd.sPlaneControl.CmdPhi = Phi
        self.output_cmd.sPlaneControl.CmdNy = Ny
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
        
        #这部分代码是十分实用的用来判断转动较小角度时，飞机的转向顺逆时针的代码！
        a1=RDer.superr2d(self.info.Yaw)
        a2=Deg
        delta = (a2-a1)%360
        if delta > 180:
            if TurnMode==1:
                TurnDirection[int((self.DroneID/100000)-1)] = -1
            elif TurnMode==2:
                TurnDirection[int((self.DroneID/100000)-1)] = 1
        else :
            if TurnMode==1:
                TurnDirection[int((self.DroneID/100000)-1)] = 1
            elif TurnMode==2:
                TurnDirection[int((self.DroneID/100000)-1)] = -1
                
                
        self.output_cmd.sPlaneControl.TurnDirection = TurnDirection[int((self.DroneID/100000)-1)]

    def SheXing(self,Phi,Deg1,Deg2,Ny,Spd,Thrust=120):
        """蛇形,传入参数:滚转角`Phi`,起始航向`Deg1`,终止航向`Deg2`,过载`Ny`,预设速度`Spd`,油门(默认`120`),注意:需要确保航向夹在两个角度之间"""
        #此函数没有使用上面的ZhuanWan函数进行优化，读者可以尝试利用参考代码，自行写出一个更高效的函数
        global flag
        self.output_cmd.sPlaneControl.CmdID = 6
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdPhi = Phi
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdNy = Ny
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
        judge_deg=0#0为夹角小于180，1为夹角大于180 
        nega=0#判断两角度的弧度制是否为同号，nega%2==0为同号 ==1为异号
        Deg1=Deg1%360
        Deg2=Deg2%360
        if Deg1>Deg2:#使Deg1为较小的度数
            temp=Deg1
            Deg1=Deg2
            Deg2=temp
        if Deg2-Deg1>180:
            judge_deg=1
        Yaw1_my=0
        Yaw2_my=0
        if Deg1<=180 :
            Yaw1_my=Deg1*math.pi/180
        else:
            Yaw1_my=(Deg1-360)*math.pi/180
            nega=nega+1
        if Deg2<=180 :
            Yaw2_my=Deg2*math.pi/80
        else:
            Yaw2_my=(Deg2-360)*math.pi/180
            nega=nega+1
        if judge_deg==0:    
            if flag == 1 :
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg1
                self.output_cmd.sPlaneControl.TurnDirection = -1
                if nega%2==0:
                    if  self.info.Yaw <=Yaw1_my:
                        flag = 0
                if nega%2==1:
                    if self.info.Yaw<=Yaw1_my and self.info.Yaw>0:
                        flag = 0
            elif flag==0:
                self.output_cmd.sPlaneControl.TurnDirection = 1
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg2
                if nega%2==0:
                    if self.info.Yaw >= Yaw2_my/180:
                        flag=1
                if nega%2==1:
                    if self.info.Yaw >= Yaw2_my/180 and self.info.Yaw<0:
                        flag=1
        elif judge_deg==1:
            if flag == 1 :
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg1
                self.output_cmd.sPlaneControl.TurnDirection = 1
                if self.info.Yaw>=Yaw1_my:
                    flag=0
            elif flag==0:
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg2
                self.output_cmd.sPlaneControl.TurnDirection = -1
                if self.info.Yaw<=Yaw2_my:
                    flag=1

    
def CABP(x0, y0, z0, vn1, ve1, l, h1, theta_rad):
    """`A`(我方飞机)的经度、纬度、海拔、北向速度、东向速度、B的直线距离、海拔、方向角 ————> `B`（敌方飞机）的经度、纬度、海拔"""
    global EARTH_R,rfactor
    EARTH_R = 6378137.0
    rfactor=57.32484
    A_rad = math.atan2(ve1, vn1)
    nt_rad = A_rad + theta_rad
    delta_h = abs(h1 - z0)
    pingmiandis = math.sqrt(abs(l**2 - delta_h**2))  
    A_weidu = y0
    A_jingdu = x0 
    raddistance = pingmiandis / EARTH_R
    B_weidu = math.asin(
        math.sin(A_weidu) * math.cos(raddistance) +
        math.cos(A_weidu) * math.sin(raddistance) * math.cos(nt_rad)
    )
    B_jingdu = A_jingdu + math.atan2(
        math.sin(nt_rad) * math.sin(raddistance) * math.cos(B_weidu),
        math.cos(raddistance) - math.sin(A_weidu) * math.sin(B_weidu)
    )
    x1 = RDer.r2d(B_jingdu)
    y1 = RDer.r2d(B_weidu)
    # 处理经度溢出
    x1 = (x1 + 180) % 360 - 180
    return RDer.d2r(x1), RDer.d2r(y1), h1


def GetRelative(info,DroneID):
    """获取锁定`DroneID`对应的飞机的导弹相对于该飞机的相对角度,返回值为弧度制列表"""
    #此函数基于我们能够获取导弹的与飞机的相对角度，且需要被雷达探测到
    if info.DroneID==DroneID:
        RelativeList=[]
        for index in range(len(info.AlarmList)):
            if info.AlarmList[index].AlarmType=="导弹" and not info.AlarmList[index].MisAzi == 0:
                RelativeList.append(info.AlarmList[index].MisAzi)
        return RelativeList
    else:
        return None
    
def GetMissleDirection(info,DroneID,plane_Yaw):
    """获取锁定`DroneID`对应的飞机的导弹的真实航向,返回值为角度制列表"""
    if info.DroneID==DroneID:
         RelativeList=GetRelative(info,DroneID)
         DirectionList=[]
         for index in range(len(RelativeList)):
             DirectionList.append((180+(RDer.superr2d(plane_Yaw)+RDer.superr2d(RelativeList[index])))%360)
         return DirectionList
    else:
         return None


    

def GetTargetID(info,DroneID):
    """获取Drone的FoundEnemyList(由雷达探测)中距离最近的敌方飞机的ID,未探测到目标则返回404"""
    if info.DroneID==DroneID:
        Enemy=[]
        for i in range(len(info.FoundEnemyList)):
            if info.FoundEnemyList[i].TargetDis!=0:
                Enemy.append(info.FoundEnemyList[i])
        if len(Enemy)!=0:
            for j in range(len(Enemy)):
                if Enemy[j].TargetDis==min([Enemy[i].TargetDis for i in range(len(Enemy))]):
                    return Enemy[j].EnemyID
        else:
            return 404
            


def GetTargetDistance(info,DroneID):
    """获取Drone的AttackEnemyList(由武器系统探测)中距离最近的敌方飞机的距离,未探测到目标则返回0"""
    if info.DroneID==DroneID:
        for target in info.AttackEnemyList:
            if target.EnemyID == GetTargetID(info,DroneID):
                return target.TargetDis
    return 0


