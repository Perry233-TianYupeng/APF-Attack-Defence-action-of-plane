from CommunicationTool import *
import math
import numpy as np


global flag
flag=1#一定要有 并且 在主函数create_action_cmd中添加global flag

class RD:
    """包含4个方法分别为`r2d`,`d2r`,`superr2d`,`superr2d`,用于将弧度制和角度制相互转换"""
    def __init__(self):
        self.rfactor=57.32484
    def r2d(self,r):
        return r*self.rfactor
    def d2r(self,d):
        return d/self.rfactor
    def superd2r(self,d):
        d=d%360
        return self.d2r(d) if d<=180 else self.d2r(d-360)
    def superr2d(self,r):
        return self.r2d(r) if r>=0 else self.r2d(r)+360
    def LongituteDis(self,LonDiffer,Lat):
        """计算经度差对应的距离,单位为米"""
        return LonDiffer*(math.pi/180)*6378137.0*math.cos(self.d2r(Lat))
    def LatitudeDis(self,LatDiffer):
        """计算纬度差对应的距离,单位为米"""
        return LatDiffer*(math.pi/180)*6378137.0
    def GetPosition(self,info,DroneID):
        """获取DroneID对应无人机的位置信息，返回值为角度制的经纬度和高度"""
        if info.DroneID==DroneID:
            position=(self.r2d(info.Longtitude),self.r2d(info.Latitude),info.Altitude)
        return position
    
RDer=RD()

def PrintData(info,step_num,DroneID,unit_step,object1,object2,object3,object4):
    """检查函数,每隔一定有限元步数打印`info`中指定飞机的指定数据,其中`unit_step`是打印的时间间隔"""
    #TODO:进行自动化处理
    if info.DroneID == DroneID and step_num%unit_step==0:
        print(object1,object2,object3,object4)        

class JDDZ:
    """机动动作类,需要提前依次传入`output_cmd`和`info`"""
    def __init__(self,output_cmd,info):
        self.output_cmd=output_cmd
        self.info=info

    def PingFei(self,Deg,Spd,Thrust=100): 
        """平飞,传入参数:平飞方向`Deg`,预设速度`Spd`,油门(如果不传入默认`100`)"""
        self.output_cmd.sPlaneControl.CmdID = 1
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
        return self.output_cmd
    
    def JiaJianSu(self,Deg,Spd,Thrust=120):
        """加减速,传入参数:方向`Deg`,预设速度`Spd`,油门(如果不传入默认`120`)"""
        self.output_cmd.sPlaneControl.CmdID = 2
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust 
        return self.output_cmd
    
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
        return self.output_cmd
    
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
        return self.output_cmd

    def ZhuanWan(self,Phi,Deg,Ny,Spd,TurnMode,Thrust=120):
        """转弯,传入参数:滚转角`Phi`,转弯方向`Deg`,过载`Ny`,预设速度`Spd`,转弯模式(`1`为锐角转弯,`2`为钝角转弯),油门(默认`120`)"""
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

        judge_deg= 0 if(abs(Deg-RDer.r2d(self.info.Yaw)))<180 else 1
        judge_mode= 1 if(Deg-RDer.r2d(self.info.Yaw))>0 else 0
        self.output_cmd.sPlaneControl.TurnDirectionif=-1 if (judge_deg+judge_mode+TurnMode)%2==1 else 1

        return self.output_cmd

    def SheXing(self,Phi,Deg1,Deg2,Ny,Spd,Thrust=120):
        """蛇形,传入参数:滚转角`Phi`,起始航向`Deg1`,终止航向`Deg2`,过载`Ny`,预设速度`Spd`,油门(默认`120`),注意:需要确保航向夹在两个角度之间"""
        global flag
        flag=1
        Deg1=Deg1%360
        Deg2=Deg2%360
        Deg1,Deg2=sorted([Deg1, Deg2])

        self.output_cmd.sPlaneControl.CmdID = 6
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdPhi = Phi
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdNy = Ny
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust

        judge_deg=0 if(Deg2-Deg1)>180 else 1  
        Yaw1_my=RDer.superd2r(Deg1)
        Yaw2_my=RDer.superd2r(Deg2)
        nega=0 if Yaw1_my*Yaw2_my>0 else 1

        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg1 if flag==1 else Deg2
        self.output_cmd.sPlaneControl.TurnDirection = -1 if (judge_deg+flag)%2==1 else 1
        if (judge_deg-0.5)*(self.info.Yaw-Yaw1_my)>0 and flag==1:flag=0
        if (judge_deg-0.5)*(self.info.Yaw-Yaw2_my)<0 and flag==0:flag=1
        if nega==1 and self.info.Yaw<=0 and flag==0:flag=1
        if nega==1 and self.info.Yaw>=0 and flag==1:flag=0

        return self.output_cmd
    
def CABP(x0, y0, z0, vn1, ve1, l, h1, theta_rad):
    """`A`的经度、纬度、海拔、北向速度、东向速度、B的直线距离、海拔、方向角 ————> `B`的经度、纬度、海拔"""
    global EARTH_R,rfactor
    EARTH_R = 6378137.0
    rfactor=57.32484
    A_rad = math.atan2(ve1, vn1)
    nt_rad = A_rad + theta_rad
    delta_h = abs(h1 - z0)
    pingmiandis = math.sqrt(abs(l**2 - delta_h**2))  
    A_weidu = math.radians(y0) 
    A_jingdu = math.radians(x0) 
    raddistance = pingmiandis / EARTH_R
    B_weidu = math.asin(
        math.sin(A_weidu) * math.cos(raddistance) +
        math.cos(A_weidu) * math.sin(raddistance) * math.cos(nt_rad)
    )
    B_jingdu = A_jingdu + math.atan2(
        math.sin(nt_rad) * math.sin(raddistance) * math.cos(B_weidu),
        math.cos(raddistance) - math.sin(A_weidu) * math.sin(B_weidu)
    )
    x1 = math.degrees(B_jingdu)
    y1 = math.degrees(B_weidu)
    # 处理经度溢出
    x1 = (x1 + 180) % 360 - 180
    return RDer.d2r(x1), RDer.d2r(y1), h1


def GetRelative(info,DroneID):
    """获取锁定`DroneID`对应的飞机的导弹相对于该飞机的相对角度,返回值为弧度制列表"""
    if info.DroneID==DroneID:
        RelativeList=[]
        for index in range(len(info.AlarmList)):
            if info.AlarmList[index].AlarmType=="导弹" and not info.AlarmList[index].MisAzi == 0:
                RelativeList.append(info.AlarmList[index].MisAzi)
        return RelativeList
    else:
        return None
    
def GetMissleDirection(info,DroneID,plane_Yaw):
    """获取锁定`DroneID`对应的飞机的导弹相对于该飞机的相对角度,返回值为角度制列表"""
    if info.DroneID==DroneID:
         RelativeList=GetRelative(info,DroneID)
         DirectionList=[]
         for index in range(len(RelativeList)):
             DirectionList.append((180+(RDer.superr2d(plane_Yaw)+RDer.superr2d(RelativeList[index])))%360)
         return DirectionList
    else:
         return None

class attackmethod(JDDZ):
    """使用以下各个方法时,需要承接返回值`output_cmd`"""
    def __init__(self,output_cmd,info):
        super().__init__(output_cmd,info)

    def attack0(self,DroneID,EnemyID):
        """需要指定发弹飞机`DroneID`和被打击的飞机的`EnemyID`"""
        if self.info.DroneID == DroneID: 
            for target in self.info.AttackEnemyList:
                if target.EnemyID == EnemyID:
                    if target.NTSstate == 2: 
                        self.output_cmd.sOtherControl.isLaunch = 1 
                        self.info.AttackEnemyList.remove(target)
                    else: 
                        self.output_cmd.sOtherControl.isLaunch = 0
                        self.output_cmd.sSOCtrl.isNTSAssigned = 1
                        self.output_cmd.sSOCtrl.NTSEntityIdAssigned = EnemyID
                    break
        return self.output_cmd
    
    def attack1(self,DroneID,missilenum):
        """需要指定发弹飞机`DroneID`和发弹数量"""
        #TODO:是不是反应速度太慢了导致的?
        temp={}
        if self.info.DroneID == DroneID and len(self.info.AttackEnemyList)!=0:
            for i in range(0,len(self.info.AttackEnemyList)):
                temp[self.info.AttackEnemyList[i].EnemyID]=self.info.AttackEnemyList[i].TargetDis
            temp=sorted(temp.items(), key=lambda x: x[1])
            for i in range(0,min(missilenum,len(temp))):
                self.output_cmd=self.attack0(DroneID,temp[i][0])
        return self.output_cmd
