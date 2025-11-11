
#空战仿真的人工势场法函数（本文档仅设计最简单的正反比关系为例，具体函数请根据实际平台进行优化）


# 人工势场法介绍：
# 人工势场法（Artificial Potential Field, APF）是一种经典的机器人路径规划与避障算法，
# 由 Khatib 于 1985 年提出。其核心思想是将机器人在环境中的运动类比为在虚拟力场中的受力运动：
# 目标点对机器人产生引力（吸引机器人向目标移动）
# 障碍物对机器人产生斥力（迫使机器人远离障碍）
# 通过计算引力和斥力的合力，引导机器人逐步向目标移动并实现避障。

# 优点:
# 计算简单高效：实时性好，适合动态环境
# 路径平滑：无需后期平滑处理，利于机器人运动控制
# 无需先验地图：可结合传感器实时感知环境

# 缺点：
# 局部极小值问题：当引力与多个障碍物斥力平衡时，机器人可能停滞
# 目标不可达问题：目标附近障碍物密集时，斥力可能大于引力，导致机器人无法到达目标
# 复杂环境建模困难：障碍物形状复杂时，势场设计繁琐
# 以上优缺点由AI生成，仅供参考


#本文件参照可以直接通过端口控制平台飞机的最大速度，油门，航向，倾角，过载，航迹倾角的仿真平台进行设计，如需使用请根据使用的平台实际指令进行修改
#本文件使用场景为敌我双方4v4空战，其中主程序通过调用create_action_cmd函数与平台传参，每次调用该函数时，会传入己方一架飞机的info和output_cmd
#主程序会按照我方1到4号机的顺序轮流访问飞机，每一轮次传入的指令只对当前访问的飞机进行操控
#以下函数均按照该逻辑进行编写
#其中RDer，JDDZ类详见“空战仿真的一些常见函数参考”

#假设飞机编号分别为：100000，200000，300000，400000
import KU18 as KU
import math
import numpy as np
import sys


class Mp():#建立战场类
    def __init__(self,lon_start,lon_end,lat_start,lat_end,ChiliParameter):# 初始化函数，用于创建一个战场区域对象
        self.lon_start = lon_start # 设置战场区域的起始经度
        self.lon_end = lon_end# 设置战场区域的结束经度
        self.lat_start = lat_start# 设置战场区域的起始纬度
        self.lat_end = lat_end# 设置战场区域的结束纬度
        self.ChiliParameter = ChiliParameter # 设置斥力参数

    def is_inside(self,position):
        """Position为GetPosition(info,DroneID)的返回值,判断飞机是否在战场内"""
        Plane_lon,Plane_lat,Plane_alt=position
        if self.lon_start <= Plane_lon <= self.lon_end and self.lat_start <= Plane_lat <= self.lat_end:
            return True
        else:
            return False
        
    def distance2boundary(self,position):
        """返回值为东向力距离-西向力距离，北向力距离-南向力距离"""
        EARTH_R = 6378137.0
        Plane_lon,Plane_lat,Plane_alt=position
        DisMapLonLeft=(Plane_lon-self.lon_start)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))#求解与各边界的距离
        DisMapLonRight=(Plane_lon-self.lon_end)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))
        DisMapLatUp=(Plane_lat-self.lat_end)*(math.pi/180)*EARTH_R
        DisMapLatDown=(Plane_lat-self.lat_start)*(math.pi/180)*EARTH_R
        Fright=self.ChiliParameter/abs(DisMapLonLeft)#正数，东向力
        Fleft=-self.ChiliParameter/abs(DisMapLonRight)#负数，西向力
        Fup=self.ChiliParameter/abs(DisMapLatDown)#正数，北向力
        Fdown=-self.ChiliParameter/abs(DisMapLatUp)#负数，南向力
        if self.is_inside(position):
            return Fright+Fleft,Fup+Fdown,0
        else:
            return (Fright+Fleft)*100,(Fup+Fdown)*100,0
    def distanceleft2boundary(self,position):
        """返回值为距离四条边界的最小距离，若超出则返回负数"""
        EARTH_R = 6378137.0
        Plane_lon,Plane_lat,Plane_alt=position
        DisMapLonLeft=(Plane_lon-self.lon_start)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))#求解与各边界的距离
        DisMapLonRight=(self.lon_end-Plane_lon)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))
        DisMapLatUp=(self.lat_end-Plane_lat)*(math.pi/180)*EARTH_R
        DisMapLatDown=(Plane_lat-self.lat_start)*(math.pi/180)*EARTH_R
        distance=min(DisMapLonLeft,DisMapLonRight,DisMapLatUp,DisMapLatDown)
        return distance
    
class Obstacle:#建立威胁区类
    def __init__(self,info,lon,lat,alt,radius,ChiliParameter):
        self.info = info
        self.lon = lon
        self.lat = lat
        self.alt = alt
        self.radius = radius
        self.ChiliParameter=ChiliParameter
    def is_inside(self,DroneID):
        """
        DroneID:我方飞机ID
        判断我方飞机是否在威胁区中
        """
        if self.info.DroneID==DroneID:
            x=RDer.superr2d(self.info.Longtitude)
            y=RDer.superr2d(self.info.Latitude)
            z=self.info.Altitude
            if (RDer.LongituteDis(self.lon-x,y))**2+(RDer.LatitudeDis(y-self.lat))**2+(z-self.alt)**2<=self.radius**2:
                return True
            else:
                return False
    def distance2Obs(self,position):
        """依次返回东、北、上三个方向的斥力信息"""
        Plane_lon,Plane_lat,Plane_alt=position
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)-math.sqrt(self.radius**2-Plane_alt**2)
        theta=np.arctan2((RDer.LongituteDis((Plane_lon-self.lon),self.lat)),(RDer.LatitudeDis(Plane_lat-self.lat)))
        if Plane_alt>=self.radius:
            return 0,0,0
        if Length>0:
            Force= self.ChiliParameter/Length
            ForceEast=Force*math.sin(theta)
            ForceNorth=Force*math.cos(theta)
        elif Length<0: 
            Force= self.ChiliParameter*100/-Length
            ForceEast=Force*math.sin(theta)
            ForceNorth=Force*math.cos(theta)
        elif Length==0:
            ForceEast=ForceNorth=0
        return ForceEast,ForceNorth,0
    def LeftDistance2Obs(self,position):
        """同水平面上我放飞机与威胁区所围圆形的剩余距离"""
        Plane_lon,Plane_lat,Plane_alt=position
        if self.radius**2-Plane_alt**2<0:
            return 90000
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)-math.sqrt(self.radius**2-Plane_alt**2)
        return Length
    
    
def TargetDist(DroneID,TargetID,info,YinliParameter):
    """依次返回东、北、上三个方向的斥力信息"""
    Plane_lon,Plane_lat,Plane_alt=RDer.GetPosition(info,DroneID)
    if info.DroneID==DroneID:
        x=info.Longtitude
        y=info.Latitude
        z=info.Altitude
        vn1=info.V_N
        ve1=info.V_E
        index=-1
        #判断是否找到目标敌机，若没有则引力返回0，0，0
        for i in range(len(info.FoundEnemyList)):
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                index=i
                break
        if index==-1:
            return 0,0,0
        #若找到目标敌机，则引力返回目标敌机与自身之间的距离和引力系数的乘积
        else:
            l=info.FoundEnemyList[i].TargetDis
            h1=info.FoundEnemyList[i].TargetAlt
            theta=info.FoundEnemyList[i].TargetYaw
            goal_lon,goal_lat,goal_alt=CABP(x,y,z,vn1,ve1,l,h1,theta)
            goal_lon=RDer.superr2d(goal_lon)
            goal_lat=RDer.superr2d(goal_lat)
            goal_alt=goal_alt
        DisEast=RDer.LongituteDis((goal_lon-Plane_lon),Plane_lat)#飞机在目标东面为正数，在西面为负数
        DisNorth=RDer.LatitudeDis(goal_lat-Plane_lat)#飞机在目标北面为正数，在南面为负数
        DisUp=(goal_alt-Plane_alt)
        DisEast=90000 if DisEast>90000 else DisEast
        DisEast=-90000 if DisEast<-90000 else DisEast
        DisNorth=90000 if DisNorth>90000 else DisNorth
        DisNorth=-90000 if DisNorth<-90000 else DisNorth
        return YinliParameter*DisEast,YinliParameter*DisNorth,YinliParameter*DisUp
    

global ZhuiJiMode     
ZhuiJiMode=[0,0,0,0]#0表示探测到目标，1表示没有目标

global biaoji_a
biaoji_a=[0,0,0,0]

def APF_Valpha(output_cmd,info,DroneID,TargetID,mp,obstacle,Spd_PingFei,Thrust_PingFei,Spd_PaSheng,Thrust_PaSheng,YinliParameter):
    """TargetID"""
    global biaoji_a
    Foundflag=0
    if TargetID==0:
        TargetID=GetTargetID(info,DroneID)
    if TargetID==404:
        TargetID=GetTargetID(info,DroneID)
    if info.DroneID==DroneID:
        JDDZer=JDDZ(output_cmd,info,DroneID)
        Mapper=Mp(mp.lon_start,mp.lon_end,mp.lat_start,mp.lat_end,mp.ChiliParameter)
        ob=Obstacle(info,obstacle.lon,obstacle.lat,obstacle.alt,obstacle.radius,obstacle.ChiliParameter)
        ForceEast1,ForceNorth1,ForceUp1=TargetDist(DroneID,TargetID,info,YinliParameter)#获取引力信息
        ForceEast2,ForceNorth2,ForceUp2=Mapper.distance2boundary(RDer.GetPosition(info,DroneID))#获取战场边界斥力信息
        ForceEast3,ForceNorth3,ForceUp3=ob.distance2Obs(RDer.GetPosition(info,DroneID))#获取危险区斥力信息
       #判断平飞速度是否能够追上敌机
        for i in range(len(info.FoundEnemyList)): 
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                Foundflag=1
                if (info.FoundEnemyList[i].TargetV_N)*(info.V_N)>0 and (info.FoundEnemyList[i].TargetV_E)*(info.V_E)>0: #判断敌方在接近还是远离我方
                    if info.Mach_M - 0.3 < info.FoundEnemyList[i].TargetMach_M:#判断我方速度是否小于敌方速度
                       Spd_PingFei=info.FoundEnemyList[i].TargetMach_M+0.3#如果小于敌方速度，则将平飞速度设为敌方速度+0.3
                       Thrust_PingFei=(Spd_PingFei)*100+40#调节油门大小
        if Foundflag==0:
            TargetID=GetTargetID(info,DroneID)
             
        elif Foundflag!=0:
            biaoji_a[int(DroneID/100000)-1]=0
                        
        #引力为0代表没有探测到目标，将原地盘旋等待雷达探测到目标
        if ForceEast1==0 and ForceNorth1==0 and ForceUp1==0 and ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=10000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>=7000:
            ZhuiJiMode[int(DroneID/100000)-1]=0
        #引力不为0代表探测到目标，此时需要判断是否需要考虑斥力
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:#离威胁区和边界较远，忽略斥力，直接追击敌方
            ZhuiJiMode[int(DroneID/100000)-1]=1
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:#离危险区较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=2
            biaoji_a[int(DroneID/100000)-1]=0
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID)) < 3000:#离战场边界较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=3
            biaoji_a[int(DroneID/100000)-1]=0
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))<3000  :#离战场边界较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=4
            biaoji_a[int(DroneID/100000)-1]=0
            
        if ZhuiJiMode[int(DroneID/100000)-1]==0:#盘旋等待敌方出现
            if (-math.pi < info.Yaw < 0 )and biaoji_a[int(DroneID/100000)-1]==0 :
                biaoji_a[int(DroneID/100000)-1]=1#逆时针转动
            elif (0 < info.Yaw < math.pi )and biaoji_a[int(DroneID/100000)-1]==0:
                biaoji_a[int(DroneID/100000)-1]=2#顺时针转动
            if biaoji_a[int(DroneID/100000)-1]==1:
                JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)-120,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif biaoji_a[int(DroneID/100000)-1]==2:
                JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)+120,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                
        #离威胁区较远可以忽略威胁区斥力，直接追击敌方
        elif ZhuiJiMode[int(DroneID/100000)-1]==1:
            if ForceEast1==0 and ForceNorth1==0 and biaoji_a[int(DroneID/100000)-1] != 0 :
                if biaoji_a[int(DroneID/100000)-1]==1:
                    JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)-120,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                elif biaoji_a[int(DroneID/100000)-1] == 2:
                    JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)+120,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif ForceEast1==0 and ForceNorth1==0 :
                theta_rad=np.arctan2(ForceEast2+ForceEast3,ForceNorth2+ForceNorth3)
                theta_deg=RDer.superr2d(theta_rad)
                if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID) or not Mapper.is_inside(RDer.GetPosition(info,DroneID)):
                    JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                    JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
            elif ForceEast1!=0 or ForceNorth1!=0 :    
                theta_rad=np.arctan2(ForceEast1,ForceNorth1)
                theta_deg=RDer.superr2d(theta_rad)
                if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID) or not Mapper.is_inside(RDer.GetPosition(info,DroneID)):
                    JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                    JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                if  ForceUp1>3000*YinliParameter and info.Altitude<12000:
            #当敌方飞机高于我方3000m以上时，我方将爬升来追击敌方     
                    JDDZer.PaSheng(theta_deg,Spd_PaSheng,info.Altitude+200,Thrust_PaSheng,Thrust_PingFei)
            #当敌方低于我方3000m以上时，我方向下俯冲500m来追击敌方 
                if ForceUp1<-3000*YinliParameter or info.Altitude>12000:
                    JDDZer.FuChong(Spd_PaSheng,info.Altitude-500,-40,theta_deg,Thrust_PingFei)     
                    
        #离威胁区较近需要考虑威胁区斥力影响
        elif ZhuiJiMode[int(DroneID/100000)-1]==2:
            theta_rad=np.arctan2(ForceEast1+ForceEast3,ForceNorth1+ForceNorth3)
            theta_deg=RDer.superr2d(theta_rad)
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 :
                JDDZer.ZhuanWan(60,theta_deg,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                
        #离战场边界较近需要考虑边界斥力影响
        elif ZhuiJiMode[int(DroneID/100000)-1]==3:
            theta_rad=np.arctan2(ForceEast1+ForceEast2,ForceNorth1+ForceNorth2)
            theta_deg=RDer.superr2d(theta_rad)
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 :
                JDDZer.ZhuanWan(60,theta_deg,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                
        #离战场边界和威胁区较近需要考虑两者斥力影响        
        elif ZhuiJiMode[int(DroneID/100000)-1]==4:
            theta_rad=np.arctan2(ForceEast1+ForceEast2+ForceEast3,ForceNorth1+ForceNorth2+ForceNorth3)#根据力的分量大小求出水平面上力的方向
            theta_deg=RDer.superr2d(theta_rad)#转换为平飞航向代码所需的航向（角度制）
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20:
                JDDZer.ZhuanWan(60,theta_deg,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei) 
                
global biaoji
biaoji=[0,0,0,0]                         
def APF_Vbeta(output_cmd,info,DroneID,TargetID,mp,obstacle,Spd_PingFei,Thrust_PingFei,Spd_PaSheng,Thrust_PaSheng,YinliParameter):
    """TargetID"""
    global biaoji
    Foundflag=0
    if TargetID==0:
        TargetID=GetTargetID(info,DroneID)
    if TargetID==404:
        TargetID=GetTargetID(info,DroneID)
    if info.DroneID==DroneID:
        JDDZer=JDDZ(output_cmd,info,DroneID)
        Mapper=Mp(mp.lon_start,mp.lon_end,mp.lat_start,mp.lat_end,mp.ChiliParameter)
        ob=Obstacle(info,obstacle.lon,obstacle.lat,obstacle.alt,obstacle.radius,obstacle.ChiliParameter)
        ForceEast1,ForceNorth1,ForceUp1=TargetDist(DroneID,TargetID,info,YinliParameter)#获取引力信息
        ForceEast2,ForceNorth2,ForceUp2=Mapper.distance2boundary(RDer.GetPosition(info,DroneID))#获取战场边界斥力信息
        ForceEast3,ForceNorth3,ForceUp3=ob.distance2Obs(RDer.GetPosition(info,DroneID))#获取危险区斥力信息
       #判断平飞速度是否能够追上敌机
        for i in range(len(info.FoundEnemyList)): 
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                Foundflag=1
                if (info.FoundEnemyList[i].TargetV_N)*(info.V_N)>0 and (info.FoundEnemyList[i].TargetV_E)*(info.V_E)>0: #判断敌方在接近还是远离我方
                    if info.Mach_M - 0.3 < info.FoundEnemyList[i].TargetMach_M:#判断我方速度是否小于敌方速度
                       Spd_PingFei=info.FoundEnemyList[i].TargetMach_M+0.3#如果小于敌方速度，则将平飞速度设为敌方速度+0.3
                       Thrust_PingFei=(Spd_PingFei)*100+40#调节油门大小
        if Foundflag==0:
            TargetID=GetTargetID(info,DroneID)
             
        elif Foundflag!=0:
            biaoji[int(DroneID/100000)-1]=0
                        
        #引力为0代表没有探测到目标，将原地盘旋等待雷达探测到目标
        if ForceEast1==0 and ForceNorth1==0 and ForceUp1==0 :
            ZhuiJiMode[int(DroneID/100000)-1]=0
            biaoji[int(DroneID/100000)-1]=0
        #引力不为0代表探测到目标，此时需要判断是否需要考虑斥力
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:#离威胁区和边界较远，忽略斥力，直接追击敌方
            ZhuiJiMode[int(DroneID/100000)-1]=1
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:#离危险区较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=2
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID)) < 3000:#离战场边界较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=3
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<4000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))<3000  :#离战场边界较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=4
            
        if ZhuiJiMode[int(DroneID/100000)-1]==0:
            if ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=7000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:
                biaoji[int(DroneID/100000)-1]=1
            else:
                biaoji[int(DroneID/100000)-1]=2
            if biaoji[int(DroneID/100000)-1]==1:
                plane_lon,plane_lat,plane_alt = RDer.GetPosition(info,DroneID)
                distanceast=RDer.LongituteDis(ob.lon-plane_lon,plane_lat)
                distancenorth=RDer.LatitudeDis(ob.lat-plane_lat)
                theta_center=RDer.superr2d(np.arctan2(distanceast,distancenorth)) #朝向威胁区中心偏航向
                if plane_lon<ob.lon:
                    theta_aim2=theta_center+90
                else:
                    theta_aim2=theta_center-90
                if abs(RDer.superr2d(info.Yaw)-theta_aim2)>20:
                    JDDZer.ZhuanWan(60,theta_aim2,4,Spd_PingFei,1,Thrust_PaSheng)
                else:
                    JDDZer.PingFei(theta_aim2,1.5,250)
                
            if biaoji[int(DroneID/100000)-1]==2:
                theta_rad=np.arctan2(ForceEast2+ForceEast3,ForceNorth2+ForceNorth3)
                theta_deg=RDer.superr2d(theta_rad)
                if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID) or not Mapper.is_inside(RDer.GetPosition(info,DroneID)):
                    JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                    JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                
        #离威胁区较远可以忽略威胁区斥力，直接追击敌方
        elif ZhuiJiMode[int(DroneID/100000)-1]==1:
            theta_rad=np.arctan2(ForceEast1+ForceEast2+ForceEast3,ForceNorth1+ForceNorth2+ForceNorth3)
            theta_deg=RDer.superr2d(theta_rad)
            mode=1
            plane_lon,plane_lat,plane_alt = RDer.GetPosition(info,DroneID)
            distanceast=RDer.LongituteDis(ob.lon-plane_lon,plane_lat)
            distancenorth=RDer.LatitudeDis(ob.lat-plane_lat)
            theta_center=RDer.superr2d(np.arctan2(distanceast,distancenorth)) #朝向威胁区中心偏航向
            if plane_lon<ob.lon:
                theta_aim2=theta_center+90
            else:
                theta_aim2=theta_center-90
            if theta_aim2==theta_center+90:
                if theta_aim2-theta_deg>50 and plane_lat>24.5:
                    mode=1  #mode1,说明敌机方位连线与切线角度过大，经过威胁区，故忽略敌机方位连线与切线夹角，直接飞向切线方向，快速通过威胁区
                else:
                    mode=2  #mode2,说明敌机方位连线与切线夹角较小，故计算敌机和威胁区合力大小，选择合力方向飞行
            elif theta_aim2==theta_center-90:
                if theta_deg>theta_aim2+50 and plane_lat>24.5:
                    mode=1
                else:
                    mode=2
                
            if mode==1:
                if ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>=7000 and Mapper.distanceleft2boundary(RDer.GetPosition(info,DroneID))>= 3000:
                    if abs(RDer.superr2d(info.Yaw)-theta_aim2)>20:
                        JDDZer.ZhuanWan(60,theta_aim2,4,Spd_PingFei,1,Thrust_PaSheng)
                    else:
                        JDDZer.PingFei(theta_aim2,1.5,150)
                    
                else:
                    theta_rad=np.arctan2(ForceEast2+ForceEast3,ForceNorth2+ForceNorth3)
                    theta_deg=RDer.superr2d(theta_rad)
                    if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID) or not Mapper.is_inside(RDer.GetPosition(info,DroneID)):
                        JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust=Thrust_PaSheng)
                    elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                        JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                    
            elif mode==2:
                if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID) or not Mapper.is_inside(RDer.GetPosition(info,DroneID)):
                    JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust_PaSheng)
                elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                    JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
                if  ForceUp1>3000*YinliParameter and info.Altitude<12000:
                #当敌方飞机高于我方3000m以上时，我方将爬升来追击敌方     
                    JDDZer.PaSheng(theta_deg,Spd_PaSheng,info.Altitude+200,Thrust_PaSheng,Thrust_PingFei)
                #当敌方低于我方3000m以上时，我方向下俯冲500m来追击敌方 
                if ForceUp1<-3000*YinliParameter or info.Altitude>12000:
                    JDDZer.FuChong(Spd_PaSheng,info.Altitude-500,-40,theta_deg,Thrust_PingFei) 
                  
        #离威胁区较近需要考虑威胁区斥力影响
        elif ZhuiJiMode[int(DroneID/100000)-1]==2:
            theta_rad=np.arctan2(ForceEast1+ForceEast3,ForceNorth1+ForceNorth3)
            theta_deg=RDer.superr2d(theta_rad)
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 :
                JDDZer.ZhuanWan(60,theta_deg,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
            
        #离战场边界较近需要考虑边界斥力影响
        elif ZhuiJiMode[int(DroneID/100000)-1]==3:
            theta_rad=np.arctan2(ForceEast1+ForceEast2,ForceNorth1+ForceNorth2)
            theta_deg=RDer.superr2d(theta_rad)
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 :
                JDDZer.ZhuanWan(60,theta_deg,8,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
             
                
        #离战场边界和威胁区较近需要考虑两者斥力影响        
        elif ZhuiJiMode[int(DroneID/100000)-1]==4:
            theta_rad=np.arctan2(ForceEast1+ForceEast2+ForceEast3,ForceNorth1+ForceNorth2+ForceNorth3)#根据力的分量大小求出水平面上力的方向
            theta_deg=RDer.superr2d(theta_rad)#转换为平飞航向代码所需的航向（角度制）
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20:
                JDDZer.ZhuanWan(60,theta_deg,4,Spd_PingFei,1,Thrust=Thrust_PaSheng)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)          