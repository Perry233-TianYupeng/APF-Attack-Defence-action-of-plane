import numpy as np
import KU_v18 as KU
import math

JDDZ=KU.JDDZ()
RDer=KU.RD()
#力的正方向分别为东，北，上
def GetTargetID(info,DroneID):
    """获取Drone的FoundEnemyList中距离最近的敌方飞机的ID"""
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
            
class Mp(JDDZ):#建立战场类
    def __init__(self,lon_start,lon_end,lat_start,lat_end,Chiliparameter):# 初始化函数，用于创建一个战场区域对象
        self.lon_start = lon_start # 设置战场区域的起始经度
        self.lon_end = lon_end# 设置战场区域的结束经度
        self.lat_start = lat_start# 设置战场区域的起始纬度
        self.lat_end = lat_end# 设置战场区域的结束纬度
        self.Chiliparameter=Chiliparameter # 设置斥力参数

    def is_inside(self,position):
        """Position为GetPosition（info，DroneID）的返回值，判断飞机是否在威胁区内"""
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
        Fright=self.Chiliparameter/abs(DisMapLonLeft)#正数，东向力
        Fleft=-self.Chiliparameter/abs(DisMapLonRight)#负数，西向力
        Fup=self.Chiliparameter/abs(DisMapLatDown)#正数，北向力
        Fdown=-self.Chiliparameter/abs(DisMapLatUp)#负数，南向力
        return Fright+Fleft,Fup+Fdown,0
    
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
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)
        theta=math.acos((RDer.LongituteDis((Plane_lon-self.lon),self.lat))/Length)
        if Plane_lon>=self.lon:
            DisEast=RDer.LongituteDis((Plane_lon-self.lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*math.cos(theta)#飞机在障碍的东面为正数，在西面为负数
        else:
            DisEast=-(RDer.LongituteDis((self.lon-Plane_lon),self.lat)+(math.sqrt(self.radius**2-Plane_alt**2))*math.cos(theta))
        if Plane_lat>=self.lat:
            DisNorth=RDer.LatitudeDis(Plane_lat-self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*math.sin(theta)#飞机在障碍的北面为正数，在南面为负数
        else:
            DisNorth=-RDer.LatitudeDis(self.lat-Plane_lat)+(math.sqrt(self.radius**2-Plane_alt**2))*math.sin(theta)
        return self.ChiliParameter/DisEast,self.ChiliParameter/DisNorth,0
    def LeftDistance2Obs(self,position):
        """同水平面上的剩余距离"""
        Plane_lon,Plane_lat,Plane_alt=position
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)-(math.sqrt(self.radius**2-Plane_alt**2))
        return Length
def TargetDist(DroneID,TargetID,info,YinliParameter):
    """依次返回东、北、上三个方向的斥力信息"""
    Plane_lon,Plane_lat,Plane_alt=RDer.GetPosition(info,DroneID)
    if info.DroneID==DroneID:
        x=RDer.superr2d(info.Longtitude)
        y=RDer.superr2d(info.Latitude)
        z=info.Altitude
        vn1=info.V_N
        ve1=info.V_E
        index=-1
        for i in range(len(info.FoundEnemyList)):
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                index=i
                break
        if index==-1:
            return 0,0,0
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
        return YinliParameter*DisEast,YinliParameter*DisNorth,YinliParameter*DisUp
    
global ZhuiJiMode     
ZhuiJiMode=[0,0,0,0]#0表示探测到目标，1表示没有目标
def APF_Valpha(output_cmd,info,DroneID,TargetID,lon_start,lon_end,lat_start,lat_end,obstacle_lon,obstacle_lat,obstacle_alt,obstacle_radius,Spd_PingFei,Thrust_PingFei,Spd_PaSheng,Thrust_PaSheng,ChiliParameter,YinliParameter):
    """TargetID"""
    if TargetID==0:
        TargetID=GetTargetID(info,DroneID)
    if TargetID==404:
        TargetID=GetTargetID(info,DroneID)
    if info.DroneID==DroneID:
        JDDZer=JDDZ(output_cmd,info)
        Mapper=Mp(lon_start,lon_end,lat_start,lat_end,ChiliParameter)
        ob=Obstacle(info,obstacle_lon,obstacle_lat,obstacle_alt,obstacle_radius,ChiliParameter)
        ForceEast1,ForceNorth1,ForceUp1=TargetDist(DroneID,TargetID,info,YinliParameter)#获取引力信息
        ForceEast2,ForceNorth2,ForceUp2=Mapper.distance2boundary(RDer.GetPosition(info,DroneID))#获取战场边界斥力信息
        ForceEast3,ForceNorth3,ForceUp3=ob.distance2Obs(RDer.GetPosition(info,DroneID))#获取危险区斥力信息
        print(info.DroneID,"引力:",ForceEast1,ForceNorth1,ForceUp1)
        print(info.DroneID,"边界斥力: ",ForceEast2,ForceNorth2,ForceUp2)
        print(info.DroneID,"危险区斥力: ",ForceEast3,ForceNorth3,ForceUp3)
        print(info.DroneID,"追击对象",TargetID)        
        if ForceEast1==0 and ForceNorth1==0 and ForceUp1==0 and math.sqrt((ForceEast2+ForceEast3)**2+(ForceNorth2+ForceNorth3)**2)<150:
            ZhuiJiMode[int(DroneID/100000)-1]=0
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>6000:
            ZhuiJiMode[int(DroneID/100000)-1]=1
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<6000:
            ZhuiJiMode[int(DroneID/100000)-1]=2
        elif  ZhuiJiMode[int(DroneID/100000)-1]==0 and math.sqrt((ForceEast2+ForceEast3)**2+(ForceNorth2+ForceNorth3)**2)>300:
            ZhuiJiMode[int(DroneID/100000)-1]=2
        if ZhuiJiMode[int(DroneID/100000)-1]==0:
            JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)+30,4,Spd_PingFei,1,Thrust=Thrust_PingFei)
        elif ZhuiJiMode[int(DroneID/100000)-1]==1:
            theta_rad=np.arctan2(ForceEast1+ForceEast2,ForceNorth1+ForceNorth2)
            theta_deg=RDer.superr2d(theta_rad)
            JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
        elif ZhuiJiMode[int(DroneID/100000)-1]==2:
            matrix=np.array([[ForceEast1,ForceNorth1,ForceUp1],[ForceEast2,ForceNorth2,ForceUp2],[ForceEast3,ForceNorth3,ForceUp3]])#利用一个矩阵来存储三组力
            Force_sums = np.sum(matrix, axis=0)#对矩阵按列求和，得到东、北、上三个方向的力的分量大小（为负数即为反向）
            theta_rad=np.arctan2(Force_sums[0],Force_sums[1])#根据力的分量大小求出水平面上力的方向
            theta_deg=RDer.superr2d(theta_rad)#转换为平飞航向代码所需的航向（角度制）
            i=-1
            for i in range(len(info.FoundEnemyList)):
                if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                    break
            h1=info.FoundEnemyList[i].TargetAlt
            JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
            if  (h1-info.Altitude)>400 and abs(theta_deg-RDer.r2d(info.Yaw))<20:
                JDDZer.PaSheng(theta_deg,Spd_PaSheng,(h1+info.Altitude)/2+500,Thrust_PaSheng,Thrust_PingFei)