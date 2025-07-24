# -*- coding: utf-8 -*-
import socket
import threading
from ctypes import *
import ClassName
import AiScenarioDef
import time

platforms = dict()
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.connect(("192.168.50.35", 60000))


class Header(Structure):
    _pack_ = 1
    _fields_ = [
        ("size", c_int),
        ("state", c_char)
    ]


class DataService(threading.Thread):
    def __init__(self, gft_ip, gft_point):
        threading.Thread.__init__(self)
        self.gft_ip, self.gft_point = gft_ip, gft_point
        self.server_socket = server_socket

    def run(self):
        state = 0
        while True:
            if state ==0:
                print(f"GFT {self.gft_ip, self.gft_point} 启动！")
                time.sleep(3)
                #  全体无人机定义
                data = AiScenarioDef.ST_AI_SCENARIO()
                data.m_type =1
                data.m_plane_count = 8
                plane1= AiScenarioDef.m_plane_info()
                plane2 = AiScenarioDef.m_plane_info()
                plane3 = AiScenarioDef.m_plane_info()
                plane4 = AiScenarioDef.m_plane_info()
                plane5 = AiScenarioDef.m_plane_info()
                plane6 = AiScenarioDef.m_plane_info()
                plane7 = AiScenarioDef.m_plane_info()
                plane8 = AiScenarioDef.m_plane_info()

                #  红蓝双方所有无人机初始态势设置
                plane1.m_lon = 123.6
                plane1.m_lat = 41.6
                plane1.m_alt = 5000
                plane1.m_heading = 0
                plane1.m_speed_m_s = 340
                plane1.m_oil = 5000
                plane1.m_entity_id = 100000
                plane1.m_camp = 1
                plane1.m_leader = 1
                plane1.m_form_id = 0
                plane1.m_number = 1
    
                plane2.m_lon = 123.8
                plane2.m_lat = 41.6
                plane2.m_alt =5000
                plane2.m_heading = 0
                plane2.m_speed_m_s = 340
                plane2.m_oil = 5000
                plane2.m_entity_id = 200000
                plane2.m_camp = 1
                plane2.m_leader = 1
                plane2.m_form_id = 0
                plane2.m_number = 2
    
                plane3.m_lon = 124
                plane3.m_lat = 41.6
                plane3.m_alt =5000
                plane3.m_heading = 0
                plane3.m_speed_m_s = 340
                plane3.m_oil = 5000
                plane3.m_entity_id = 300000
                plane3.m_camp = 1
                plane3.m_leader = 1
                plane3.m_form_id = 0
                plane3.m_number = 3
    
                plane4.m_lon = 124.2
                plane4.m_lat = 41.6
                plane4.m_alt = 5000
                plane4.m_heading = 0
                plane4.m_speed_m_s = 340
                plane4.m_oil = 5000
                plane4.m_entity_id = 400000
                plane4.m_camp = 1
                plane4.m_leader = 1
                plane4.m_form_id = 0
                plane4.m_number = 4
    
                plane5.m_lon = 123.6
                plane5.m_lat = 42.8
                plane5.m_alt =5000
                plane5.m_heading = 180
                plane5.m_speed_m_s = 400
                plane5.m_oil = 5000
                plane5.m_entity_id = 500000
                plane5.m_camp = 2
                plane5.m_leader = 1
                plane5.m_form_id = 0
                plane5.m_number = 5
    
                plane6.m_lon = 123.8
                plane6.m_lat = 42.8
                plane6.m_alt = 5000
                plane6.m_heading = 180
                plane6.m_speed_m_s = 400
                plane6.m_oil = 5000
                plane6.m_entity_id = 600000
                plane6.m_camp = 2
                plane6.m_leader = 1
                plane6.m_form_id = 0
                plane6.m_number = 6
    
                plane7.m_lon = 124
                plane7.m_lat = 42.8
                plane7.m_alt = 5000
                plane7.m_heading = 180
                plane7.m_speed_m_s = 400
                plane7.m_oil = 5000
                plane7.m_entity_id = 700000
                plane7.m_camp = 2
                plane7.m_leader = 1
                plane7.m_form_id = 0
                plane7.m_number = 7
    
                plane8.m_lon = 124.2
                plane8.m_lat = 42.8
                plane8.m_alt = 5000
                plane8.m_heading =180
                plane8.m_speed_m_s = 400
                plane8.m_oil = 5000
                plane8.m_entity_id = 800000
                plane8.m_camp = 2
                plane8.m_leader = 1
                plane8.m_form_id = 0
                plane8.m_number = 8


                data.m_plane_info_data[0] = plane1
                data.m_plane_info_data[1] = plane2
                data.m_plane_info_data[2] = plane3
                data.m_plane_info_data[3] = plane4
                data.m_plane_info_data[4] = plane5
                data.m_plane_info_data[5] = plane6
                data.m_plane_info_data[6] = plane7
                data.m_plane_info_data[7] = plane8

                # 头
                header = Header()
                header.size = sizeof(data) + 1
                header.state = 0
                output = string_at(addressof(header), sizeof(header))
                output += string_at(addressof(data), sizeof(data)) # 只发state时不需要
                self.server_socket.send(output)
                print('send success')
    
                recv_buff = self.server_socket.recv(sizeof(ClassName.Commu))
                situation = ClassName.Commu()
                memmove(addressof(situation), recv_buff, sizeof(ClassName.Commu))

                hexstate = int(situation.state.hex())
                if hexstate ==1:
                    print('state 1 get')
                    state = 1
                    continue
                else:
                    print("get wrong message")
                    continue

            if state ==1:

                # 头
                header2 = Header()
                header2.size = 1
                header2.state = 2
                output = string_at(addressof(header2), sizeof(header2))
                self.server_socket.send(output)

                # 接
                recv_buff2 = self.server_socket.recv(sizeof(ClassName.Commu))
                situation2 = ClassName.Commu()
                memmove(addressof(situation2), recv_buff2, sizeof(ClassName.Commu))

                hexstate = int(situation2.state.hex())
                if hexstate == 3:
                    print('state 1 init finished')
                    state = 2
                    continue
                else:
                    continue


            if state == 2:
                print('state 2 init finished')
                # 头
                header3 = Header()
                header3.size = 1
                header3.state = 4
                output = string_at(addressof(header3), sizeof(header3))
                self.server_socket.send(output)
                state = 4

        self.server_socket.close()

    def close(self):
        self.server_socket.close()


if __name__ == '__main__':
    # 仿真端（虚拟机）ip 端口
    data_serv = DataService("192.168.50.35", 60000)
    data_serv.run()

