#!/usr/bin/python3
import json 
import struct
import socket
import time
import datetime
import signal
import rospy
import os
import sys
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgGpsHdt
from shapely.geometry import Point, Polygon
from std_msgs.msg import Int16MultiArray

HOST = 'localhost'
PORT = 9100

class V2X:
    def __init__(self):
        self.pvd_temp = '{"startVector":{"long":1268905411,"lat":375781661,"heading":8800, "speed":{"transmisson":"unavailable","speed":738}},"vehicleType":{"vehicleType":"cars"},"snapshots":[{"thePosition":{"long":1268905411,"lat":375781661, "heading" : 8800,"speed":{"transmisson":"unavailable","speed":738}},"dataSet":{"lights":{"value":"C800","length":9},"wipers":{"statusFront":"high","rateFront":100,"statusRear":"low","rateRear":50},"brakeStatus":{"wheelBrakes":"C0","traction":"on","abs":"on","scs":"on","brakeBoost":"off","auxBrakes":"off"},"brakePressure":"minPressure"}}]}'
        self.pvd = json.loads(self.pvd_temp)
        self.toc = 0
        self.sock = None
        self.bag = {'long_bag' : 0, 'lat_bag' : 0}
        self.head = 0
        # self.msg = False
    
        rospy.init_node('V2X_node', anonymous = True)
        rospy.Subscriber("sbg/ekf_nav",SbgEkfNav, self.ekf_nav)
        rospy.Subscriber("sbg/gps_hdt",SbgGpsHdt, self.gps_hdt)
        self.SPaT = rospy.Publisher('/spat_msg', Int16MultiArray, queue_size = 1)
    
    def spat(self, data):
        spat_msg = Int16MultiArray()  ## convert 'eventstate' to int
        os.system('clear')
        print('SPaT-----------------------------------------------------')
        for sections in data['intersections']:
            print(sections['name'])
            # print(self.head)
            for states in data['intersections'][0]['states']:
                state = states['state-time-speed']
                eventstate = state[0]['eventState']
                es_int = self.signalstate(eventstate)
                timing = state[0]['timing']['minEndTime']
                if (sections['name'] == 'World Cup Park 207 dong' or sections['name'] == 'World Cup Park 2 Complex' or sections['name'] == 'Sangam Police Station' 
                    or sections['name'] == 'Sangam Elementary School SUB' or sections['name'] == 'Sangam Elementary School') and states['signalGroup'] == 50:
                    print('signal 50 : {0} , minEndTime : {1}'.format(eventstate,timing))
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing]) 
                elif (sections['name'] == 'World Cup Park 4 Complex' or sections['name'] == 'World Cup Park 5 Complex') and states['signalGroup'] == 60:
                    print('signal 60 : {0} , minEndTime : {1}'.format(eventstate,timing))
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing]) 
                elif (sections['name'] == '407 World Cup Park' or sections['name'] == 'Sangam Savoy City DMC') and states['signalGroup'] == 70:
                    print('signal 70 : {0} , minEndTime : {1}'.format(eventstate,timing))
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing]) 
                elif (sections['name'] == 'Sangam MBC SUB' or sections['name'] == 'Sangam SBS' or sections['name'] == 'Seongam three-way street') and states['signalGroup'] == 80:
                    print('signal 80 : {0} , minEndTime : {1}'.format(eventstate,timing))
                    print(states['movementName'])
                    spat_msg.data.extend([es_int,timing])
                elif sections['name'] == 'Digital Media City' and states['signalGroup'] == 60:
                    if self.head >= 270 and self.head <= 330:
                        print('signal 60 : {0} , minEndTime : {1}'.format(eventstate,timing))
                        print(states['movementName'])
                        spat_msg.data.extend([es_int,timing])
                elif sections['name'] == 'Digital Media City' and states['signalGroup'] == 70:
                    if self.head >= 340 or self.head <= 50:
                        print('signal 70 : {0} , minEndTime : {1}'.format(eventstate,timing))
                        print(states['movementName'])
                        spat_msg.data.extend([es_int,timing])
                elif sections['name'] == 'Sangam MBC' and states['signalGroup'] == 70:
                    print('signal 70 : {0} , minEndTime : {1}'.format(eventstate,timing))
                    print(states['movementName'])
                    if states['movementName'] == 'LEFT':
                        spat_msg.data = [0,0]
                    spat_msg.data.extend([es_int,timing])

        print(spat_msg.data)
        self.SPaT.publish(spat_msg)
        spat_msg.data = []

    def signalstate(self, signal):
        temp = 0
        if signal =='stop-Then-Proceed' or signal =='stop-And-Remain':
            temp = 1  # red
        elif signal =='pre-Movement' or signal =='permissive-Movement-Allowed' or signal =='protected-Movement-Allowed' or signal =='permissive-clearance':
            temp = 2  # green
        elif signal =='protected-clearance':
            temp = 3  # yellow
        else:
            pass
        return temp

    def ekf_nav(self,data):
        # self.bag['long_bag'] = data.position.y
        # self.bag['lat_bag'] = data.position.x
        self.bag['long_bag'] = data.longitude
        self.bag['lat_bag'] = data.latitude
        
    def gps_hdt(self,data):
        head = data.true_heading
        if (head >= 0 and head <= 180):
            self.head = head + 180
        else:
            self.head = head - 180
            # self.msg = True

    def getMsg(self): 
        msg = [0x61]
        data = self.sock.recv(9192)
        leng, typ= struct.unpack('>IB', data[:5]) 
        # print(hex(typ))
        if typ in msg:
            real_data = data[5:]
            real_data = json.loads(real_data.decode())
            # print(data[5:])
            self.spat(real_data)

    def dumper(self):
        self.pvd['startVector']['long'] = int(round(self.bag['long_bag'] * 10000000))
        self.pvd['startVector']['lat'] = int(round(self.bag['lat_bag'] * 10000000))
        self.pvd['startVector']['heading'] = int(round(self.head / 0.0125 ))
        self.pvd['snapshots'][0]['thePosition']['long'] = int(round(self.bag['long_bag'] * 10000000))
        self.pvd['snapshots'][0]['thePosition']['lat'] = int(round(self.bag['lat_bag'] * 10000000))
        self.pvd['snapshots'][0]['thePosition']['heading'] = int(round(self.head / 0.0125 ))
        data = json.dumps(self.pvd)
        return data

    def packer(self, data):
        packet = struct.pack('>IB{}s'.format(len(data)), len(data)+5, 0x01, data.encode())
        return packet

    def timer(self, tic):
        if (time.time() - self.toc > tic):
            self.toc = time.time()
            return True
        else:
            return False

    def createConn(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        print('connected')

    def daemon(self):
        if (self.timer(0.05) == True):
            data = self.dumper()
            data = self.packer(data)
            self.sock.sendall(data)
            # self.msg = False

def signal_handler(sig, frame):
    print('\nPressed CTRL + C !')
    sys.exit(0)

if __name__ == '__main__':
    V2X = V2X()
    V2X.createConn()
    while 1:
        try:
            signal.signal(signal.SIGINT, signal_handler)
            V2X.daemon()        
            V2X.getMsg()
        except ValueError:
            pass
        except Exception as e:
            print(e)
            print(type(e))
            exit()

