
import sys
# import random
import numpy as np

from global_map import Map

from path_planner import Path_Planner

# from task_allocation import task_allocation_main
from simple_allocation import task_allocation_main
from threading import Thread, Event
import threading
import multiprocessing

# import concurrent.futures
import queue
# 调试用延时，确保有足够的时间attach到进程
import time
from queue import Empty

n = 200
robot_num = 10
berth_num = 10
boat_num = 5
N = 210
gbmap = Map()

money = 0 # 机器人的货物价值
boat_capacity = 0 # 船只的装载量
id = 0 # 当前帧id
ch = [] # 用于存储地图信息
# gds = [[0 for _ in range(N)] for _ in range(N)] # 用于存储各个物品的价值信息
robot_cods = [] # 用于存储机器人的坐标信息


class Robot:
    def __init__(self, num = 0, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.num = num
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.last_status = status
        self.mbx = mbx # 机器人移动的目标点
        self.mby = mby # 机器人移动的目标点

        self.mb_berth_id = 0 # 机器人移动的目标泊位id

        self.allocation_status = 0 # 0表示未分配任务，1表示已分配任务

        self.path_list = []

        self.get_away_from_crash_success = 1 # 标志位

    def get_good(self):
        """机器人拿取货物"""
        print("get",self.num) 
        sys.stdout.flush()

    def pull_good(self):
        """机器人卸货"""
        print("pull", self.num)  
        sys.stdout.flush()

    def get_away_from_crash(self):
        '''先向可行方向走一步，拉开距离'''
        
        if gbmap.is_available(self.x, self.y - 1, self): # 如果左侧空闲，那就左移一步
            print("move", self.num, '1')
            sys.stdout.flush()
            self.get_away_from_crash_success = 1
            return
        elif gbmap.is_available(self.x - 1, self.y, self): # 如果上面空闲，那就上移一步
            print("move", self.num, '2')
            sys.stdout.flush()
            self.get_away_from_crash_success = 1
            return
        elif gbmap.is_available(self.x, self.y + 1, self): # 如果右侧空闲，那就右移一步
            print("move", self.num, '0')
            sys.stdout.flush()
            self.get_away_from_crash_success = 1
            return
        elif gbmap.is_available(self.x + 1, self.y, self): # 如果下面空闲，那就下移一步
            print("move", self.num, '3')
            sys.stdout.flush()
            self.get_away_from_crash_success = 1
            return
        else: # 都不空闲
            self.get_away_from_crash_success = 0
            return # 不动
    
    def bypass_crash(self, gbmap, planner):
        '''如果下一个点是挂掉的机器人
        停止
        下下个点是挂掉的机器人吗
        下下下个点是挂掉的机器人吗
        直到找到可以走的点 记为中间点
        A*查找当前点到中间点的路径，将这一段路径插入到self.path里面
        '''

        if self.get_next_coordinate() in gbmap.crash_robot_coord:
            # print_log = open("./printlog.txt",'a')
            # print("id:",now_frame_id,file = print_log)
            # print("robot",self.num,file = print_log)
            # print("bypass_crash",file = print_log)
            # print(" ",file = print_log)
            # print_log.close()

            for i in range(2,10):
                if self.get_next_coordinate(num = i) in gbmap.crash_robot_coord: # 当前机器人走i步后装上正在恢复状态的机器人
                    continue
                else:
                    inter_point = self.get_next_coordinate(num= i)

                    tmp_map_info = gbmap.map_info
                    for coord in gbmap.crash_robot_coord:
                        tmp_map_info[coord[0]][coord[1]] = gbmap.OBS
                    
                    tmp_planner = Path_Planner(tmp_map_info)
                    inter_path_list = tmp_planner.astar((self.x, self.y), inter_point)

                    inter_path_list = inter_path_list[1:-1] # ind + 1
                    ind1 = self.path_list.index((self.x,self.y))
                    ind2 = self.path_list.index(inter_point)

                    self.path_list = self.path_list[:ind1+1] + self.path_list[ind2:]
                    self.path_list[ind1+1:ind1+1] = inter_path_list
                    return

    def check_collision(self, robot_list):
        """检查机器人是否容易发生碰撞，即和另一个机器人的曼哈顿距离小于等于2，则认为容易碰撞，编号小的机器人停车等待/后退避让
        想办法拉开机器人之间的距离
        """
        for i in range(self.num+1, robot_num):
            if abs(robot_list[i].x - self.x) + abs(robot_list[i].y - self.y) == 2:
                if self.get_next_coordinate() == robot_list[i].get_next_coordinate(): # 如果即将发生碰撞
                    if robot_list[i].x == self.x: # 如果是垂直对撞情况
                        self.move_back_along_path()
                        return 2 # 先停止，转换成距离为1的情况，然后水平方向上等待
                    elif robot_list[i].y == self.y: # 如果是水平对撞情况
                        self.move_back_along_path()
                        return 3 # 先停止，转换成距离为1的情况，然后垂直方向上等待
                    else: # 垂直碰撞情况
                        return 1 # 等待即可
                    
                elif self.get_next_coordinate(num = 2) == (robot_list[i].x,robot_list[i].y): # 处在self追踪robot_list[i]运行的状态
                    return 1 # 让self等待
                
                elif robot_list[i].get_next_coordinate(num = 2) == (self.x, self.y): # 处在robot_list[i]追踪self运行的状态
                    if gbmap.is_available(self.x + 1, self.y, self) and gbmap.is_available(self.x - 1, self.y, self) and gbmap.is_available(self.x, self.y + 1, self) and gbmap.is_available(self.x, self.y - 1, self): # 处在空旷环境
                        return 1 # 让self等待，准备让robot_list[i]超车
                
                else: # 不是对撞情况，不会相撞
                    pass
            elif abs(robot_list[i].x - self.x) + abs(robot_list[i].y - self.y) == 1:
                if self.get_next_coordinate() == (robot_list[i].x,robot_list[i].y) and robot_list[i].get_next_coordinate() == (self.x, self.y): # 如果即将发生对撞
                    if robot_list[i].x == self.x: # 如果是水平对撞情况

                        if gbmap.is_available(self.x + 1, self.y, self): # 检查下侧是否有空间避让
                            if (self.x + 1, self.y) in self.path_list: # 下侧点在路径中，直接回退到这一步
                                print("move", self.num, '3')
                                sys.stdout.flush()
                                self.x = self.x + 1
                                return 4
                            else: # 下侧点不在路径中，需要将该点纳入路径中
                                print("move", self.num, '3')
                                sys.stdout.flush()
                                ind = self.path_list.index((self.x,self.y))
                                self.path_list.insert(ind,(self.x + 1,self.y))
                                self.x = self.x + 1
                                return 4
                        elif gbmap.is_available(self.x - 1, self.y, self): # 检查上侧是否有空间避让
                            if (self.x - 1, self.y) in self.path_list: # 上侧点在路径中，直接回退到这一步
                                print("move", self.num, '2')
                                sys.stdout.flush()
                                self.x = self.x - 1
                                return 4
                            else:
                                print("move", self.num, '2')
                                sys.stdout.flush()
                                ind = self.path_list.index((self.x,self.y))
                                self.path_list.insert(ind,(self.x - 1,self.y))
                                self.x = self.x - 1
                                return 4
                        else: # 无法避让，沿着路径倒车
                            self.move_back_along_path()
                            return 4

                    elif robot_list[i].y == self.y: # 如果是垂直对撞情况

                        if gbmap.is_available(self.x, self.y + 1, self): # 检查右边是否有空间避让
                            if (self.x, self.y + 1) in self.path_list:
                                print("move", self.num, '0')
                                sys.stdout.flush()
                                self.y = self.y + 1
                                return 5
                            else:
                                print("move", self.num, '0')
                                sys.stdout.flush()
                                ind = self.path_list.index((self.x,self.y))
                                self.path_list.insert(ind,(self.x,self.y + 1))
                                self.y = self.y + 1
                                return 5
                        elif gbmap.is_available(self.x, self.y - 1, self): # 检查左边是否有空间避让
                            if (self.x, self.y - 1) in self.path_list:
                                print("move", self.num, '1')
                                sys.stdout.flush()
                                self.y = self.y - 1
                                return 5
                            else:
                                print("move", self.num, '1')
                                sys.stdout.flush()
                                ind = self.path_list.index((self.x,self.y))
                                self.path_list.insert(ind,(self.x,self.y - 1))
                                self.y = self.y - 1
                                return 5
                        else: # 无法避让，沿着路径倒车
                            self.move_back_along_path()
                            return 5

                
                    else: # 不是对撞情况，等待即可
                        return 1
                    
                elif self.get_next_coordinate() == (robot_list[i].x,robot_list[i].y): # 处在self追踪robot_list[i]运行的状态
                    return 1 # 让self等待
                
                elif robot_list[i].get_next_coordinate() == (self.x, self.y): # 处在robot_list[i]追踪self运行的状态
                    # 让robot_list[i]超车
                    if robot_list[i].x == self.x: # 如果是水平追踪情况

                        if gbmap.is_available(self.x + 1, self.y, self): # 检查下侧是否有空间避让
                            if (self.x + 1, self.y) in self.path_list: # 下侧点在路径中，直接回退到这一步
                                print("move", self.num, '3')
                                sys.stdout.flush()
                                self.x = self.x + 1
                                return 4
                            else: # 下侧点不在路径中，需要将该点纳入路径中
                                print("move", self.num, '3')
                                sys.stdout.flush()
                                if (self.x,self.y) in self.path_list:
                                    ind = self.path_list.index((self.x,self.y))
                                    self.path_list.insert(ind,(self.x + 1,self.y))
                                self.x = self.x + 1
                                return 4
                        elif gbmap.is_available(self.x - 1, self.y, self): # 检查上侧是否有空间避让
                            if (self.x - 1, self.y) in self.path_list: # 上侧点在路径中，直接回退到这一步
                                print("move", self.num, '2')
                                sys.stdout.flush()
                                self.x = self.x - 1
                                return 4
                            else:
                                print("move", self.num, '2')
                                sys.stdout.flush()
                                if (self.x,self.y) in self.path_list:
                                    ind = self.path_list.index((self.x,self.y))
                                    self.path_list.insert(ind,(self.x - 1,self.y))
                                self.x = self.x - 1
                                return 4
                        else: # 无法避让，沿着路径倒车
                            pass # 继续运行，等待机会

                    elif robot_list[i].y == self.y: # 如果是垂直追踪情况

                        if gbmap.is_available(self.x, self.y + 1, self): # 检查右边是否有空间避让
                            if (self.x, self.y + 1) in self.path_list:
                                print("move", self.num, '0')
                                sys.stdout.flush()
                                self.y = self.y + 1
                                return 5
                            else:
                                print("move", self.num, '0')
                                sys.stdout.flush()
                                if (self.x,self.y) in self.path_list:
                                    ind = self.path_list.index((self.x,self.y))
                                    self.path_list.insert(ind,(self.x,self.y + 1))
                                self.y = self.y + 1
                                return 5
                        elif gbmap.is_available(self.x, self.y - 1, self): # 检查左边是否有空间避让
                            if (self.x, self.y - 1) in self.path_list:
                                print("move", self.num, '1')
                                sys.stdout.flush()
                                self.y = self.y - 1
                                return 5
                            else:
                                print("move", self.num, '1')
                                sys.stdout.flush()
                                if (self.x,self.y) in self.path_list:
                                    ind = self.path_list.index((self.x,self.y))
                                    self.path_list.insert(ind,(self.x,self.y - 1))
                                self.y = self.y - 1
                                return 5
                        else: # 无法避让，沿着路径倒车
                            pass # 继续运行等待机会
                
                else: # 不是对撞情况，不会相撞
                    # pass
                    return 1
            # elif robot_list[i].x == self.x and robot_list[i].y == self.y: # 如果已经相撞 碰撞状态是两个机器人处在同一个坐标吗？看看任务书的说明
            #     # 随机脱困，随机向上/下/左/右移动一步
            #     print("move", self.num, np.random.randint(4))
            #     sys.stdout.flush()
            #     return 6
                
        return 0
    
    def get_next_coordinate(self, num = 1):
        '''查询当前点的下Num个点是什么'''
        if (self.x,self.y) in self.path_list:
            ind = self.path_list.index((self.x,self.y))

            if ind+num > len(self.path_list)-1:
                return (self.x,self.y)
            else:
                return (self.path_list[ind+num][0],self.path_list[ind+num][1])
        else:
            return (self.x,self.y)


    def move_back_along_path(self):
        '''沿着路径倒车'''
        back_path_list = self.path_list[::-1] # 路径翻转

        if (self.x, self.y) in back_path_list:
            ind = back_path_list.index((self.x,self.y))
            # print_log = open("./printlog.txt",'a')
            # print("ind",ind,file = print_log)
            # # print(back_path_list,file = print_log)
            # print_log.close()

            
            if len(back_path_list)-1 == ind: # 如果已经走到终点
                back_path_list = []
                return 0
            elif self.x == back_path_list[ind+1][0] and self.y - back_path_list[ind+1][1] == 1: # 左移
                print("move", self.num, '1')
                sys.stdout.flush()
                return len(back_path_list)
            elif self.x == back_path_list[ind+1][0] and self.y - back_path_list[ind+1][1] == -1: # 右移
                print("move", self.num, '0')
                sys.stdout.flush()
                back_path_list.pop(0)
                return len(back_path_list)
            elif self.x - back_path_list[ind+1][0] == 1 and self.y == back_path_list[ind+1][1]: # 上移
                print("move", self.num, '2')
                sys.stdout.flush()
                return len(back_path_list)
            elif self.x - back_path_list[ind+1][0] == -1 and self.y == back_path_list[ind+1][1]: # 下移
                print("move", self.num, '3')
                sys.stdout.flush()
                return len(back_path_list)
            # else:
            #     print_log = open("./printlog.txt",'a')
            #     print("re-plan",file = print_log)
            #     print_log.close()
            
        else:
            # 随机脱困，随机向上/下/左/右移动一步
            print("move", self.num, np.random.randint(4))
            sys.stdout.flush()
            return 0


robot = [Robot(num = i) for i in range(robot_num + 10)]

class Berth:
    def __init__(self, num = 0, x=0, y=0, transport_time=0, loading_speed=0):
        self.num = num
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed

        self.frame_id = 0 # 当前帧id
        self.now_gds_num = 0 # 当前泊位上等待装载的货物数量

        self.park_coordinates = [] # 可供机器人停靠的坐标

        self.isworking = False # 泊位是否正在工作

    def get_park_coordinates(self):
        # 获取泊位上的停靠坐标
        # self.park_coordinates.append((self.x, self.y))
        # self.park_coordinates.append((self.x + 3, self.y + 3))
        # self.park_coordinates.append((self.x , self.y + 3))
        # self.park_coordinates.append((self.x + 3, self.y))
        # self.park_coordinates.append((self.x + 2, self.y + 2))

        self.park_coordinates.append((self.x, self.y))
        self.park_coordinates.append((self.x + 1, self.y + 3))
        self.park_coordinates.append((self.x + 3, self.y + 1))

    def get_eta_time(self):
        # 获取当前泊位还需要多少帧才能将岸上的货物装载完毕
        return int(self.now_gds_num / self.loading_speed) + 1 # 向上取整
    
    def add_gds_num(self, num):
        # 机器人放下货物后，泊位上的货物数量增加
        self.now_gds_num += num

    def update_gds_num(self, now_frame_id):
        # 随着船只的装载，泊位上的货物数量减少
        self.update_working_status() # 更新泊位的工作状态
        if self.isworking: # 只有当泊位正在工作时，才会减少货物数量
            self.now_gds_num = self.now_gds_num - (now_frame_id - self.frame_id) * self.loading_speed
            if self.now_gds_num < 0:
                self.now_gds_num = 0

    def update_working_status(self):
        # 当泊位上有货物，且停靠着未装满的船只时，isworking为True
        if self.now_gds_num > 0:
            for i in range(boat_num):
                if boat[i].pos == self.num and boat[i].status != 2 and boat[i].get_etl_time(id) > 0:
                    self.isworking = True
                    return
            self.isworking = False
        else:
            self.isworking = False


# berth = [Berth() for _ in range(berth_num + 10)]
berth = [Berth(num = i) for i in range(berth_num)]

class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status
        self.last_status = status # 上一帧的状态
        self.begin_sail_frame_id = 0 # 开始航行的帧id
        self.begin_load_frame_id = 0 # 开始装载货物的帧id
        # self.destination = 0 # 0表示目的地是虚拟点，1表示目的地是泊位

        self.start_id = 0 #开始装载货物的帧id，辅助任务分配
        self.leaving_time = 0 #即将离去的时间，动态变化
    def go(self):
        # 船只从泊位驶出至虚拟点运输货物
        print("go", self.num)
        sys.stdout.flush() # 大多数语言会默认对输出数据做缓冲，因此在输出完一帧后应该主动flush标准输出
        # self.destination = 0

    def ship(self, berth_id, now_frame_id):
        # 船只移动到泊位id
        print("ship", self.num, berth_id)
        sys.stdout.flush()
        self.begin_sail_frame_id = now_frame_id
        # self.destination = 1

    def get_etl_time(self, now_frame_id):
        # 还需要多长时间才能装满货物
        if self.pos != -1 and self.status == 1:
            loaded_gds = (now_frame_id - self.begin_load_frame_id) * berth[self.pos].loading_speed # 已经装了多少货物
            remain_gds = boat_capacity - loaded_gds # 还剩多少货物
            return int(remain_gds / berth[self.pos].loading_speed) + 1 # 向上取整
    
    def get_now_capacity(self, now_frame_id):

        # print_log = open("./printlog.txt",'a')
        # print("boat_capacity",file = print_log)
        # print(boat_capacity,file = print_log)
        # print_log.close()

        # 获取当前船只的装载量
        if self.pos != -1 and self.status == 1:
            loaded_gds = (now_frame_id - self.begin_load_frame_id) * berth[self.pos].loading_speed
            remain_gds = boat_capacity - loaded_gds # 还剩多少货物
            return remain_gds
        else:
            return 0
        
    def get_begin_load_frame_id(self, now_frame_id):
        # 判断从哪一帧开始装载货物
        # 条件：目标泊位非虚拟点，船只状态由0或2变为1
        if self.pos != -1 and self.status == 1 and self.last_status != 1:
            self.begin_load_frame_id = now_frame_id

    def get_eta_time(self, now_frame_id):
        # 还需要多长时间才能到达泊位
        if self.pos != -1 and self.status == 0:
            sp_time = now_frame_id - self.begin_sail_frame_id # 已经航行了多少帧
            remain_time = berth[self.pos].transport_time - sp_time # 还需要航行多少帧
            return remain_time


boat = [Boat(num = i) for i in range(boat_num)]

class Allocation_aid:   #内置各种数组及队列，辅助任务分配
    def __init__(self):
        self.berth_boat_stop = [] #储存港口有船或即将有船
        self.berth_used = []      #使用的泊位
        self.sum_value = 0       #统计总价值

allocation_aid = Allocation_aid()

def Init():
    for i in range(0, n):
        line = input()
        ch.append([c for c in line.split(sep=" ")])
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
        berth[id].get_park_coordinates()

    boat_capacity = int(input()) # 为啥读到的是0？
    okk = input()
    print("OK")
    sys.stdout.flush()
    return boat_capacity

def Input():
    id, money = map(int, input().split(" "))

    # # 删除超时的货物
    # gbmap.check_gds_dict(id) # 可以单独拿到一个update函数中

    # 将物品信息读入地图中
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        # gds[x][y] = val
        gbmap.add_gds_dict(x, y, val, id)

    # 更新泊位信息 # 可以单独拿到一个update函数中
    for i in range(berth_num):
        # 根据当前帧id，更新泊位上的货物数量
        # berth[i].update_gds_num(id)
        # 将当前帧id输入berth对象
        berth[i].frame_id = id

    
    for i in range(robot_num):
        robot[i].last_status = robot[i].status
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
        
    gbmap.update_robot_obs(robot)
    gbmap.update_crash_robot_coord(robot)

    # 将船只信息读入boat对象中
    for i in range(boat_num):
        boat[i].last_status = boat[i].status
        boat[i].status, boat[i].pos = map(int, input().split())
        boat[i].get_begin_load_frame_id(id) # 获取船只开始装载货物的帧id
    okk = input()
    return id, num


class path_process:
    def __init__(self, planner):
        self.planner = planner

    def path_thread(self, input_queue, output_queue):

        while True:
            try:
                start, end, robot_number = input_queue.get(timeout=0.01)
                # print("计划机器人的移动: ", start, " 到 ", end)

                if start == end :
                    path = []
                    output_queue.put(path) 
                    # end_event.set()
                else:
                    start_time = time.time()

                    path = self.planner.astar((start[0], start[1]),(end[0], end[1]))

                    end_time = time.time()
                    execution_time = end_time - start_time

                    # print_log = open("./printlog.txt",'a')
                    # print(f"path planning cost {execution_time} s",file = print_log)
                    # print_log.close()
                    output_queue.put((path, robot_number)) 
                    # end_event.set()
                # start_event.clear()

            except queue.Empty:
                pass
                # start_event.clear()


if __name__ == "__main__":
    # with open(r'./printlog.txt','a+',encoding='utf-8') as prtlog:
    #     prtlog.truncate(0)


    boat_capacity = Init()

    # time.sleep(10) # 调试用

    # string = str(ch[0])[1]
    # print(string)
    gbmap.init_map(ch)
    # gbmap.transform(berth) # 对地图进行改造

    planner = Path_Planner(gbmap.map_info)

    input_queue = multiprocessing.Queue() 
    output_queue = multiprocessing.Queue()  
    # end_event = multiprocessing.Event()
    Path = path_process(planner)
    lock = multiprocessing.Lock()  # 实例化 Lock 对象

    p = multiprocessing.Process(target=Path.path_thread, args=(input_queue, output_queue,))
    p.start() 


    for zhen in range(1, 15001):
        # start_t = time.clock()

        now_frame_id, gds_add_num = Input()
        start_zhen_time = time.perf_counter()

        # if now_frame_id <= 400: # 先让货物多生成一点
        #     print("OK")
        #     sys.stdout.flush()
        #     continue

        # now_frame_id -= 400

        # Update()
        

        # --------------正式代码----------------
        task_allocation_main(robot, gbmap, berth, boat, gds_add_num, now_frame_id, allocation_aid, boat_capacity)


        for i in range(robot_num):
            if robot[i].status == 1: # 如果机器人处于正常状态
                if robot[i].last_status == 0: # 刚刚恢复正常
                    robot[i].get_away_from_crash() # 脱困算法，向可行方向走一步
                elif robot[i].get_away_from_crash_success == 0: # 还没有脱困成功
                    robot[i].get_away_from_crash() # 脱困算法，向可行方向走一步
                else:
                    if robot[i].path_list != []: # 当机器人未到达目标点时，沿着路径行走

                        robot[i].bypass_crash(gbmap, planner) # 可能会有问题，可以注释掉
                        
                        if robot[i].check_collision(robot) == 0: # 如果不需要避让

                            if (robot[i].x,robot[i].y) in robot[i].path_list:
                                ind = robot[i].path_list.index((robot[i].x,robot[i].y))
                              
                                if len(robot[i].path_list)-1 == ind: # 如果已经走到终点
                                    robot[i].path_list = []
                                elif robot[i].x == robot[i].path_list[ind+1][0] and robot[i].y - robot[i].path_list[ind+1][1] == 1: # 左移
                                    print("move", robot[i].num, '1')
                                    sys.stdout.flush()
                                elif robot[i].x == robot[i].path_list[ind+1][0] and robot[i].y - robot[i].path_list[ind+1][1] == -1: # 右移
                                    print("move", robot[i].num, '0')
                                    sys.stdout.flush()
                                elif robot[i].x - robot[i].path_list[ind+1][0] == 1 and robot[i].y == robot[i].path_list[ind+1][1]: # 上移
                                    print("move", robot[i].num, '2')
                                    sys.stdout.flush()
                                elif robot[i].x - robot[i].path_list[ind+1][0] == -1 and robot[i].y == robot[i].path_list[ind+1][1]: # 下移
                                    print("move", robot[i].num, '3')
                                    sys.stdout.flush()
                                else: # 下一步不合法
                                    # if not input_queue.empty():
                                    #     pass
                                    # else:
                                    if input_queue.qsize() < 4:
                                        input_queue.put(((robot[i].x, robot[i].y), (robot[i].mbx, robot[i].mby), i))
                            
                            else:
                                # if not input_queue.empty():
                                #     pass
                                # else:
                                if input_queue.qsize() < 4:
                                    input_queue.put(((robot[i].x, robot[i].y), (robot[i].mbx, robot[i].mby), i))


                    elif (robot[i].mbx, robot[i].mby) != (robot[i].x, robot[i].y):
                        # if not input_queue.empty():
                        #     pass
                        # else:    
                        if input_queue.qsize() < 4:                        
                            input_queue.put(((robot[i].x, robot[i].y), (robot[i].mbx, robot[i].mby), i))
                        # start_event.set()
                        
                    else: # 有路径，但已经到达目标点
                        robot[i].path_list = []
                        pass

            elif robot[i].status == 0: # 如果机器人处于恢复状态
                pass
            
            #统一查找路径是否搜索完毕
            # lock.acquire() # 加锁
            # if end_event.is_set():
            try:
                path, robotnumber = output_queue.get_nowait()
                robot[robotnumber].path_list = path

                # end_event.clear()
            except Empty:
                # time.sleep(0.0001)
                pass


        # print_log = open("./printlog.txt",'a')
        # print("id:",now_frame_id,file = print_log)
        # for i in range(robot_num):
        #     print("robot",robot[i].num,file = print_log)
        #     print("goods:",robot[i].goods,file = print_log)
        #     print("status:",robot[i].status,file = print_log)
        #     print("(x,y):",(robot[i].x,robot[i].y),file = print_log)
        #     print("(mbx,mby):",(robot[i].mbx,robot[i].mby),file = print_log)
        #     print("path_list:",robot[i].path_list,file = print_log)
        #     print("allocation_status:",robot[i].allocation_status,file = print_log)
        #     print(" ",file = print_log)
        # print(" ",file = print_log)
        # print_log.close()

        # 充分利用时间
        end_zhen_time = time.perf_counter()
        execution_time = end_zhen_time - start_zhen_time
        if(execution_time < 0.01 and execution_time > 0):
            time.sleep(0.001)
            # 我不知道为什么只能sleep这么多，多一点一帧就比15ms长？？？？

        print("OK")
        sys.stdout.flush()

        # print_log = open("./printlog.txt",'a')
        # print("end of one frame\r\n",file = print_log)
        # print_log.close()

        # end_t = time.clock()
        # print("time:", end_t - start_t, "s")