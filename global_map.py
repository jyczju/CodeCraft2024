import numpy as np


class Map():
    def __init__(self):
        self.width = 200
        self.height = 200
        self.robot_num = 10
        self.boat_num = 5
        self.berth_num = 10
        self.map_info = np.zeros((self.width, self.height))

        self.gds_dict = {} # 用字典维护地图上的货物信息，键为坐标，值为货物价值和生成时的帧号(x, y):(value, frame_id)

        self.robot_obs = [] # 将机器人所在位置及其下一步视为障碍物（仅在isavailable函数中这样考虑） (x,y)
        self.robot_obs_org = []

        self.crash_robot_coord = [] # 将处于恢复状态的机器人坐标存起来

        self.robot_start_coord = []

        self.SPACE = 0 # 空地用0表示
        self.OCEAN = 1 # 海洋用1表示
        self.OBS = 2 # 墙用2表示
        self.ROBOT = 3 # 机器人用3表示
        self.BERTH = 4 # 泊位用4表示

        # self.init_map(ch)

    def init_map(self, ch):
        for i in range(self.height):
            for j in range(self.width):

                if str(ch[i])[j+2] == '.': # j为什么要加2？因为ch[i]的前两个字符是[和'，所以要加2
                    self.map_info[i][j] = self.SPACE
                elif str(ch[i])[j+2] == '*':
                    self.map_info[i][j] = self.OCEAN
                elif str(ch[i])[j+2] == '#':
                    self.map_info[i][j] = self.OBS
                elif str(ch[i])[j+2] == 'A':
                    self.map_info[i][j] = self.SPACE # 机器人的位置也是空地
                    self.robot_start_coord.append((i,j))
                elif str(ch[i])[j+2] == 'B':
                    self.map_info[i][j] = self.BERTH
        
        # print_log = open("./printlog.txt",'a')
        # print("map_info",file = print_log)
        # for i in range(self.height):
        #     for j in range(self.width):
        #         print(int(self.map_info[i][j]),end=' ',file = print_log)
        #     print('\n',file = print_log)
        # print_log.close()

    def transform(self):
        '''对地图进行改造，将小于等于两个可行方向的点变为障碍物'''
        orig_map_info = self.map_info.copy()
        for i in range(self.height):
            for j in range(self.width):
                # 如果这个点小于等于两个可行方向
                if orig_map_info[i][j] == self.SPACE:
                    sum_dir = 0
                    # 判断点(i+1,j)是否可行
                    if 0 <= i+1 < self.height and 0 <= j < self.width and orig_map_info[i+1][j] == self.SPACE:
                        sum_dir += 1
                    # 判断点(i-1,j)是否可行
                    if 0 <= i-1 < self.height and 0 <= j < self.width and orig_map_info[i-1][j] == self.SPACE:
                        sum_dir += 1
                    # 判断点(i,j+1)是否可行
                    if 0 <= i < self.height and 0 <= j+1 < self.width and orig_map_info[i][j+1] == self.SPACE:
                        sum_dir += 1
                    # 判断点(i,j-1)是否可行
                    if 0 <= i < self.height and 0 <= j-1 < self.width and orig_map_info[i][j-1] == self.SPACE:
                        sum_dir += 1

                    if sum_dir <= 2:
                        self.map_info[i][j] = self.OBS

        # arr = np.ones((self.height,1))
        # self.map_info[:,int(self.height/2-1)]=arr[:,0] # 将中间列变为障碍物

        
        # print_log = open("./printlog.txt",'a')
        # print("map_info",file = print_log)
        # for i in range(self.height):
        #     for j in range(self.width):
        #         print(int(self.map_info[i][j]),end=' ',file = print_log)
        #     print('\n',file = print_log)
        # print_log.close()
                
    def update_robot_obs(self,robot_list):
        self.robot_obs = []
        for robot in robot_list:
            self.robot_obs.append((robot.x,robot.y))
            if robot.status == 1: # 机器人正常运行状态下
                tmp_coor = robot.get_next_coordinate()
                if tmp_coor not in self.robot_obs:
                    self.robot_obs.append(tmp_coor)
        self.robot_obs_org = self.robot_obs

    def update_crash_robot_coord(self,robot_list):
        self.crash_robot_coord = []
        for robot in robot_list:
            if robot.status == 0 :
                self.crash_robot_coord.append((robot.x,robot.y))

                    
    def is_available(self, x, y, robot_self):
        # 判断坐标(x,y)能否被机器人进入
        if (robot_self.x,robot_self.y) in self.robot_obs:
            self.robot_obs.remove((robot_self.x,robot_self.y))
        tmp_coor = robot_self.get_next_coordinate()
        if tmp_coor in self.robot_obs:
            self.robot_obs.remove(tmp_coor)
        
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            # if (robot_self.x,robot_self.y) not in self.robot_obs:
            #     self.robot_obs.append((robot_self.x,robot_self.y))
            # if tmp_coor not in self.robot_obs:
            #     self.robot_obs.append(tmp_coor)
            self.robot_obs = self.robot_obs_org
            return False
        elif self.map_info[x][y] == self.OCEAN or self.map_info[x][y] == self.OBS or (x,y) in self.robot_obs:
            # if (robot_self.x,robot_self.y) not in self.robot_obs:
            #     self.robot_obs.append((robot_self.x,robot_self.y))
            # if tmp_coor not in self.robot_obs:
            #     self.robot_obs.append(tmp_coor)
            self.robot_obs = self.robot_obs_org
            return False
        
        # if (robot_self.x,robot_self.y) not in self.robot_obs:
        #     self.robot_obs.append((robot_self.x,robot_self.y))
        # if tmp_coor not in self.robot_obs:
        #     self.robot_obs.append(tmp_coor)
        self.robot_obs = self.robot_obs_org
        return True
    
    # def is_available(self, x, y):
    #     # 判断坐标(x,y)能否被机器人进入
    #     # 关于两个机器人是否会碰撞，可以在Robot类中判断
    #     if x < 0 or x >= self.width or y < 0 or y >= self.height:
    #         return False
    #     elif self.map_info[x][y] == self.OCEAN or self.map_info[x][y] == self.OBS:
    #         return False
    #     return True
    
    def add_gds_dict(self, x, y, value, frame_id):
        self.gds_dict.update({(x, y):(value, frame_id)})

    def delete_gds_dict(self, x, y):
        if (x, y) in self.gds_dict: #删除之前先判断是否被删除过了
            del self.gds_dict[(x, y)]
        # pass # 暂时不实现，机器人取走货物时，将货物从字典中删除

    def check_gds_dict(self, now_frame_id):
        # 超时则删除对应的货物
        del_k = []
        for k, v in self.gds_dict.items():
            if now_frame_id - v[1] > 1000:
                del_k.append(k)

        if del_k != []:
            for k in del_k:
                if k in self.gds_dict:  #删之前判断是否被删除过了
                    del self.gds_dict[k]
            return 1
        else:
            return 0

    def check_gds_time(self, now_frame_id): #计算货物剩多少时间消失
        time_gds = []
        for k, v in self.gds_dict.items():
            time_remain = now_frame_id - v[1]
            if time_remain >= 1000:
                time_gds.append(0)
            elif time_remain < 1000:
                time_gds.append(time_remain)
            else:
                time_gds.append(-1)
        return time_gds
    
if __name__ == "__main__":
    # 定义迷宫
    def read_map_txt_file(file_path):
            # 打开文件并读取内容
            with open(file_path, 'r') as file:
                lines = file.readlines()

            # 初始化二维数组
            matrix = []

            # 处理每一行数据
            for line in lines:
                # 去除行末的换行符并按字符分割数据
                data = list(line.strip())
                
                # 将数据添加到二维数组中
                matrix.append(data)

            return matrix
    
    # file_path = r'D:\OneDrive - zju.edu.cn\HWCodeCraft\WindowsRelease\maps\map6.txt'
    file_path = r'D:\OneDrive - zju.edu.cn\HWCodeCraft\WindowsRelease\maps\map3.txt'
    matrix = read_map_txt_file(file_path)

    map_info = np.zeros((200,200))

    for i in range(len(matrix)):
        for j in range(len(matrix[0])):

            if matrix[i][j] == '.':
                map_info[i][j] = 0
            elif matrix[i][j] == '*':
                map_info[i][j] = 1
            elif matrix[i][j] == '#':
                map_info[i][j] = 2
            elif matrix[i][j] == 'A':
                map_info[i][j] = 0 # 机器人的位置也是空地
            elif matrix[i][j] == 'B':
                map_info[i][j] = 4

    gbmap = Map()

    gbmap.map_info = map_info

    with open(r'./printlog.txt','a+',encoding='utf-8') as prtlog:
            prtlog.truncate(0)

    print_log = open("./printlog.txt",'a')
    for i in range(200):
        for j in range(200):
            print(int(map_info[i][j]),end=' ',file = print_log)
        print('\n',file = print_log)
    print('\n',file = print_log)
    print_log.close()

    import time

    start_time = time.time()

    gbmap.transform()

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"地图预处理用时为: {execution_time} 秒")

    print_log = open("./printlog.txt",'a')
    for i in range(200):
        for j in range(200):
            print(int(gbmap.map_info[i][j]),end=' ',file = print_log)
        print('\n',file = print_log)
    print('\n',file = print_log)
    print_log.close()