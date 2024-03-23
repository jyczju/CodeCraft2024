# @File: map_info_queue_bfs

import numpy as np
from collections import deque

import time

import copy


def berth_bfs(x1, y1, map_info1):
    start_time = time.time()


    map_info = copy.deepcopy(map_info1)
    i_dict = {}

    dirs = [
    lambda x, y: (x, y + 1),  # 右
    lambda x, y: (x + 1, y),  # 下
    lambda x, y: (x, y - 1),  # 左
    lambda x, y: (x - 1, y),  # 上
    ]

    q = deque()
    path = []
    q.append((x1, y1, -1))
    map_info[x1][y1] = 2
    while len(q) > 0:
        cur_node = q.popleft()
        path.append(cur_node)
        
        # 计算当前点的路径
        i = len(path) - 1
        i_dict[cur_node[:2]] = i

        for d in dirs:
            next_x, next_y = d(cur_node[0], cur_node[1])
            if 0 <= next_x < 200 and 0 <= next_y < 200 and map_info[next_x][next_y] != 1 and map_info[next_x][next_y] != 2:
                q.append((next_x, next_y, len(path) - 1))  # path列表n-1位置的点是它的父亲
                map_info[next_x][next_y] = 1
    # print('无路可走')
                
    end_time = time.time()
                
    # print_log = open("./printlog.txt",'a')
    # print("berth_bfs cost:",end_time - start_time, file = print_log)
    # print_log.close()
    return i_dict, path

def get_path(iter, path):
    realpath = []
    i = iter
    while i >= 0:
        realpath.append(path[i][:2])
        i = path[i][2]
    realpath.reverse()

    # print_log = open("./printlog.txt",'a')
    # print('realpath',realpath,file = print_log)
    # print_log.close()

    return realpath



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
    file_path = r'D:\OneDrive - zju.edu.cn\HWCodeCraft\WindowsRelease\maps\map8.txt'
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


    map_info_copy = map_info.copy()

    import time
    start_time = time.time()


    i_dict, path_list = berth_bfs(31, 68, map_info)

    # print(i_dict)
    # print(path_list)



    # 使用A*算法寻找最短路径
    # 打印最短路径

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"New BFS路径树构建用时为: {execution_time} 秒")

    start_time = time.time()

    realpath = get_path(i_dict[(156,156)], path_list)

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"New BFS路径查询用时为: {execution_time} 秒")

    # print(realpath)

    # print(i_dict)
    # # print(path)

    print_log = open("./printlog.txt",'a')
    for i in range(200):
        for j in range(200):
            if (i,j) in i_dict:
            # if (i,j) in realpath:
                print("*",end=' ',file = print_log)
            else:
                print(int(map_info_copy[i][j]),end=' ',file = print_log)
        print('\n',file = print_log)
    print('\n',file = print_log)
    print_log.close()
