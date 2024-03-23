# import collections
import numpy as np
import time

def floodFill(map_info, start_x, start_y):
    # 已填充的单元格
    filled = set()
    # 待填充的单元格
    fill = set()
    fill.add((start_x, start_y))
    width = map_info.shape[1] - 1
    height = map_info.shape[0] - 1
    # 输出的淹没数组
    flood = np.zeros_like(map_info, dtype=np.int8)
    while fill:
        # 获取单元格
        x, y = fill.pop()
        if y > height or x > width or x < 0 or y < 0:
            # 不填充
            continue
        if map_info[x][y] != 1 and map_info[x][y] != 2:
            # 填充
            flood[x][y] = 1
            filled.add((x, y))
            # 检查相邻单元格
            west = (x - 1, y)
            east = (x + 1, y)
            north = (x, y - 1)
            south = (x, y + 1)
            if west not in filled:
                fill.add(west)
            if east not in filled:
                fill.add(east)
            if north not in filled:
                fill.add(north)
            if south not in filled:
                fill.add(south)
    return flood

# def update_reachable_region(robot_list, map_info):
#     '''更新机器人的可达区域'''

#     robot_list[0].reachable_region = floodFill(map_info, robot_list[0].x, robot_list[0].y)

#     for i in range(1,len(robot_list)):
#         flag = 0
#         for j in range(0,i):
#             if robot_list[j].reachable_region[robot_list[i].x][robot_list[i].y] == 1:
#                 robot_list[i].reachable_region = robot_list[j].reachable_region
#                 flag = 1
#                 break
#         if flag == 0:
#             robot_list[i].reachable_region = floodFill(map_info, robot_list[i].x, robot_list[i].y)

def update_reachable_region(robot_list, robot_start_coord, map_info):
    '''更新机器人的可达区域'''

    # print_log = open("./printlog.txt",'a')
    # print('robot_start_coord:',robot_start_coord,file = print_log)
    # print_log.close()

    robot_list[0].reachable_region = floodFill(map_info, robot_start_coord[0][0], robot_start_coord[0][1])

    for i in range(1,len(robot_start_coord)):
        flag = 0
        for j in range(0,i):
            if robot_list[j].reachable_region[robot_start_coord[i][0]][robot_start_coord[i][1]] == 1:
                robot_list[i].reachable_region = robot_list[j].reachable_region
                flag = 1
                break
        if flag == 0:
            robot_list[i].reachable_region = floodFill(map_info, robot_start_coord[i][0], robot_start_coord[i][1])

    # 调试用
    # for k in range(10):
    #     print_log = open("./printlog.txt",'a')
    #     for i in range(200):
    #         for j in range(200):
    #             print(robot_list[k].reachable_region[i][j],end=' ',file = print_log)
    #         print('\n',file = print_log)
    #     print('\n',file = print_log)
    #     print_log.close()
        
    


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
    file_path = r'D:\OneDrive - zju.edu.cn\HWCodeCraft\WindowsRelease\maps\map7.txt'
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



    start_time = time.time()

    # start_info_list = [(25,79),(25,119),(47,153),(69,79),(69,119),(130,79),(130,119),(152,44),(174,79),(174,119)]
    start_info_list = [(26,26),(29,63),(85,149),(90,32),(116,80),(118,29),(130,143),(136,72),(167,122),(167,173)]



    flood_list = []

    flood0 = floodFill(map_info, start_info_list[0][0], start_info_list[0][1])
    flood_list.append(flood0)

    for i in range(1,10):
        for j in range(0,i):
            if flood_list[j][start_info_list[i][0]][start_info_list[i][1]] == 1:
                flood_list.append(flood_list[j])
                break
        if len(flood_list) == i:
            flood_list.append(floodFill(map_info, start_info_list[i][0], start_info_list[i][1]))

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"洪水填充算法用时为: {execution_time} 秒")

    for k in range(10):
        print_log = open("./printlog.txt",'a')
        for i in range(200):
            for j in range(200):
                print(int(flood_list[k][i][j]),end=' ',file = print_log)
            print('\n',file = print_log)
        print('\n',file = print_log)
        print_log.close()




    # 定义起点和终点
    # start_info = (25,79)  #定义info信息为15

    # start_time = time.time()
    # map1 = floodFill(map_info, start_info[0], start_info[1])

    # end_time = time.time()
    # execution_time = end_time - start_time
    # print(f"洪水填充算法用时为: {execution_time} 秒")



    # print_log = open("./printlog.txt",'a')
    # for i in range(200):
    #     for j in range(200):
    #         print(matrix[i][j],end=' ',file = print_log)
    #     print('\n',file = print_log)
    # print('\n',file = print_log)
    # print_log.close()


    # print_log = open("./printlog.txt",'a')
    # for i in range(200):
    #     for j in range(200):
    #         print(int(map_info[i][j]),end=' ',file = print_log)
    #     print('\n',file = print_log)
    # print('\n',file = print_log)
    # print_log.close()


    # print_log = open("./printlog.txt",'a')
    # for i in range(200):
    #     for j in range(200):
    #         print(int(map1[i][j]),end=' ',file = print_log)
    #     print('\n',file = print_log)
    # print('\n',file = print_log)
    # print_log.close()





