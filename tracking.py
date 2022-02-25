import robomaster
from robomaster import robot, flight
import numpy as np
import cv2
import time

#初始化操作
tl_drone = robot.Drone()
tl_drone.initialize()
tl_flight = tl_drone.flight
tl_camera = tl_drone.camera
tl_battery = tl_drone.battery
battery = tl_battery.get_battery()
drone_version = tl_drone.get_sdk_version()
GREY_MIN=180    #灰度最小值，如果摄像头未识别出道路请更改此处数值
GREY_MAX=255    #灰度最大值，如果摄像头未识别出道路请更改此处数值
WHITE_LIMIT = 3     #连续白色索引阈值
FORWARD_SPEED = 16  #无人机的前进速度
#显示无人机SDK版本
print("Drone sdk version: {0}".format(drone_version))
#显示无人机当前电量
print("The Battery is: {0}".format(battery))

#控制无人机起飞/降落/开启摄像头等行为命令
def send_ctrl_cmd(cmd):
    tl_drone.action_dispatcher.send_action(flight.FlightAction(cmd))

#发送无人机运动命令，a控制横滚，b控制俯仰，c控制上升下落，d控制旋转
def send_rc_cmd(a, b, c, d):
    tl_flight.rc(a=a, b=b, c=c, d=d)

#限定阈值，避免参数过大导致无人机失控
def out_limit(val, min, max):
    if val > max:
        val = max
    elif val < min:
        val = min
    return val

#巡线函数，返回摄像头图像与道路中心点
def get_line_pos(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    #对图像进行灰度处理
    # 二值化操作，将灰度值在180~255（GREY_MIN~GREY_MAX）区间内的部分变为白色，其余变为黑色
    ret, img_binary = cv2.threshold(img_gray, GREY_MIN, GREY_MAX, cv2.THRESH_BINARY, dst=None)
    img_binary = cv2.erode(img_binary, None, iterations=1)      #腐蚀图像，使画面更平滑
    color = []
    color.append(img_binary[:, 60])     #记录图像第60列的灰度值
    result = []
    white_sum = np.sum(color[0] == 255)  # 记录白色像素点个数
    white_index = np.where(color[0] == 255) # 记录白色像素点的坐标
    temp_index = []
    real_index = []
    #将连续的白色像素点判定为道路
    for j in range(white_sum - 1):
        if white_index[0][j] + 3 > white_index[0][j + 1]:  # 此处连续白色索引阈值为3，只有连续的白色才能被认定为道路
            temp_index.append(white_index[0][j])
        else:
            if len(temp_index) > len(real_index):
                real_index = temp_index
                temp_index = []
    if len(temp_index) > len(real_index):
        real_index = temp_index
    real_sum = len(real_index)
    #只有第60列的白色像素点个数大于5个才认为存在道路，返回计算出的连续白色索引中心点
    if real_sum > 5:
        white_center = (real_index[real_sum - 1] + real_index[0]) / 2
        cv2.circle(img_binary, (60, int(white_center)), 1, (0, 0, 0), 3)    #在道路中心画点
        result.append([1, white_center-120])  # 左正右负
    else:
        result.append([0, 0])
    print(result)
    return result, img_binary


if __name__ == '__main__':
    # # 无人机起飞，起飞时只需要将此处代码解除注释即可
    # send_ctrl_cmd('takeoff')

    tl_camera.start_video_stream(display=False)
    send_ctrl_cmd('downvision 1')   #打开下方摄像头
    while True:
        key = cv2.waitKey(1) & 0xff
        # 按ESC无人机降落/结束运行
        if key == 27:
            send_rc_cmd(0, 0, 0, 0)
            send_ctrl_cmd('land')
            break
        img = tl_camera.read_cv2_image(strategy='newest')
        img = cv2.resize(img, (320, 240))
        t = time.time()
        ret, imgbinary = get_line_pos(img)

        cv2.imshow("BIN", imgbinary)
        cv2.waitKey(1)
        if ret[0][0] == 0:
            a_fly=0
            d_fly = 0
        else:
            if abs(ret[0][1])>15:                       #如果道路中心偏移过大，无人机主改变偏航角
                d_fly = -int(ret[0][1] * 1.8)
                d_fly = out_limit(d_fly, -90, 90)  # 偏航角
                a_fly = -int(ret[0][1] * 50 / 160)
                a_fly = out_limit(a_fly, -10, 10)  # 横滚
            else:                                       #如果道路中心偏移较小，无人机主改变横滚
                a_fly = -int(ret[0][1] * 100 / 160)
                a_fly = out_limit(a_fly, -15, 15)  # 横滚
                d_fly = -int(ret[0][1])
                d_fly = out_limit(d_fly, -45, 45)  # 偏航角
        tl_flight.rc(a_fly,FORWARD_SPEED, 0, d_fly)

        # #发送数据
        # print('_________________________')
        # print((16 -abs(ret[0][1])))
        # print('%d, %d' % (a_fly, d_fly))
        # print('_________________________')

    #关闭窗口，关闭无人机
    cv2.destroyAllWindows()
    tl_camera.stop_video_stream()
    tl_drone.close()