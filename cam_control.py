# opencv包
import cv2
import numpy as np
import time
# 串口包
import serial
# ROS部分功能包
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# 定义画面翻转函数
def totateAntiClockWise90ByNumpy(img_file):  # np.rot90(img, -1) 顺时针旋转90度   旋转程度：1,2,-1,-2
    img90 = np.rot90(img_file, -1)
    return img90


flage = None  # 控制标志位
flage_move = 1  # 控制转移标志位

counter = 0

kp_angle = 0.005  # 旋转pid参数设置
kp_v = 0.000006  # 速度pid参数
traget_s = 16000  # 目标大小设置
traget_s_arm = 20000
angle_x_lim = 0.05  # 旋转速度限幅
v_lim = 0.04  # 速度限幅
S_error = 5000  # 目标锁定允许误差范围
# 主函数
if __name__ == '__main__':
    # ROS通信部分
    rospy.init_node('camcontrol', anonymous=True)  # 初始化节点
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 发布者 对象 #发布给谁
    # pub2 = rospy.Publisher("control_flage_back", String, queue_size=10)
    # msg2 = String()  # 创建 msg 对象
    rate = rospy.Rate(10)  # 循环频率
    msg_v = Twist()  # 创建消息

    # 串口控制底层节点
    serial_port = serial.Serial(
        port='/dev/ttyUSB1',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    # opencv部分
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # 机械臂初始化程序
    # send_data = "{\"T\":12}"
    # serial_port.write(send_data.encode())
    # time.sleep(3)
    send_data = "{\"T\":2,\"P1\":277.5104065,\"P2\":-13.75,\"P3\":290.0,\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":200}"
    serial_port.write(send_data.encode())

    go_two_flag = False

    # 定义中心列表
    while True:

        if not go_two_flag:
            while 1:  # 一直检测接收控制标志位
                try:
                    msg = rospy.wait_for_message("/control_flage", String, timeout=None)
                    flage = msg.data
                    #if flage == '2':
                    if flage == '2' or flage == '3':
                        go_two_flag = True
                        break
                except:
                    pass
        ret, frame = cap.read()  # 视频流中读取一帧图像
        if not ret:  # 未读取到布尔值
            print("Can't receive frame")
            break
        # **********画面旋转********
        frame = totateAntiClockWise90ByNumpy(frame)
        # *******************
        framehsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 图像格式转换，frame转HSV
        # 阈值设置
        lower = np.array([8, 89, 143])
        upper = np.array([23, 254, 230])
        mask = cv2.inRange(framehsv, lower, upper)  # 利用阈值去除背景
        frameand = cv2.bitwise_and(frame, frame, mask=mask)  # 图像重合求与
        framegray = cv2.cvtColor(frameand, cv2.COLOR_BGR2GRAY)  # 图像格式转换，转换为灰度图
        # 查找轮廓
        contours, _ = cv2.findContours(framegray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = framegray.shape
        # 打印高和宽
        if len(contours) > 0:
            # 初始化最大周长和对应的轮廓
            max_perimeter = 0
            max_perimeter_contour = None
            # 寻找最大周长的轮廓
            for contour in contours:
                perimeter = cv2.arcLength(contour, False)
                if perimeter > max_perimeter:
                    max_perimeter = perimeter
                    max_perimeter_contour = contour
            if max_perimeter_contour is not None:
                x, y, w, h = cv2.boundingRect(max_perimeter_contour)
                cv2.rectangle(frameand, (x, y), (x + w, y + h), (0, 255, 0), 2)
                c_x, c_y = (int(x + w / 2) - 120, int(y + h / 2) - 120)  # x方向
                # 画圈
                # print(c_x,c_y)
                S = abs(w) * abs(h)

                angle_x = c_x * kp_angle
                v = (traget_s - S) * kp_v
                # print(traget_s - S)
                # print(v)
                # flage_move == 1
                if abs(c_x) > 10:
                    print("旋转定位")
                    if angle_x > angle_x_lim:
                        angle_x = angle_x_lim
                    if angle_x < -angle_x_lim:
                        angle_x = -angle_x_lim
                    msg_v.linear.x = 0
                    msg_v.angular.z = -angle_x
                    pub.publish(msg_v)
                else:
                    # print('大', traget_s - S > S_error)
                    # print('小', traget_s - S < -S_error)
                    msg_v.linear.x = 0
                    msg_v.angular.z = 0
                    pub.publish(msg_v)
                    # print(traget_s - S)
                    if traget_s - S > S_error:
                        if v > v_lim:  # 限制幅度
                            v = v_lim
                        msg_v.linear.x = v  # 前进
                        # print(msg_v.linear.x)
                        msg_v.angular.z = 0
                        pub.publish(msg_v)
                    elif traget_s - S < -S_error:
                        if v < -v_lim:  # 限制幅度
                            v = -v_lim
                        msg_v.linear.x = v  # 后退
                        msg_v.angular.z = 0
                        pub.publish(msg_v)
                    else:
                        counter = counter + 1  # 等待稳定
                        if counter == 100:
                            counter = 0  # 计数清除
                            print("控制权转移")
                            flage_move = 0  # 控制权转移标志位

                if flage_move == 0:  # 控制权交接机械臂
                    # print(666)
                    # if abs(traget_s_arm - S ) > 10:
                    hight = 300
                    # 机械臂调整类pid
                    c_x = c_x / 6
                    c_y = -c_y / 5 + hight
                    print("机械臂对准")
                    print(traget_s_arm - S)
                    # 限制幅度
                    if c_x > 50:
                        c_x = 50
                    if c_x < -50:
                        c_x = -50
                    if c_y > 50 + hight:
                        c_y = 50 + hight
                    if c_y < -50 + hight:
                        c_y = -50 + hight
                    #send_data = "{\"T\":2,\"P1\":277.5104065,\"P2\":" + str(c_x) + ",\"P3\":" + str(
                     #   c_y) + ",\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":100}"
                    send_data = "{\"T\":2,\"P1\":277.5104065,\"P2\":-13.75,\"P3\":" + str(
                        c_y) + ",\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":100}"
                    serial_port.write(send_data.encode())

                    if flage == '2':
                    # else: #执行夹取动作
                        counter = counter + 1
                        print(counter)
                        if abs(counter - 50):  # 拿取动作序列
                            print("夹取固定动作")
                            # 夹子打开
                            # 向前
                            send_data = "{\"T\":2,\"P1\":360,\"P2\":-13.75,\"P3\":" + str(
                                c_y) + ",\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":50}"
                            serial_port.write(send_data.encode())
                            time.sleep(3)
                            # 夹子夹取然后拿起
                            send_data = "{\"T\":2,\"P1\":360,\"P2\":-13.75,\"P3\":" + str(
                                c_y) + ",\"P4\":90,\"P5\":250,\"S1\":10,\"S5\":150}"
                            serial_port.write(send_data.encode())
                            time.sleep(4)
                            send_data = "{\"T\":2,\"P1\":360,\"P2\":-13.75,\"P3\":330 " \
                                                                                   ",\"P4\":90,\"P5\":250,\"S1\":10,\"S5\":150}"
                            serial_port.write(send_data.encode())
                            time.sleep(1)
                            # 向后
                            send_data = "{\"T\":2,\"P1\":277,\"P2\":-13.75,\"P3\":330" \
                                                                                   ",\"P4\":90,\"P5\":250,\"S1\":10,\"S5\":50}"
                            serial_port.write(send_data.encode())
                            time.sleep(3)
                            counter = 0  # 计数清除
                            flage_move = 1
                            go_two_flag = False
                            # 发布控制权利消息
                            # msg2.data = str(flage_move)
                            # pub2.publish(msg2)
                            # print("end")
                    elif flage == '3': #开门动作序列
                        # 夹子打开
                         # 向上向前
                        send_data = "{\"T\":2,\"P1\":316,\"P2\":-13.75,\"P3\":" + str(
                            c_y+60) + ",\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":50}"
                        serial_port.write(send_data.encode())
                        time.sleep(5)
                        # 夹子夹取
                        send_data = "{\"T\":2,\"P1\":316,\"P2\":-13.75,\"P3\":" + str(
                            c_y+60) + ",\"P4\":90,\"P5\":280,\"S1\":10,\"S5\":200}"
                        serial_port.write(send_data.encode())
                        time.sleep(8)
                        # 夹子向下带动门把手
                        send_data = "{\"T\":2,\"P1\":316,\"P2\":-13.75,\"P3\":" + str(
                            c_y-140) + ",\"P4\":90,\"P5\":280,\"S1\":50,\"S5\":150}"
                        serial_port.write(send_data.encode())
                        time.sleep(8)
                        # 向后把门拉开
                        send_data = "{\"T\":2,\"P1\":257,\"P2\":-13.75,\"P3\":" + str(
                            c_y-140) + ",\"P4\":90,\"P5\":280,\"S1\":10,\"S5\":150}"
                        serial_port.write(send_data.encode())
                        time.sleep(5)
                        # 夹子松开
                        send_data = "{\"T\":2,\"P1\":277,\"P2\":-13.75,\"P3\":" + str(
                            c_y-140) + ",\"P4\":90,\"P5\":210,\"S1\":10,\"S5\":150}"
                        serial_port.write(send_data.encode())
                        time.sleep(5)
                        
                        counter = 0  # 计数清除
                        flage_move = 1
                        go_two_flag = False
        # 画面展示
        cv2.imshow('b', frame)
        cv2.imshow('a', frameand)
        # 推出设置
        if cv2.waitKey(1) == ord('q'):
            break

