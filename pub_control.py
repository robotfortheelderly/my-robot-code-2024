# 接收语音控制，作为发布者
import rospy # 导入ROS用于Python的库：rospy
from geometry_msgs.msg import Twist # 导入geometry_msgs包中的Twist消息类型，用于控制机器人移动
from std_msgs.msg import String#
import serial
import time

serial_port = serial.Serial(
    #port='/dev/voice_usb',
    port='/dev/ttyUSB0',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
serial_port_arm = serial.Serial(
    #port='/dev/voice_usb',
    port='/dev/ttyUSB1',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
if __name__ == '__main__':
    rospy.init_node('Voicecontrol', anonymous=True)  # 初始化节点
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 发布者 对象 #发布给谁
    pub1 = rospy.Publisher("control_flage", String, queue_size=10)
    # pub = rospy.Publisher('movedata', Twist,queue_size=10)#发布者 对象 #发布给谁
    # 小车base_control
    flage = 0
    flage_back = 0
    counter = 0
    flage_tow_now = True
    msg1 = String()  #创建 msg 对象
    rate = rospy.Rate(50)  # 循环频率
    msg = Twist()  # 创建消息
    while not rospy.is_shutdown():  # 运行时候
        if serial_port.inWaiting() > 0:
            data = serial_port.read_all()
            print(data)
            data_de = data.decode()
            if data_de == '50':
                flage = 1
            elif data_de == '100':
                flage = 2
            elif data_de == '200':  
                flage = 3
                if flage_back == 2:
                    flage = 1
            if flage == 1:  # 语音接入控制
                if data_de == '1':  # 前
                    number = 5000
                    for i in range(number):
                        msg.linear.x = 0.15
                        msg.angular.z = 0
                        pub.publish(msg)
                if data_de == '2':  # 后
                    number = 5000
                    for i in range(number):
                        msg.linear.x = -0.15
                        msg.angular.z = 0
                        pub.publish(msg)
                if data_de == '3':  # 左转
                    number = 20000
                    for i in range(number):
                        msg.linear.x = 0
                        msg.angular.z = 0.5
                        pub.publish(msg)
                if data_de == '4':  # 右转
                    number = 20000
                    for i in range(number):
                        msg.linear.x = 0
                        msg.angular.z = -0.5
                        pub.publish(msg)
                if data_de == '5':  # 放下
                    # 夹子打开
                    send_data = "{\"T\":2,\"P1\":277,\"P2\":-13.5,\"P3\":330,\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":150}"
                    serial_port_arm.write(send_data.encode())
                    time.sleep(5)
                    send_data = "{\"T\":2,\"P1\":277.5104065,\"P2\":-13.75,\"P3\":290.0,\"P4\":90,\"P5\":180,\"S1\":10,\"S5\":150}"
                    serial_port_arm.write(send_data.encode())
                    time.sleep(1)
                if data_de == '6':  # 扶起
                    # 机械臂到低位,合手
                    send_data = "{\"T\":2,\"P1\":277,\"P2\":-13.5,\"P3\":180,\"P4\":90,\"P5\":260,\"S1\":10,\"S5\":250}"
                    serial_port_arm.write(send_data.encode())
                    time.sleep(6)
                    # 扶起
                    send_data = "{\"T\":2,\"P1\":277,\"P2\":-13.5,\"P3\":330.0,\"P4\":90,\"P5\":260,\"S1\":50,\"S5\":150}"
                    serial_port_arm.write(send_data.encode())
                    time.sleep(8)

        #msg2 = rospy.wait_for_message("/control_flage_back", String, timeout=1.0)
        #flage_back = msg2.data
        #print(flage_back)
        #发布控制权利消息
        if flage == 1:
            msg1.data = str(flage)
            pub1.publish(msg1)
        elif flage == 2 or flage == 3:
            for i in range(20000):
                msg1.data = str(flage)
                pub1.publish(msg1)
              #  print("这是flage的值：",flage)
            flage = 1
            flage_tow_now = True




