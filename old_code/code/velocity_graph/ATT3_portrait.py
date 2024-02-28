from SerialHandler import SerialHandler
from XbusPacket import XbusPacket
from DataPacketParser import DataPacketParser, XsDataPacket
from SetOutput import set_output_conf
import time
import threading
import keyboard
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue

# 50 hz 기준 
def on_live_data_available(packet):
    """index 순서대로 
    0. packetCounterAvailable
    1. sampleTimeFineAvailable
    2. utcTimeAvailable
    3. eulerAvailable
    4. quaternionAvailable
    5. rotAvailable
    6. accAvailable
    7. magAvailable
    8. latlonAvailable
    9. velocityAvailable    
    """ 

    live_data = [[] for _ in range(10)] 
        
    xbus_data = XsDataPacket() 
    DataPacketParser.parse_data_packet(packet, xbus_data)

    if xbus_data.packetCounterAvailable:
        # print(f"\npacketCounter: {xbus_data.packetCounter}, ", end='')
        live_data[0].append(xbus_data.packetCounter)

    if xbus_data.sampleTimeFineAvailable:
        # print(f"sampleTimeFine: {xbus_data.sampleTimeFine}, ", end='')
        live_data[1].append(xbus_data.sampleTimeFine)

    if xbus_data.utcTimeAvailable:
        print(f"utctime epochSeconds: {xbus_data.utcTime:.9f}")
        live_data[2].append(xbus_data.utcTime)

    if xbus_data.eulerAvailable:
        # print(f"\nRoll, Pitch, Yaw: [{xbus_data.euler[0]:.2f}, {xbus_data.euler[1]:.2f}, {xbus_data.euler[2]:.2f}], ", end='')
        live_data[3].extend([xbus_data.euler[0],xbus_data.euler[1],xbus_data.euler[2]])

    if xbus_data.quaternionAvailable:
        print(f"q0, q1, q2, q3: [{xbus_data.quat[0]:.4f}, {xbus_data.quat[1]:.4f}, {xbus_data.quat[2]:.4f}, {xbus_data.quat[3]:.4f}], ", end='')
        live_data[4].extend([xbus_data.quat[0],xbus_data.quat[1],xbus_data.quat[2],xbus_data.quat[3]])

    if xbus_data.rotAvailable:
        rate_of_turn_degree = [
            xbus_data.rad2deg * xbus_data.rot[0],
            xbus_data.rad2deg * xbus_data.rot[1],
            xbus_data.rad2deg * xbus_data.rot[2]
        ]
        print(f"\nRateOfTurn: [{rate_of_turn_degree[0]:.2f}, {rate_of_turn_degree[1]:.2f}, {rate_of_turn_degree[2]:.2f}], ", end='')
        live_data[5].extend([rate_of_turn_degree[0],rate_of_turn_degree[1],rate_of_turn_degree[2]])

    if xbus_data.accAvailable:
        print(f"Acceleration: [{xbus_data.acc[0]:.2f}, {xbus_data.acc[1]:.2f}, {xbus_data.acc[2]:.2f}], ", end='')
        live_data[6].extend([xbus_data.acc[0],xbus_data.acc[1],xbus_data.acc[2]])

    if xbus_data.magAvailable:
        print(f"Magnetic Field: [{xbus_data.mag[0]:.2f}, {xbus_data.mag[1]:.2f}, {xbus_data.mag[2]:.2f}]")
        live_data[7].extend([xbus_data.mag[0],xbus_data.mag[1],xbus_data.mag[2]])

    if xbus_data.latlonAvailable and xbus_data.altitudeAvailable:
        print(f"\nLat, Lon, Alt: [{xbus_data.latlon[0]:.9f}, {xbus_data.latlon[1]:.9f}, {xbus_data.altitude:.9f}]")
        live_data[8].extend([xbus_data.latlon[0],xbus_data.latlon[1]])

    if xbus_data.velocityAvailable:
        print(f"Vel E, N, U: [{xbus_data.vel[0]:.9f}, {xbus_data.vel[1]:.9f}, {xbus_data.vel[2]:.9f}]")
        live_data[9].extend([xbus_data.vel[0],xbus_data.vel[1],xbus_data.vel[2]])

    # print('123123123')
    return live_data

def processing_yaw(yaw_degree):
    if yaw_degree >0:
        return (180-abs(yaw_degree))
    elif yaw_degree <0:
        return -(180-abs(yaw_degree))
    else:
        return 0
    
# IMU 각도 전처리
def processing_roll_pitch(roll, pitch):

    avg_roll_pitch = (roll + pitch) / 2
    # res = abs(avg_roll_pitch) - 90
    res = (avg_roll_pitch + 45)*2
    return res

def plt_show(data_queue):
    min_av =0
    max_av =0

    data_length = 300  # 데이터 길이 설정
    frame = np.zeros(data_length)
    thigh_angles = np.zeros(data_length)  # 초기 허벅지 각도 데이터
    thigh_angles_norm = np.zeros(data_length)  # 초기 허벅지 각도 데이터
    thigh_velocities = np.zeros(data_length)  # 초기 허벅지 속도 데이터

    # frame =[]
    # thigh_angles = []  # 초기 허벅지 각도 데이터
    # thigh_angles_norm = []  # 초기 허벅지 각도 데이터
    # thigh_velocities = []  # 초기 허벅지 속도 데이터

    fig, ax = plt.subplots()
    line, = ax.plot(frame, thigh_velocities, 'b-')  # 초기 그래프
    # line, = plt.plot([], [], 'r', animated=True)
    ax.set_xlim(100, 400)  # X축 범위 설정
    ax.set_ylim(-150, 150)  # Y축 범위 설정

    # plt.xlabel('Thigh Angle (rad)')
    plt.ylabel('Thigh Velocity (deg/s)')
    plt.title('Thigh Angle Phase Portrait')

    def update_graph(i, data_queue):
        while not data_queue.empty():
            live_data_list = data_queue.get_nowait()  # 큐에서 데이터 가져오기
            print('live_data_list : ', live_data_list )

            # 가끔씩 못받아오는 경우 있으므로,
            if len(live_data_list[3]) != 0:
                # yaw 각도 받아와서 스케일링
                yaw_degree = live_data_list[3][1]
                # yaw_degree = processing_yaw(yaw_degree)
                roll_degree = live_data_list[3][0]
                pitch_degree = live_data_list[3][1] 
                
                avg_roll_pitch = processing_roll_pitch(roll_degree, pitch_degree)
                new_angle = avg_roll_pitch
                print('new_angle: ', new_angle)

                # frame 리스트
                frame[:-1] = frame[1:]
                frame[-1] = i

                # # frmae
                # frame.append(i)

                # velocity 계산 위한 thigh_angles
                thigh_angles[:-1] = thigh_angles[1:]
                thigh_angles[-1] = new_angle

                # # velocity 계산 위한 thigh_angles
                # thigh_angles.append(new_angle)

                if abs(np.gradient(thigh_angles)[-1]*50) < 400:
                    thigh_velocities[:-1] = thigh_velocities[1:]
                    thigh_velocities[-1] = np.gradient(thigh_angles)[-1]*50 #50hz

                # if len(thigh_angles) >= 2:
                #     if abs(np.gradient(thigh_angles)[-1]*50) < 400:
                #         thigh_velocities.append(np.gradient(thigh_angles)[-1]*50) #50hz

                # normalized 된 thigh angle
                thigh_angles_norm[:-1] = thigh_angles_norm[1:]
                thigh_angles_norm[-1] = scale_angle_to_angular_velocity(new_angle, -50, 50)


                # print('frame', frame)
                # print('thigh_velocities',thigh_velocities )
                

                # 그래프 업데이트
                line.set_data(frame, thigh_velocities)
                
        return line,

    ani = animation.FuncAnimation(fig, update_graph, fargs=(data_queue,), interval=10)

    plt.show()

    return thigh_velocities

def read_data(data_queue):
    # try:
    # serial = SerialHandler("COM6", 115200) ##change the port and baudrate to your own MTi's baudrate.
    packet = XbusPacket(on_data_available=on_live_data_available)

    print("Listening for packets...")
    
    # 송신부 블루투스 모듈
    serial_sd1000 = serial.Serial('COM10', 115200, timeout=1)
    # thread = threading.Thread(target=read_thread, args=(serial_sd1000,packet,),daemon=True)  # 시리얼 통신 받는 부분
    # thread.start()
    while True:
        # if serial_sd1000.in_waiting > 0:
        if serial_sd1000.readable():  # 값이 들어왔는지 확인
            # res = serial_sd1000.readline()  # 값 받기 (byte 형식)
            # print(f'raw data: {res.hex()}')
            res_ser = serial_sd1000.read(1)
            # print(f'raw data - res_ser: {res_ser}')
            # print(f"raw byte: 0x{res_ser.hex()}")


            packet.feed_byte(res_ser)
            live_data_list = packet.return_live_data_list()
            if live_data_list:
                # print(f'받음 live_data_list: {live_data_list}')
                data_queue.put(live_data_list)  # 큐에 데이터 저장
                # print('data_queue - live_data_list[0] : ', live_data_list[3][2])

        # q 키를 누르면 종료
        if keyboard.is_pressed('q'):
            break

    print('끝')

def scale_angle_to_angular_velocity(angle, min_angular_velocity, max_angular_velocity):
    scaled_angle = (angle - min_angular_velocity) / (max_angular_velocity - min_angular_velocity)
    return scaled_angle

if __name__ == '__main__':
    # min_av = 0
    # max_av = 0 
    # 전역 데이터 큐
    data_queue = Queue()
    thread = threading.Thread(target=read_data, args=(data_queue,), daemon=True)
    thread.start()

    thigh_velocities = plt_show(data_queue)

    print('thigh_velocities  : ', thigh_velocities*50)
    if keyboard == 'e':
        thread.join()  # 쓰레드가 종료될 때까지 기다림
