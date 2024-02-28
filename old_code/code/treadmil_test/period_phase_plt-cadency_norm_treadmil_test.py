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
from scipy.signal import butter, lfilter
import math

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
        # print(f"q0, q1, q2, q3: [{xbus_data.quat[0]:.4f}, {xbus_data.quat[1]:.4f}, {xbus_data.quat[2]:.4f}, {xbus_data.quat[3]:.4f}], ", end='')
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

    return live_data


def read_data(data_queue):
    packet = XbusPacket(on_data_available=on_live_data_available)

    print("Listening for packets...")
    
    # 송신부 블루투스 모듈
    serial_sd1000 = serial.Serial('COM10', 115200, timeout=1)
    while True:
        if serial_sd1000.readable():  # 값이 들어왔는지 확인
            res_ser = serial_sd1000.read(1)
            packet.feed_byte(res_ser)
            live_data_list = packet.return_live_data_list()
            if live_data_list:
                data_queue.put(live_data_list)  # 큐에 데이터 저장

        # q 키를 누르면 종료
        if keyboard.is_pressed('q'):
            break

    print('끝')

def plt_show(data_queue):
    
    # 각도 처리 ------------------------------------------------------------
    # portrait 의 각도 측정 함수
    def portrait_deg(x,y):
        deg = np.rad2deg(np.arctan2(y, x))
        if deg < 0:
            deg = 180+(180-abs(deg))
            
        return deg
    
    # IMU 각도 전처리
    def processing_yaw(yaw_degree):
        if yaw_degree >0:
            return (180-abs(yaw_degree))
        elif yaw_degree <0:
            return -(180-abs(yaw_degree))
        else:
            return 0
        
    # IMU 각도 전처리
    def processing_pitch(pitch_degree, roll_degree):
        # if roll_degree >=0:
        #     return (90-abs(pitch_degree))
        # elif roll_degree <0:
        #     return -(90-abs(pitch_degree))
        
        return (90-abs(pitch_degree))
    
    # IMU 각도 전처리
    def processing_roll_pitch(roll, pitch):
        avg_roll_pitch = roll + pitch / 2
        res = abs(avg_roll_pitch) - 90
        return res
    
    # portrait 주기 측정
    def count_cycle(portrait_deg):
        if portrait_deg > 310 or portrait_deg < 50:  # 360을 넘어가는 조건
            return True
        else:
            return False
        
    # normalized 함수
    def normalize_phase(min_max_values_list, theta, theta_dot):

        theta_max, theta_min, theta_dot_max, theta_dot_min = min_max_values_list
        # parameters
        z_t = np.abs(theta_dot_max - theta_dot_min) / np.abs(theta_max - theta_min) # z(t)
        gamma_t = -(theta_max + theta_min) / 2.0 # γ(t)
        gamma_dot_t = -(theta_dot_max + theta_dot_min) / 2.0 # Γ(t)

        # 계산
        theta_x_t = z_t * (theta + gamma_t) # θx(t)
        theta_dot_y_t = -(theta_dot + gamma_dot_t) # ˙θy(t) 

        # return z_t, gamma_t, gamma_dot_t, theta_x_t, theta_dot_y_t
        return theta_x_t, theta_dot_y_t
    
    # new normalized 함수
    def new_normalize_phase(min_max_values_list, theta, theta_dot):

        theta_max, theta_min, theta_dot_max, theta_dot_min = min_max_values_list

        # 중심 계산
        theta_o = (theta_max + theta_min) / 2
        theta_dot_o = (theta_dot_max + theta_dot_min) / 2
        
        # 진폭 계산
        A_theta = (theta_max - theta_min) / 2
        A_theta_dot = (theta_dot_max - theta_dot_min) / 2

        normalized_angle = (theta - theta_o) / A_theta
        normalized_angular_velocity = (theta_dot - theta_dot_o) / A_theta_dot
        
        return normalized_angle, normalized_angular_velocity
    
    # 무한 임펄스 필터 적용    
    def apply_infinite_impulse_filter1(min_max_values_list, param, pole=0.98):

        theta_max, theta_min, theta_dot_max, theta_dot_min = min_max_values_list

        # 초기화
        prev_theta_o = (theta_max + theta_min) / 2
        prev_theta_dot_o = (theta_dot_max + theta_dot_min) / 2

        filtered_param_theta = (1 - pole) * prev_theta_o + pole * param
        filtered_param_theta_dot = (1 - pole) * prev_theta_dot_o + pole * param

        return filtered_param_theta, filtered_param_theta_dot
    
    
    # 필터 ------------------------------------------------------------
    # 필터 파라미터 설정
    order = 16  # 필터 차수 (조절 가능)
    cutoff_frequency = 0.1  # 차단 주파수 (0.0 ~ 1.0 사이 값으로 조절)

    # Butterworth 필터 생성
    def butter_lowpass(cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def butter_lowpass_filter(data, cutoff, fs, order=5):
        b, a = butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    
    # 필터 적용 함수
    def apply_filter(data):
        fs = 1.8  # 샘플링 주파수 (예시)
        filtered_data = butter_lowpass_filter(data, cutoff_frequency, fs, order)
        return filtered_data

    # 처리 ----------------------------------------------------------
    data_length = 200  # 데이터 길이 설정
    thigh_angles = np.zeros(data_length)  # 초기 허벅지 각도 데이터
    thigh_velocities = np.zeros(data_length)  # 초기 허벅지 속도 데이터

    # 필터링된 np
    filtered_a = np.zeros(data_length) # 초기 허벅지 속도 데이터
    filtered_v = np.zeros(data_length)  # 초기 허벅지 속도 데이터

    # normalized 용 np
    normalize_a = np.zeros(data_length)
    normalize_v = np.zeros(data_length)


    # new normalized 용 np
    new_normalize_a = np.zeros(data_length)
    new_normalize_v = np.zeros(data_length)


    # 초기 x, y 좌표 설정
    current_x = 0
    current_y = 0

    # 초기 normalized 설정
    start_sign = False
    # angle_x_max = 0
    # angle_x_min = 0
    # angle_y_dot_max = 0
    # angle_y_dot_min = 0
    angle_x_max = 0
    angle_x_min = 0
    angle_y_dot_max = 0
    angle_y_dot_min = 0
    out_of_cycle_i = 0
    out_of_cycle = 0

    min_max_values_list =[]

    xy_data_list = []
    sampletimefine = 0
    period = 0

    avg_r_list=[]

    # 송신 주파수 Tx (Transmit_frequency)
    Transmit_frequency = 50 # 50hz 기본

    # matplotlib 그래프 설정
    fig, ax = plt.subplots()
    ## 2개 그래프 비교
    line1, = ax.plot(thigh_angles, thigh_velocities, 'b-', label='원래 그래프')  # 원래 그래프
    line2, = ax.plot(filtered_a, filtered_v, 'r-', label='필터링된 그래프')  # 필터링된 그래프
    
    # 3개의 그래프 비교
    line3, = ax.plot(normalize_a, normalize_v, 'g-', label='정규화된 그래프')  # 정규화된 그래프
    line4, = ax.plot(new_normalize_a, new_normalize_v, 'y-', label='새로 정규화된 그래프')  # 정규화된 그래프
    
    # 1개의 그래프
    # line, = ax.plot(thigh_angles, thigh_velocities, 'b-')  # 초기 그래프

    ax.set_xlim(-100, 100)  # X축 범위 설정
    ax.set_ylim(-100, 100)  # Y축 범위 설정
    
    # ax.set_xlim(-250, 250)  # X축 범위 설정
    # ax.set_ylim(-250, 250)  # Y축 범위 설정

    # ax.set_xlim(-3, 3)  # X축 범위 설정
    # ax.set_ylim(-3, 3)  # Y축 범위 설정

    text_x = ax.text(130, -100, f'X: {current_x:.2f}', fontsize=12, ha='left')
    text_y = ax.text(130, -200, f'Y: {current_y:.2f}', fontsize=12, ha='center')
    text_z = ax.text(130, -300, f'portrait_deg: {current_y:.2f}', fontsize=12, ha='center')
    
    plt.xlabel('Thigh Angle (deg)')
    plt.ylabel('Thigh Velocity (deg/s)')
    plt.title('Thigh Angle Phase Portrait')
    plt.legend(loc='upper left')


    def update_graph(i, data_queue):
        nonlocal start_sign, angle_x_max, angle_x_min, angle_y_dot_max, angle_y_dot_min
        nonlocal out_of_cycle_i, xy_data_list
        nonlocal min_max_values_list
        nonlocal sampletimefine, Transmit_frequency, out_of_cycle ,period 
        nonlocal new_normalize_v, new_normalize_a, avg_r_list
        while not data_queue.empty():
            live_data_list = data_queue.get_nowait()  # 큐에서 데이터 가져오기
            # print('live_data_list : ', live_data_list )
            if len(live_data_list[3]) != 0:
                if sampletimefine:
                    diff_time = live_data_list[1][0] - sampletimefine
                    Transmit_frequency = 1/(diff_time*0.0001)
                    # print('diff_time : ', diff_time)
                    # print('current : ', live_data_list[1])
                    # print('diff_time_sec : ',diff_time*0.0001)
                    # print('hz : ',1/(diff_time*0.0001))
                sampletimefine = live_data_list[1][0]
                roll_degree = live_data_list[3][0]
                pitch_degree = live_data_list[3][1] 
                yaw_degree = live_data_list[3][2]
                
                # print('yaw_degree: ', yaw_degree)
                
                avg_roll_pitch = processing_roll_pitch(roll_degree, pitch_degree)
                # print('avg_roll_pitch : ',avg_roll_pitch)
                new_angle = avg_roll_pitch
                
                if abs(np.gradient([thigh_angles[-2],new_angle])[-1]) < 100: # 이상치 속도 제거
                    thigh_angles[:-1] = thigh_angles[1:]
                    thigh_angles[-1] = new_angle

                    thigh_velocities[:-1] = thigh_velocities[1:]
                    thigh_velocities[-1] = np.gradient(thigh_angles)[-1]*(-1)*Transmit_frequency # (*-1) 그래프 방향 바꿈 # Transmit_frequency 곱하기
                        
                    # butterworth 필터 적용 (Low-Pass 필터)
                    filtered_angles = apply_filter(thigh_angles)
                    filtered_velocities = apply_filter(thigh_velocities)

                    filtered_a[:-1] = filtered_a[1:]
                    filtered_a[-1] = filtered_angles[-1]
                    
                    filtered_v[:-1] = filtered_v[1:]
                    filtered_v[-1] = filtered_velocities[-1]

                    # normalized 적용------------------------
                    normalize_a[:-1] = normalize_a[1:]
                    normalize_v[:-1] = normalize_v[1:]

                    if min_max_values_list:
                        # print('min_max_values_list : ', min_max_values_list)
                        normalized_angle_x,normalized_angle_dot_y = normalize_phase(min_max_values_list,filtered_a[-1], filtered_v[-1])
                        normalize_a[-1] = normalized_angle_x
                        normalize_v[-1] = normalized_angle_dot_y
                    else:
                        normalize_a[-1] = filtered_angles[-1]
                        normalize_v[-1] = filtered_velocities[-1]
                    # new normalized---------------------------------
                        
                    filtered_angles = apply_filter(thigh_angles)
                    filtered_velocities = apply_filter(thigh_velocities)
                    
                    new_normalize_a[:-1] = new_normalize_a[1:]
                    new_normalize_v[:-1] = new_normalize_v[1:]
                    
                    if min_max_values_list:
                        # print('min_max_values_list : ', min_max_values_list)
                        new_normalize_angle_x,new_normalize_angle_dot_y = new_normalize_phase(min_max_values_list,filtered_a[-1], filtered_v[-1])
                        new_normalize_a[-1], _ = apply_infinite_impulse_filter1(min_max_values_list, new_normalize_angle_x)
                        _ , new_normalize_v[-1] = apply_infinite_impulse_filter1(min_max_values_list, new_normalize_angle_dot_y)
                        # new_normalize_a[-1] = new_normalize_angle_x
                        # new_normalize_v[-1] = new_normalize_angle_dot_y
                    else:
                        new_normalize_a[-1] = filtered_angles[-1]
                        new_normalize_v[-1] = filtered_velocities[-1]

                    #----------------------------------------------
                    # 업데이트된 x, y 좌표 계산
                    current_angle_x = filtered_angles[-1]
                    current_angle_y_dot = filtered_velocities[-1]
                    # current_deg = portrait_deg(thigh_angles[-1], thigh_velocities[-1])
                    current_deg = portrait_deg(filtered_a[-1], filtered_v[-1])

                    # 그래프 1개
                    # line.set_data(thigh_angles, thigh_velocities)

                    # 그래프 2개 비교
                    # line1.set_data(thigh_angles, thigh_velocities)
                    line2.set_data(filtered_a, filtered_v)
                    # 그래프 3개
                    line3.set_data(normalize_a, normalize_v)
                    line4.set_data(new_normalize_a, new_normalize_v)

                    text_x.set_text(f'current_angle_x: {current_angle_x:.2f}')
                    text_y.set_text(f'current_angle_y_dot: {current_angle_y_dot:.2f}')
                    text_z.set_text(f'current_deg: {current_deg:.2f}')

                    # print('thigh_angles , thigh_velocities: ', thigh_angles[-1], thigh_velocities[-1] )
                    # print('filtered_a , filtered_v: ', filtered_a[-1], filtered_v[-1] )
                    # print('normalize_a , normalize_v: ', normalize_a[-1], normalize_v[-1] )
                    
                    
                    # normalized -----------------------------------------
                    ### 시작 사이클 따지기
                    if count_cycle(current_deg): # 특정 각도내에 있을 때
                        if start_sign == False: # 초기 세팅은 start_sign = False
                            # 원래 계산
                            angle_x_max = max(current_angle_x, angle_x_max)
                            angle_y_dot_max = max(current_angle_y_dot,angle_y_dot_max)

                            # angle_x_min 이 양수일 때를 대비한 계산
                            # angle_x_min = current_angle_x
                            # angle_y_dot_min = current_angle_y_dot
                            angle_x_min = min(current_angle_x, 100000)
                            angle_y_dot_min = min(current_angle_y_dot, 100000)

                            # x,y 좌표 저장
                            xy_data_list =[]
                            xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])
                            
                            out_of_cycle = sampletimefine
                            # print('각도 내 out_of_cycle : ', out_of_cycle)
                            # print('각도 내 sampletimefine : ', sampletimefine)
                        else:
                            # if abs(i - out_of_cycle_i ) >= 10:
                            # start sign 바꾸기
                            start_sign = False

                            print('-'*70)
                            print('한 주기 끝')
                            # print('angle_x_max : ',angle_x_max)
                            # print('angle_x_min : ',angle_x_min)
                            # print('angle_y_dot_max : ',angle_y_dot_max)
                            # print('angle_y_dot_min : ',angle_y_dot_min)
                            # print('current : ',current_deg)
                            # print('한주기 i : ',i)
                            # print('out_of_cycle_i i : ',out_of_cycle_i)

                            # min_max_values
                            min_max_values_list = [angle_x_max,angle_x_min,angle_y_dot_max,angle_y_dot_min]
                            # print('한주기 이후의 min_max_values_list : ', min_max_values_list)

                            # print('변화 전 period:' , period )
                            # print('변화 전 out_of_cycle:' , out_of_cycle )
                            
                            # 주기 계산
                            period = sampletimefine-out_of_cycle
                            out_of_cycle = sampletimefine

                            # print('sampletimefine :' , sampletimefine )
                            # print('변화 후 period :' , period)
                            # print('변화 전 out_of_cycle:' , out_of_cycle )
                            print('period*0.0001 :' , period *0.0001)

                            # x와 y 좌표 추출
                            x_values = [coord[0] for coord in xy_data_list]
                            y_values = [coord[1] for coord in xy_data_list]
                            radius_list =[math.sqrt((x)**2 + (y)**2) for x,y in zip(x_values,y_values)]
                            avg_r = sum(radius_list)/len(radius_list)
                            
                            # print('xy_data_list : ', xy_data_list)
                            # print('x_values : ', x_values)

                            plt.xlim(-280, 280)  # X축 범위 설정
                            plt.ylim(-280, 280)  # Y축 범위 설정

                            if out_of_cycle_i <= 100:
                                plt.clf()
                                pass
                            else:
                                avg_r_list.append(avg_r)
                                print('r :', avg_r)
                                print('avg_r_list :', avg_r_list)
                            # print('x_values : ', x_values)
                            # print('y_values : ', y_values)

                            # # 그래프 그리기

                            plt.scatter(x_values, y_values, color='blue', marker='o')  # 산점도 그래프
                            plt.title('Scatter Plot of X and Y Coordinates')  # 그래프 제목
                            plt.xlabel('X Coordinate')  # x 축 레이블
                            plt.ylabel('Y Coordinate')  # y 축 레이블
                            plt.grid(True)  # 격자 표시
                            plt.show()  # 그래프 표시


                            out_of_cycle_i = i
                            angle_x_max = 0 
                            angle_x_min = 0
                            angle_y_dot_max = 0
                            angle_y_dot_min = 0
                            

                            xy_data_list = []
                            xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])
                    
                    # 시작 사이클이 아니라면 계산 수행 ( = 특정 각도를 벗어났을 때)
                    elif not count_cycle(current_deg):
                        # print('벗어남', current_deg, i)
                        angle_x_max = max(current_angle_x, angle_x_max)
                        angle_y_dot_max = max(current_angle_y_dot,angle_y_dot_max)
                        angle_x_min = min(current_angle_x, angle_x_min)
                        angle_y_dot_min = min(current_angle_y_dot, angle_y_dot_min)
                        # print(idx, angles[idx], angle_y_dot[idx], angle_y_dot_min)

                        if start_sign == False:
                            start_sign = True
                        else:
                            pass
                        
                        xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])

        # plt.savefig('treadmil_v_0.5_filter_3.png')
        # return line, text_x, text_y
        # return line1, line2, text_x, text_y
        return line3, text_x, text_y

    ani = animation.FuncAnimation(fig, update_graph, fargs=(data_queue,), interval=10)

    plt.savefig('sample_plot.png')
    plt.show()
    # 그래프를 이미지 파일로 저장

    print('thigh_angles :', thigh_angles)
    print('thigh_velocities :', thigh_velocities)
    print('filtered_angles :', filtered_a)
    print('filtered_velocities :', filtered_v)

if __name__ == '__main__':
    # 전역 데이터 큐
    data_queue = Queue()
    thread = threading.Thread(target=read_data, args=(data_queue,), daemon=True)
    thread.start()

    plt_show(data_queue)
    
    if keyboard == 'e':
        thread.join()  # 쓰레드가 종료될 때까지 기다림
