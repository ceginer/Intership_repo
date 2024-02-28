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
import os
import pandas as pd

# live data 받는 func.
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


# PC에 연결된 블루투스 동글이가 받은 데이터를 [PC<->동글이] 간의 Serial 통신 func.
def read_data(data_queue):
    # Xbus 프로토콜의 인스턴스 생성
    packet = XbusPacket(on_data_available=on_live_data_available)

    print("Listening for packets...")
    
    # 송신부 블루투스 모듈, baudrate = 115200
    serial_sd1000 = serial.Serial('COM10', 115200, timeout=1)
    while True:
        if serial_sd1000.readable():  # 값이 들어왔는지 확인
            res_ser = serial_sd1000.read(1)
            packet.feed_byte(res_ser)
            live_data_list = packet.return_live_data_list() # 받은 데이터 list 로 반환
            if live_data_list:
                data_queue.put(live_data_list)  # 큐에 데이터 저장(데이터 끊김현상 방지)

        # q 키를 누르면 종료, 보통 ctrl+c 누르면 종료되긴 함.
        if keyboard.is_pressed('q'):
            break

    print('끝')

# matplotlib 을 활용한 그래프 시각화 func.
def plt_show(data_queue):

    # 속도, 스텝거리 계산------------------
    def step_velocity(r):
        a1 = 0.0081648 # 선형회귀 기울기
        b1 = -0.1885588449802471 # 선형회귀 절편
        return a1*r+b1

    def step_length(step_velocity, step_time):
        return step_velocity*step_time

    # csv 저장 ----------------------------------------------------
    def save_csv(treadmil_v,period_times,x_values,y_values):
        # 파일 경로 설정
        velocity = str(treadmil_v)
        file_path = f'front_raw_data_v_{velocity}.csv'
        all_file_path = 'front_raw_data_all.csv'

        # 예비용 csv
        # 데이터 프레임 생성
        df = pd.DataFrame({'Treadmil_v': velocity, 'Period': period_times, 'X': x_values, 'Y': y_values})

        # 파일이 이미 존재하는지 확인 
        if os.path.isfile(file_path):
            # 파일이 존재하는 경우, 헤더를 추가하지 않고 데이터를 추가하여 저장
            df.to_csv(file_path, index=False, header=False, mode='a')
        else:
            # 파일이 존재하지 않는 경우, 헤더와 함께 데이터를 저장
            df.to_csv(file_path, index=False)

        # 모두 모아두는 파일
        if os.path.isfile(all_file_path):
            # 파일이 존재하는 경우, 헤더를 추가하지 않고 데이터를 추가하여 저장
            df.to_csv(all_file_path, index=False, header=False, mode='a')
        else:
            # 파일이 존재하지 않는 경우, 헤더와 함께 데이터를 저장
            df.to_csv(all_file_path, index=False)
    
    # 각도 처리 ------------------------------------------------------------
    # portrait 의 각도 측정 함수, base 논문 1에 관련 수식 있음.
    def portrait_deg(x,y):
        deg = np.rad2deg(np.arctan2(y, x))
        if deg < 0:
            deg = 180+(180-abs(deg))
            
        return deg
    
    # IMU 각도 전처리, avg(pitch+roll) 사용
    def processing_roll_pitch(roll, pitch):
        avg_roll_pitch = (roll + pitch) / 2 # 중간값
        res = (avg_roll_pitch + 45) * 2 # 평균을 0으로, 평균값을 원래 값으로 돌리기 위해 *2 
        return res
    
    # portrait 주기 측정, threshod (임의 조정 가능, 30~330 잘 파악됨)
    def count_cycle(portrait_deg, threhold_start_degree = 30 , threhold_end_degree = 330, remain_cycle = True):
        
        # cycle 이 초기화되는 조건 (30~330도 내부일 때)
        if remain_cycle:
            # threhold_start_degree -> 30 (default)
            # threhold_end_degree -> 330 (default)
            if portrait_deg < threhold_start_degree or portrait_deg > threhold_end_degree:  
                return True
            else:
                return False
            
        # cycle 이 다시 시작되는 조건 (30도 이상 180도 이하일 때)
        elif remain_cycle == False:
            if portrait_deg >= threhold_start_degree and portrait_deg <= 180:
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
    
    # new normalized 함수, base 논문 3 참조
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
    
    # 무한 임펄스 필터 적용, base 논문 3 참조
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
    order = 16  # 필터 차수 (조절 가능, 좀 더 수정 필요)
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

    # 처리 (초기 설정) ----------------------------------------------------------
    data_length = 200  # 데이터 길이 설정

    # angle (x축), angular velocity (y축) 살장
    thigh_angles = np.zeros(data_length)  # 초기 허벅지 각도 데이터
    thigh_velocities = np.zeros(data_length)  # 초기 허벅지 속도 데이터

    # Low-pass 필터링된 np
    filtered_a = np.zeros(data_length) 
    filtered_v = np.zeros(data_length) 

    # normalized 용 np (base 논문1)
    normalize_a = np.zeros(data_length)
    normalize_v = np.zeros(data_length)

    # new normalized 용 np (base 논문3)
    new_normalize_a = np.zeros(data_length)
    new_normalize_v = np.zeros(data_length)

    # 초기 설정 (전역 변수 설정)
    current_x = 0 
    current_y = 0

    start_sign = False

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
    period_times = 0

    avg_r_list=[]

    # 송신 주파수 Tx (Transmit_frequency)
    Transmit_frequency = 50 # 50hz 기본

    # matplotlib 그래프 설정
    fig, ax = plt.subplots()

    ## 그래프 비교
    line1, = ax.plot(thigh_angles, thigh_velocities, 'b-', label='원래 그래프')  # 원래 그래프
    line2, = ax.plot(filtered_a, filtered_v, 'r-', label='필터링된 그래프')  # 필터링된 그래프
    line3, = ax.plot(normalize_a, normalize_v, 'g-', label='normalized된 그래프')  # 정규화된 그래프
    line4, = ax.plot(new_normalize_a, new_normalize_v, 'y-', label='new normalized된 그래프')  # 정규화된 그래프

    # ax.set_xlim(-100, 100)  # X축 범위 설정
    # ax.set_ylim(-100, 100)  # Y축 범위 설정
    
    ax.set_xlim(-1000, 1000)  # X축 범위 설정
    ax.set_ylim(-1000, 1000)  # Y축 범위 설정

    # ax.set_xlim(-3, 3)  # X축 범위 설정
    # ax.set_ylim(-3, 3)  # Y축 범위 설정

    # 그래프 내 결과 숫자 표시
    text_x = ax.text(130, -100, f'X: {current_x:.2f}', fontsize=12, ha='left') # x축 값 (angle)
    text_y = ax.text(130, -200, f'Y: {current_y:.2f}', fontsize=12, ha='center') # y축 값 (angular velocity)
    text_z = ax.text(130, -300, f'portrait_deg: {current_y:.2f}', fontsize=12, ha='center') # x,y 이루는 각도 (phase portrait 각도)
    
    plt.xlabel('Thigh Angle (deg)')
    plt.ylabel('Thigh Velocity (deg/s)')
    plt.title('Thigh Angle Phase Portrait')
    plt.legend(loc='upper left')

    # matplotlib 그래프 업데이트 func. -------------------------------------------------------
    def update_graph(i, data_queue):
        # def 내에서 전역변수 변경 (-> nonlocal)
        nonlocal start_sign, angle_x_max, angle_x_min, angle_y_dot_max, angle_y_dot_min
        nonlocal out_of_cycle_i, xy_data_list
        nonlocal min_max_values_list
        nonlocal sampletimefine, Transmit_frequency, out_of_cycle ,period , period_times
        nonlocal new_normalize_v, new_normalize_a, avg_r_list

        # queue 내부가 비어있을 때까지 반복
        while not data_queue.empty():
            live_data_list = data_queue.get_nowait()  # 큐에서 데이터 가져오기
            # print('live_data_list : ', live_data_list )

            if len(live_data_list[3]) != 0:
                # sampletimefine 값을 통해 시간 간격 계산하여 frequency 계산
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

                # avg(pitch+roll) 입력       
                avg_roll_pitch = processing_roll_pitch(roll_degree, pitch_degree)
                new_angle = avg_roll_pitch
                # print('new_angle : ',avg_roll_pitch)
                
                # 각 그래프 별 list 생성
                if abs(np.gradient([thigh_angles[-2],new_angle])[-1]) < 100: # 이상치 속도 제거

                    # raw angle, angular velocity ---------------
                    thigh_angles[:-1] = thigh_angles[1:]
                    thigh_angles[-1] = new_angle

                    thigh_velocities[:-1] = thigh_velocities[1:]
                    thigh_velocities[-1] = np.gradient(thigh_angles)[-1]*Transmit_frequency # (-1)곱하기 -> 그래프 방향 바꿈 , m/s 로 계산하기 위해 Transmit_frequency 곱하기
                        
                    # butterworth 필터 적용 (Low-Pass 필터) ------------
                    filtered_angles = apply_filter(thigh_angles)
                    filtered_velocities = apply_filter(thigh_velocities)

                    filtered_a[:-1] = filtered_a[1:]
                    filtered_a[-1] = filtered_angles[-1]
                    
                    filtered_v[:-1] = filtered_v[1:]
                    filtered_v[-1] = filtered_velocities[-1]

                    # normalized 적용------------------------
                    normalize_a[:-1] = normalize_a[1:]
                    normalize_v[:-1] = normalize_v[1:]

                    # 그래프 돌아가면서 min-max angle, angular velocity 측정
                    if min_max_values_list:
                        normalized_angle_x,normalized_angle_dot_y = normalize_phase(min_max_values_list,filtered_a[-1], filtered_v[-1])
                        normalize_a[-1] = normalized_angle_x
                        normalize_v[-1] = normalized_angle_dot_y
                    else:
                        # 만약 제대로 측정되지 않았을 경우, 그래프 끊기지 않기 위해 그냥 filtering 된 그래프 출력 지시
                        normalize_a[-1] = filtered_angles[-1]
                        normalize_v[-1] = filtered_velocities[-1]

                    # new normalized 적용 ---------------------------------
                    filtered_angles = apply_filter(thigh_angles)
                    filtered_velocities = apply_filter(thigh_velocities)
                    
                    new_normalize_a[:-1] = new_normalize_a[1:]
                    new_normalize_v[:-1] = new_normalize_v[1:]
                    
                    # new normalized + 무한 임펄스 적용
                    if min_max_values_list:
                        new_normalize_angle_x,new_normalize_angle_dot_y = new_normalize_phase(min_max_values_list,filtered_a[-1], filtered_v[-1])
                        new_normalize_a[-1], _ = apply_infinite_impulse_filter1(min_max_values_list, new_normalize_angle_x)
                        _ , new_normalize_v[-1] = apply_infinite_impulse_filter1(min_max_values_list, new_normalize_angle_dot_y)
                    else:
                        new_normalize_a[-1] = filtered_angles[-1]
                        new_normalize_v[-1] = filtered_velocities[-1]

                    #----------------------------------------------
                    
                    # min-max를 판단할 x, y 좌표 계산 ( raw 데이터 대신 정제화된 filtering 데이터 사용 )
                    current_angle_x = filtered_a[-1]
                    current_angle_y_dot = filtered_v[-1]

                    # phase portrait 주기를 판단할 각도 계산 -> 보완 알고리즘 필요
                    # ( normalized 각도 사용(-> 주기 정확도 높음, 산점도 그래프 취약), min-max가 없을 때는 filterd data 로 취급되므로 상관X )
                    # ( filtered 각도 사용(-> 주기 정확도 낮음, 산점도 그래프 좋음), min-max가 없을 때는 filterd data 로 취급되므로 상관X )
                    current_deg = portrait_deg(filtered_a[-1], filtered_v[-1])
                    # print('angle :', current_deg)

                    # 그래프 업데이트
                    # line1.set_data(thigh_angles, thigh_velocities)
                    line2.set_data(filtered_a, filtered_v)
                    line3.set_data(normalize_a, normalize_v)
                    # line4.set_data(new_normalize_a, new_normalize_v)

                    text_x.set_text(f'current_angle_x: {current_angle_x:.2f}') # filterng x
                    text_y.set_text(f'current_angle_y_dot: {current_angle_y_dot:.2f}') # filterng y
                    text_z.set_text(f'current_deg: {current_deg:.2f}') # normalized portrait degree
                    
                    # normalized 를 위한 min-max 구하기 및 gait cycle 측정 -----------------------------------------
                    ### 시작 사이클 따지기
                    if count_cycle(current_deg, remain_cycle = True): # 특정 각도내에 있을 때
                        # 111111 --------------------------------------------- 각도 330~30도 내부에 있을 때
                        if start_sign == False: # 초기 세팅은 start_sign = False
                            # 원래 계산
                            angle_x_max = max(current_angle_x, angle_x_max)
                            angle_y_dot_max = max(current_angle_y_dot,angle_y_dot_max)
                            angle_x_min = min(current_angle_x, 100000)
                            angle_y_dot_min = min(current_angle_y_dot, 100000)

                            # 한 cycle 마다의 x,y 좌표 list 초기화 ( 여기서는 산점도 그래프를 위한 normalized data 저장 )
                            xy_data_list =[]
                            xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])
                            
                            # 특정 각도 내에 있을 때, 계속해서 sampletimefine 시간대 초기화
                            out_of_cycle = sampletimefine
                        # 444444  --------------------------------------------- 각도 330 내부로 들어옴
                        else: # (start_sign == True -> cycle 을 돌다가 특정 각도 내로 다시 돌아왔을 경우 )
                            # start sign 바꾸기
                            start_sign = False

                            print('-'*70)
                            print('한 주기 끝')

                            # cycle 동안의 min_max 값에 대한 list
                            min_max_values_list = [angle_x_max,angle_x_min,angle_y_dot_max,angle_y_dot_min]
                            # print('한주기 이후의 min_max_values_list : ', min_max_values_list)
                            
                            # 주기 계산
                            period = sampletimefine-out_of_cycle # (걸린시간) = (복귀한 현재시간) - (이전 cycle 출발시간)
                            out_of_cycle = sampletimefine # (이전 cycle 출발시간)을 (cycle 복귀시간) 으로 바꿈으로써, 한바퀴 돌고 난 이후로, <(걸린시간) = (복귀한 현재시간) - (이전 cycle 복귀시간)> 으로 바꾸기 위함

                            # 1주기당 걸린시간 출력 ( 현재 period 은 마이크로초 기준이므로, *0.0001 로 하여금 초로 바꿈)
                            step_time = period *0.0001
                            print('period*0.0001 :' , f'{step_time} second')

                            # 한 cycle 마다의 x,y 좌표 리스트를 통해 산점도 그래프 출력---------------------------
                            # cycle 마다의 x,y 좌표와 반지름 r 계산
                            x_values = [coord[0] for coord in xy_data_list]
                            y_values = [coord[1] for coord in xy_data_list]
                            radius_list =[math.sqrt((x)**2 + (y)**2) for x,y in zip(x_values,y_values)] # 각 그래프 x,y 좌표의 반지름 값
                            avg_r = sum(radius_list)/len(radius_list) # 한 cycle 에서의 x,y 좌표 반지름의 평균값
                            
                            # print('xy_data_list : ', xy_data_list)
                            # print('x_values : ', x_values)

                            # 산점도 그래프 출력 범위 지정
                            plt.xlim(-280, 280)  # X축 범위 설정
                            plt.ylim(-280, 280)  # Y축 범위 설정

                            # 일정 시간이 지난 후(=noramlized 가 된 이후부터)부터 출력 하기 위해 out_of_cycle_i 지정
                            if out_of_cycle_i <= 50:
                                plt.clf() # 그래프 초기화
                                pass
                            else:
                                avg_r_list.append(avg_r) # 반지름의 평균값을 list에 추가
                                print('avg_r :', avg_r)
                                print('avg_r_list :', avg_r_list)

                                period_times += 1
                                print('period_times : ', period_times)

                                step_vel = step_velocity(avg_r)
                                step_len = step_length(step_vel, step_time)

                                print('step_vel : ', step_vel)
                                print('step_len : ', step_len)

                                # 50주기만을 원할 때,
                                # if period_times > 50:
                                #     print('-'*1000)
                                #     break
                                

                                # csv 에 저장 (raw data 저장 시 사용) -> 현재는 normalized_data 가 csv 에 저장되고 있는중임.
                                # -> raw data 를 csv 에 저장하고 싶을 때는, xy_data_list.extend([[thigh_angles[-1],thigh_velocities[-1]]]) 로 바꿔주면 된다.
                                # save_csv('5.0',period_times, x_values, y_values)

                            # 산점도 그래프 그리기
                            plt.scatter(x_values, y_values, color='blue', marker='o')  # 산점도 그래프
                            plt.title('Scatter Plot of X and Y Coordinates')  # 그래프 제목
                            plt.xlabel('X Coordinate')  # x 축 레이블
                            plt.ylabel('Y Coordinate')  # y 축 레이블
                            plt.grid(True)  # 격자 표시
                            plt.show()  # 그래프 표시

                            # cycle 이 끝났으므로, cylce 초기화
                            out_of_cycle_i = i
                            angle_x_max = 0 
                            angle_x_min = 0
                            angle_y_dot_max = 0
                            angle_y_dot_min = 0
                            
                            xy_data_list = []
                            xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])
                    
                    # 222222  --------------------------------------------- 각도 30도를 벗어났을 때 (30도~180도)
                    if count_cycle(current_deg, remain_cycle = False):
                        # cycle 이 시작했으므로, start_sign 을 True 로 바꿔줌
                        if start_sign == False:
                            start_sign = True
                        else:
                            pass
                    
                    # 33333 ---------------------------------------------- 각도가 30도~330도 일 때

                    if (not count_cycle(current_deg, remain_cycle = True)):
                        # 시작 사이클이 아니라면 계산 수행 ( = 특정 각도를 벗어났을 때)
                        # min-max 계산
                        angle_x_max = max(current_angle_x, angle_x_max)
                        angle_y_dot_max = max(current_angle_y_dot,angle_y_dot_max)
                        angle_x_min = min(current_angle_x, angle_x_min)
                        angle_y_dot_min = min(current_angle_y_dot, angle_y_dot_min)

                        # # cycle 이 시작했으므로, start_sign 을 True 로 바꿔줌
                        # if start_sign == False:
                        #     start_sign = True
                        # else:
                        #     pass
                        
                        # cycle 동안의 x,y 리스트 계속 추가
                        xy_data_list.extend([[normalize_a[-1],normalize_v[-1]]])

        # # 지정된 산점도 그래프 저장
        # plt.savefig('side_treadmil_v_2.0_filter_1.8.png')
                        
        # return line, text_x, text_y
        # return line1, line2, text_x, text_y
        return line3, text_x, text_y

    ani = animation.FuncAnimation(fig, update_graph, fargs=(data_queue,), interval=10)

    # 그래프를 이미지 파일로 저장
    # plt.savefig('sample_plot.png')

    # 그래프 보이기
    plt.show()



    # print('thigh_angles :', thigh_angles)
    # print('thigh_velocities :', thigh_velocities)
    # print('filtered_angles :', filtered_a)
    # print('filtered_velocities :', filtered_v)

if __name__ == '__main__':
    # 전역 데이터 큐
    data_queue = Queue()
    thread = threading.Thread(target=read_data, args=(data_queue,), daemon=True)
    thread.start()

    plt_show(data_queue)
    
    if keyboard == 'e':
        thread.join()  # 쓰레드가 종료될 때까지 기다림
