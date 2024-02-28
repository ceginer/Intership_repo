# Hurotics Internship Repo - Gait cycle analysis

## Introduction
```
➡️ 현장실습 인턴십
📆 2023.12.22. ~ 2024.02.29.
🗨️ 허벅지 단일 IMU를 이용한 Gait Cycle 분석 및 시각화
```

## Abstract
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/08171338-b716-4ce6-be7a-7c2e1e0e7c8b" height="80%" width="80%">

<br/>

- 허벅지 단일 IMU 로부터, **thigh angle**, **angular velocity** 를 활용해 **Phase portrait 를 시각화**하여 사용자 스스로의 보행분석 시각화 및 패턴 파악에 활용
- 도출된 Phase portrait 를 통해, **최종적으로 `Step Velocity` 와 `Step Length` 파악**


## Baseline - Reference
- [Real-Time Continuous Gait Phase and Speed Estimation from a Single Sensor 
(2017, IEEE, Sensors)](https://ieeexplore.ieee.org/document/8062565)
- [Estimation of Walking Speed and Its Spatiotemporal Determinants Using a Single Inertial Sensor Worn on the Thigh: From Healthy to Hemiparetic Walking 
(2021, Sensors)](https://www.mdpi.com/1424-8220/21/21/6976)
- [Walking-Speed-Adaptive Gait Phase Estimation for Wearable Robots 
(Sensors, 2023)](https://www.mdpi.com/1424-8220/23/19/8276)


## Methods
### 1. Bluetooth 통신을 위한 데이터 송수신
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/66544f21-6d30-44d4-84f1-52f2dfff3b60" height="80%" width="80%">
  
- IMU 센서를 통한 RS232 통신을 통해 받은 데이터를 블루투스 송신기(ST1000) 에서 SPP 방식 (Serial Port Protocol) 을 통해 수신기(ST1000D) 가 받아 PC 의 Python 으로 데이터 처리

### 2. Bluetooth 통신을 위한 데이터 송수신
<table>
  <tr>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/4c607cd8-429a-4a91-9d47-9425349ed119"height="200" width="300"></td>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/3d40dc08-50e2-40c7-be92-809c001d206b"height="200" width="300"></td>
  </tr>
  <tr>
    <td>Fig 1) Thigh angle 측정</td>
    <td>Fig 2) Thigh angular velocity 측정</td>
  </tr>
</table>

- Quaternion 방식과 Euler 방식 중에서,  논문에서의 Euler 각도방식 적용

- 허벅지의 앞, 옆 모두 호환하기 위해 Roll, Pitch 방향의 중간값을 사용

- angular velocity 는 얻어진 angle 의 미분을 통해 계산

### 3. Phase Portrait 를 이용한 Gait analysis
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/36b4d102-e3e1-45e2-bd96-b06f0076eaf9" height="80%" width="80%">

<br/>

- 얻어진 angle 과 angular velocity 를 각각 x,y 축으로 하여 만들어진 Phase portrait 를 정규화하여 원처럼 만들어 시각화하고, 그 반지름을 통해 Step Velocity 를 얻는 것이 목적
- Raw 데이터에서 우선은 Low-pass Filter (Butterworth 필터) 를 적용하여 이상치 제거
- 정규화 방식은 논문 2번째의 baseline 를 참조하여 적용

### 4. 주기 측정 방식
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/49b163c8-28c1-49ca-b423-f34f3d734693" height="80%" width="80%">

<br/>


- raw data 의 이상치를 제거한 filtered data 로 만들어진 phase portrait 를 활용하여 현재 각도가 얼마를 이루는지 판단
- 각도가 30 ≤ $\theta$ ≤ 330 일 때 (이상치 고려), 한 주기 완료

### 5. Velocity, Step Length 측정
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/76121e2a-bc3b-4efe-8eb3-8d5b39cee20b" height="80%" width="80%">

<br/>

- 산점도 그래프에서의 x,y 좌표를 이용한 **한 주기에서의 $r$** (반지름 = L2 norm) 평균값 구하기
- 산출된 $r$을 이용하여 **`Step Velocity`** = $c_1 \cdot r+c_2$ 도출
- 주기시간 $T$ 를 이용하여 **`Step Length`** = $T \cdot v$ 로 도출


## Resulsts
### Scatter plot graph of one cycle
<table>
  <tr>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/92fbeb2b-473a-410e-a60d-cb74d89035d9"height="200" width="300"></td>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/5b90234d-2186-4610-89c3-244ffb9afdf6"height="200" width="300"></td>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/147d7ee6-fcdc-4650-b492-8d819ec6f917"height="200" width="300"></td>
  </tr>
  <tr>
    <td>Fig 1> $v$=0.5 m/s</td>
    <td>Fig 2> $v$=0.8 m/s</td>
    <td>Fig 3> $v$=1.1 m/s</td>
  </tr>
  <tr>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/99c2bcff-7e2c-4af0-9de2-0dc48873c8ee"height="200" width="300"></td>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/3f4c08f1-8874-4d5c-85c5-2a74ea344cee"height="200" width="300"></td>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/8af3eb90-1160-4971-a177-76cbeb70f11f"height="200" width="300"></td>
  </tr>
  <tr>
    <td>Fig 4> $v$=1.4 m/s</td>
    <td>Fig 5> $v$=1.1 m/s</td>
    <td>Fig 4> $v$=1.4 m/s</td>
  </tr>
</table>

### Linear Regression
<img src="https://github.com/ceginer/Intership_repo/assets/92140163/6d066cd6-4a64-4e93-944c-aa05e1aaa714" height="80%" width="80%">

<br/>

```
# Linear Regression func.

def step_velocity(r):
    a1 = 0.0081648 # 선형회귀 기울기
    b1 = -0.1885588449802471 # 선형회귀 절편
    return a1*r+b1

def step_length(step_velocity, step_time):
    return step_velocity*step_time
```

## Preview

```
☑️ 실제 IMU를 착용한 채 걸었을 때, 실시간으로 Step Velocity, Step Length 및 Phase portrait 측정 가능
```

### Real-time analysis - Phase portrait

<table>
  <tr>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/7bf98dcd-c4fe-45c5-90a6-a1301b920bdc"height="80%" width="120%">
  </tr>

</table>

### Real-time analysis - Scatter Plot 
<table>
  <tr>
    <td><img src="https://github.com/ceginer/Intership_repo/assets/92140163/b4b60eba-c914-441c-9642-912a7b5053ea" height="80%" width="80%">
  </tr>
</table>

</br>


