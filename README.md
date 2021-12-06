# ROS-Control-Basic
오늘날의 제어 시스템은 자동차(크루즈 모드에서 속도 제어용)에서 달 착륙선(연착륙을 위한 방향 및 속도 제어용)에 이르기까지 모든 곳에 있습니다. 

이 페이지에서는 제어 시스템에 대해 설명하지 않지만 로봇 공학에서 ros 제어 패키지를 사용하는 방법을 살펴보겠습니다.
PID 제어 시스템이 무엇이고 어떻게 작동하는지에 대한 사전 지식이 있다고 가정하고 진행하겠습니다. 

- [그림 1  피드백 루프에서 PID 컨트롤러 의  블록 다이어그램]

![PID_en svg](https://user-images.githubusercontent.com/93853610/144765839-85d9f6c2-eedc-4394-9a47-6d35f3942037.png)


## 로봇에서 제어가 필요한 이유에 대해 ????
로봇을 다룰 때 로봇의 동작이 제어되는지 확인해야 한다. 
즉, 동작이 너무 느리거나 너무 빠르지 않고 지정된 궤적을 따르고 있는지 확인해야 한다.

테이블에서/위에서 물건을 선택/배치하고 싶다고 상상해 보자. 그런 다음 자신/물건이 테이블을 세게 치지 않고 가능한 짧은 시간에 그것을 놓는 방식으로 손을 움직인다. 
여기에서 동작 전반에 걸쳐 손의 위치와 속도를 제어한다. 로봇에도 똑같이 적용해야 한다. 
로봇이 적절한 장소에 적절한 시간에 물체를 놓을 필요가 있을 때마다 우리는 물체의 움직임을 제어해야 한다.

ROS는 PID 컨트롤러를 사용하여 로봇의 동작을 제어하는데 사용할 수 있는 패키지 세트를 제공한다. 
원하는 경우 자체 컨트롤러 플러그인을 작성할 수도 있다. 그러나 우리가 우리 자신의 컨트롤러를 작성하기 시작할 때 샘플링 속도, derivative kick, on-the-fly tuning parameters, 리셋 와인드업 완화 등과 같은 많은 것을 처리해야 한다. 
ROS Control 패키지는 이러한 모든 측면은 사용자를 위해 패키징 되어 있다.

 ROS는 로봇에 사용할 수 있는 ros_control 이라는 일반화된 제어 패키지를 제공 하여 컨트롤러 코드를 다시 작성하는 시간을 줄여준다.  
 
## ROS Control

![image](https://user-images.githubusercontent.com/93853610/144766052-8f1b44fd-7db8-47a5-a98e-628fe2c73f8b.png)

[A block diagram of ROS Control] - Pic Credit: http://wiki.ros.org/ros_control


ROS_Control은 
- controller interface
- controller manager
- transmissions 
- hardware interfaces 
- Control toolbox 
를 포함하는 패키지 세트다.  이 모든 패키지를 함께 사용하면 로봇의 조인트 액추에이터와 상호 작용하고 제어할 수 있다.

ros_control 은 아래 그림과 같이 사용자/3rd party 어플리케이션의 입력으로 관절 상태 데이터와 입력 설정값(목표)를 취하고 
적절한 명령을 출력으로 액추에이터에 보낸다. 
제공된 설정점(목표)을 달성하기 위해 일반적으로 PID 컨트롤러와 같은 일반 제어 루프 피드백 메커니즘을 사용하여 출력을 제어 한다.
이 출력은 하드웨어 인터페이스를 통해 로봇에 전달된다.

![image](https://user-images.githubusercontent.com/93853610/144766483-cbfb60b9-f23c-4df1-98df-c326ef74568c.png)

[사진 제공: ROSCon 2014] 


위 그림에서 3rd party(Moveit1 / Navigation Stack)은 ros_control  패키지에 목표값을 보내는 블록을 나타낸다.
컨트롤러(base_controller/arm_controller/ base controller)은 설정된 목표를 달성하는데 필요한 풀력 명령을 계산하는 역활을 한다. 관절을 움직이기 위해서는 컨트롤러가 앱추에이터와 통신해야 한다. 하드웨어 인터페이스 노드(RobotHW) 는 컨트롤러와 실제 하드웨어 사이에 위치하며 액추에이터가 관절셍서로 부터 피드백을 받고 움직이도록 명령한다. 이것이 ros_control 이 작동하는 방식이다.

## ROS control 패키지 종류
ROS 컨트롤 패키지는 로봇의 관절과 다양한 방식으로 상호 작용할 수 있는 컨트롤러 플로글인 셋를 제공한다.
목록은 아래와 같다.

### [joint_state_controller]:
이 컨트롤러는 모든 관절위치를 읽고 "/joint_states" topic를 publishing  한다. 이 컨트롤러는 액추에이터에 명령을 보내지 않는다.
"joint_state_controller/JointStateController" 로 사용된다.

### [effort_controllers]:
effort interface 에 명령을 보낼때 사용한다. 이것은 제어하고자 하는 관절 액추에이터가 힘(전류/전압) 명령을 받아들인다는 의미이다.
#### [effort_controllers/JointPositionController] : 
이 컨트롤러 플러그인은 position(라디안/미터)값을 입력으로 받아들인다. error(목표위치 - 현재위치)는 PID 루프를 통해 출력 effort command에 매핑된다. 

#### [effort_controllers/JointVelocityController] : 
이 컨트롤러 플러그인은 속도(라디안/초, 미터/초)값을 입력으로 받아들인다. error(목표속도 - 현재 속도)는 PID 루프를 통해 출력 effort command 에 매핑된다.

#### [effort_controllers/JointEffortController] :
이 컨트롤러 플러그인은 힘 또는 토크값을 입력으로 받아들인다. Input  effort 은 output effort command 으로 액추에이터에 전달된다. PID의 값은 output effort 에 영향을 미치니 않는다.

#### [effort_controllers/JointGroupPositionController] :
이 컨트롤러는 Joint Group 에 대한 "effort_controllers/JointPositionController" 와 동일한 기능을 한다.

#### [effort_controllers/JointGroupEffortController] :
이 컨트롤러는 Joint Group 에 대한 "effort_controllers/JointEffortGroupController" 와 동일한 기능을 한다.

### [velocity_controllers] :
속도 인터페이스에 명령을 보낼때 사용한다.  이것은 제어하려는 관절 액추에이터가 속도 명령을 직접 받아들인다는 것을 의미한다.

#### [velocity_controllers/JointPositionController] :
이 컨트롤러 플러그인은 위치(라디안/미터)값을 입력으로 받아들인다. error(목표위치 - 현재위치) 는 PID 루프를 통해 output velocity command 에 맵핑된다.

#### [velocity_controllers/JointVelocityController] :
이 컨트롤러 플러그인은 속도(라디안/초, 미터/초) 값을 입력으로 받아들인다. 




















 
