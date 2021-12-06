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
이 컨트롤러 플러그인은 속도(라디안/초, 미터/초) 값을 입력으로 받아들인다. 입력속도값은 단순히 joint 액추에이터에 출력 명령으로 전달된다. PID 값은 출력명령에 영향을 미치지 않는다.

#### [velocity_controllers/JointGroupVelocityController] :
이 컨트롤러는 joint group 에 대한 velocity_controllers/JointVelocityController 와 같은 기능을 한다.

### [position_controllers] :
위치 인터페이스에 명령을 보낼때 사용한다. 즉, 제어 하려는 joint 액추에이터가 위치명령을 직접 수락한다.

#### [position_controllers/JointPositionController] :
이 컨트롤러 플러그인은 위치(라디안/미터)값을 입력으로 받아들인다. 입력 위치 값은 단순히 joint 액추에이터에 출력명령으로 전달된다. PID 값은 출력 명령에 영향을 미치지 않는다.

#### [position_controllers/JointGroupPositionController] :
이 컨트롤러는 position_controllers/JointPositionController 과 동일한 기능을 한다.

### [joint_trajectory_controllers] :
 이 컨트롤러는 joint group 에서 관절공간 궤적을 실행하는데 사용된다. 궤적은 도달할 웨이포인트 세트로 지정한다. 웨이포인트는 
 위치와 선택적으로 속도 및 가속도로 구성된다. (http://wiki.ros.org/joint_trajectory_controller)
 
 #### [effort_controllers/JointTrajectoryController] :
 이 컨트롤러 플러그인은 힘[전류(또는) 전압]을 수용하는 조인트 액츄에이터에 사용됩니다. 오류에 따른 위치+속도 궤적은 PID 루프를 통해 출력 에포트 명령에 매핑됩니다.
 
 #### [velocity_controllers/JointTrajectoryController] :
 이 컨트롤러 플러그인은 속도 명령을 직접 받는 조인트 액추에이터에 사용됩니다. 오류에 따른 위치+속도 궤적은 PID 루프를 통해 출력 속도 명령에 매핑됩니다.
 
 #### [position_controllers/JointTrajectoryController] :
 이 컨트롤러 플러그인은 위치 명령을 직접 받는 조인트 액추에이터에 사용됩니다. 지정된 궤적에서 원하는 위치가 단순히 관절로 전달됩니다. P, I, D의 값은 출력 명령에 영향을 미치지 않습니다.
 
위에 나열한 컨트롤러의 종류는 가장 기본적이고 가장 많이 사용하는 컨트롤러이다
ros_control 패키지는 몇 가지 더 유용한 컨트롤러를 제공한다. 
전체 컨트롤러 목록을 알고 싶다면 ros_control 패키지에 있는 ros_controllers 의 소스 코드를 살펴  보십시오. 
(https://github.com/ros-controls/ros_controllers)


## [hardware_interface] :
hardware_interface는 로봇 하드웨어 추상화를 구성하기 위한 모든 빌딩 블록을 구현한다. 
로봇의 소프트웨어 표현이다. 아래와 같이 다양한 유형의 모터/액추에이터 및 센서에 대한 인터페이스 세트가 있다.

- ### 조인트 액추에이터용 인터페이스
```
- EffortJointInterface : 이 인터페이스는 힘(전압/전류)명령을 받아들이는 조인트 액추에이터에 사용된다. 이 인터페이스는 effort_controllers 와 함꼐 사용된다.

- VelocityJointInterface : 이 인터페이스는 속도 명령을 직접 받는 도인트 액추에이터에 사용된다. velocity_controllers 와 함께 사용된다.

- PositionJointInterface : 이 인터페이스는 위치 명령을 직접 받는 조인트 액추에이터에 사용된다. position_controllers 와 함께 사용된다.
```
- ### 관절 센서용 인터페이스
```
- JointStateInterface : 이 인터페이스는 관절의 현재 위치 또는 속도 또는 힘(토크)를 얻기위한 센서가 있을때 사용된다. joint_state_controller 와 함께 사용한다.

JointStateInterface는 로봇의 순운동학을 계산하기 위헤 tf/tf2 에서 차례로 데이타를 사용하는 로봇의 현재 관절위치를 가져오기 때문에 거의 모든 로봇에서 사용된다.

- ImuSensorInterface : 이 인터페이스는 관절/로봇의 방향, 각속도 및 선형 가속도를 얻는 데 사용되는 IMU 센서가 있을 때 사용됩니다. 이것은 imu_sensor_controller와 함께 사용됩니다.
```

사용중인 모터 및 센서의 유형에 따라 인터페이스를 선택해야 한다. 

예로써 3개의 관절이 있는 로봇을 가정하고 이에 대한 인터페이스를 작성한다.
로봇에는 3개의 액추에이터 JointA, JointB, JointC 가 있으며 각 관절에는 위치센서가 있다. JointA 와 JointB 는 effort command을 수락하고, JointC 는 position command를 수락한다고 가정하자.

먼저 로봇을 위한 catkin 패키지를 생성하자, 그런다음 MyRobot_hardware_interface.h, MyRobot_hardware_interfrace.cpp 를 작성하자

#### [MyRobot_hardware_interface.h]
```
     #include <hardware_interface/joint_state_interface.h>
     #include <hardware_interface/joint_command_interface.h>
     #include <hardware_interface/robot_hw.h>
     #include <joint_limits_interface/joint_limits.h>
     #include <joint_limits_interface/joint_limits_interface.h>
     #include <controller_manager/controller_manage.h>
     #include <boost/scopedptr.hpp>
     #include <ros/ros.h>
     
     class MyRobot : public hardware_interface::RobotHW
     {
           public:
              MyRobot(ros::NodeHandle &nh);
              ~MyRobot();
              
              void init();
              void update(const ros::TimerEvent &e);
              void read();
              void write(ros::Duration elapsed_time);
           protected:
              hardware_interface::JointStateInterface     joint_state_interface_;
              hardware_interface::EffortJointInterface    effort_joint_interface_;
              hardware_interface::PositionJointInterface  position_joint_interface_;
              
              joint_limits_interface::JointLimits                       limits;
              joint_limits_interface::EffortJointSaturationInterface    effortJointSaturationInterface;
              joint_limits_interface::PositionJointSaturationInterface  positionJointSaturationInterface;
              
              double joint_position_[3];
              double joint_velocity_[3];
              double joint_effort_[3];
              double joint_effort_command_[3];
              double joint_position_command_[3];
              
              ros::NodeHandle _nh;
              ros::Timer my_control_loop_;
              ros::Duration elapsed_time_;
              double loop_hz;
              bootst::shared_ptr<controller_mamager::ControllerManager>  controller_manager_;
     };
   ```
   #### [MyRobot_hardware_interface.cpp]
   ```
        #include <package_name/MyRobot_hardware_interface.h>
        
         MyRobot::MyRobot(ros::NodeHandle &nh)
         {
         }
         
         MyRobot::~MyRobot()
         {
         }
        
         void MyRobot::init()
         {
         }
         
         void MyRobot::update(const ros::TimerEvent &e)
         {
         }
         
         void MyRobot::read()
         {
         }
         
         void MyRobot::write(ros::Duration elapsed_time)
         {
         }
        
        
   ```
   
   
   
















 
