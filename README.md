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
              double joint_effort_command_[2];
              double joint_position_command_;
              
              ros::NodeHandle _nh;
              ros::Timer my_control_loop_;
              ros::Duration elapsed_time_;
              double loop_hz_;
              bootst::shared_ptr<controller_mamager::ControllerManager>  controller_manager_;
     };
   ```
   #### [MyRobot_hardware_interface.cpp]
   ```
        #include <package_name/MyRobot_hardware_interface.h>
        
         MyRobot::MyRobot(ros::NodeHandle &nh) : (nh_(nh)
         {
               init();
               
               controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
               
               loop_hz_ = 10;
               ros::Duration  update_freq = ros::Duration(1.0/loop_hz_);
               
               my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
         }
         
         MyRobot::~MyRobot()
         {
         }
        
         void MyRobot::init()
         {
             // JointA
             // 1. Joint에 대한 joint_state_interface 를 생성한다.
             hardware_interface::JointStateHandle   jointStateHandleA("JointA", &joint_position_[0], 
                                                                                &joint_velocity_[0],
                                                                                &joint_effort_[0]);
             hardware_interface::JointStateHandle   jointStateHandleB("JointB", &joint_position_[1], 
                                                                                &joint_velocity_[1], 
                                                                                &joint_effort_[1]);
             hardware_interface::JointStateHandle   jointStateHandleC("JointC", &joint_position_[2],
                                                                                &joint_velocity_[2],
                                                                                &joint_effort_[2]);
             joint_state_interface_.registerHandle(jointStateHandleA);
             joint_state_interface_.registerHandle(jointStateHandleB);
             joint_state_interface_.registerHandle(jointStateHandleC);
             
             // 2. JointA, B effort command
             hardware_interface::JointHandle   jointEffortHandleA(jointStateHandleA, &joint_effort_command_[0]);
             hardware_interface::JointHandle   jointEffortHandleB(jointStateHandleB, &joint_effort_command_[1]);
             effort_joint_interface_.registerHandle(jointEffortHandleA);
             effort_joint_interface_.registerHandle(jointEffortHandleB);
             
             // 3. JointC position command
             hardware_interface::JointHandle   jointPositionHandleC(jointStateHandleC, &joint_position_command_);
             position_joint_interface_.registerHandle(jointPositionHandleC);
             
             // 4. JointA, B,C joint limits interface
             joint_limits_interface::getJointLimits("JointA", nh_, limits);
             joint_limits_interface::getJointLimits("JointB", nh_, limits);
             joint_limits_interface::getJointLimits("JointC", nh_, limits);
             
             joint_limits_interface::EffortJointSaturationHandle   jointLimitsHandleA(jointEffortHandleA, limits);
             joint_limits_interface::EffortJointSaturationHandle   jointLimitsHandleB(jointEffortHandleB, limits);
             joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
             
             effortJointSaturationInterface.registerHandle(jointLimitsHandleA);
             effortJointSaturationInterface.registerHandle(jointLimitsHandleB);
             positionJointSaturationInterface.registerHandle(jointLimitsHandleC);
             
             // Register all joints interface
             registerInterface(&joint_state_interface_);
             registerInterface(&effort_joint_interface_);
             registerInterface(&position_joint_interface_);
             registerInterface(&effortJointSaturationInterface);
             registerInterface(&positionJointSaturationInterface);
         }
         
         // control loop
         void MyRobot::update(const ros::TimerEvent &e)
         {
             elapsed_time_ = ros::Duration(e.current_real - e.last_real);
             read();
             controller_manager_->update(ros::Time::now(), elapsed_time_);
             write(elapsed_time_);
         }
         
         void MyRobot::read()
         {
             // Write the protocol(I2C, CAN, ros_serial, ros_industrial) used to get the current joint position and 
             // or velocity, effort from robot
             // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i], joint_effort_[i]
         }
         
         void MyRobot::write(ros::Duration elapsed_time)
         {
             //safty
             effortJointSaturationInterface::enforceLimits(elapsed_time); //// enforce limits for JointA and JointB
             positionJointSaturationInterface::enforceLimits(elapsed_time); // enforce limits for JointC
             
             // Write the protocol(I2C, CAN, ros_serial, ros_industrial) used to send the commands to the robot's actuators
             // the output commands need to send are joint_effort_command_[0] for JointA, 
             //                                      joint_effort_command_[1] for JointB,
             //                                      joint_position_command_ for JointC
         }
        
         int main(int argc, char **argv)
         {
             // Init ROS Node
             ros::init(argc, argv, "MyRobot_hardware_interface_node");
             ros::NodeHandle nh;
             
             //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
             ros::MultiThreadedspinner(2);
             
             // Create the object of the robot hardware_interface class and spin the thread. 
             MyRobot robot(nh);
             spinner.spin();
             return 0;
         }
   ```
   
update() 메서드는 단순히 read ( ) 메서드를 호출 하여 현재 관절 상태를 가져옵니다. 
그런 다음 현재 관절 상태가 컨트롤러 관리자로 업데이트되어 오류 ( 현재 - 목표) 를 계산하고 PID 루프를 사용하여  각 관절에 대한 출력 명령을 생성합니다.  그리고 마지막으로 write () 메서드를 호출 하여 출력 명령을  액추에이터/조인트 에 보냅니다 .

위의 코드를 로봇용 하드웨어 인터페이스 및 제어 루프를 작성하기 위한 상용구 코드로 사용할 수 있다. 
 
[참고] https://github.com/SlateRobotics/tr1_hardware_interface
[블로그]https://slaterobotics.medium.com/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e


로봇에 대한 하드웨어 인터페이스 노드를 작성한 후, 로봇을 제어하기 위한 조인트 액추에이터의 컨트롤러 및 조인트 제한을 선언하는 일부 구성 파일을 작성해야 합니다. 
구성 파일을 작성하기 전에 먼저 실제로 제어하려는 대상  ( 관절의 위치, 노력 또는 속도)을 결정해야 합니다. 이 로봇을 예로 들면서 저는 관절 위치 센서만 있다고 말했습니다. 즉, 관절의 위치 피드백만 얻을 수 있다는 뜻입니다. 따라서 로봇의 위치를 제어하기 위한 구성 파일을 작성합니다.

#### [controller.yaml]
```
    MyRobot:
        #Publish all joint states
        joint_update:
            type:joint_state_controller/JointStateController
            publish_rate: 50
            
        JointA_EffortController:                                # Name of the controller
            type:effort_controller/JointPositionController      # JointA used effort interface this controller type is used
            joint:JointA                                        # Name of the joint for which this controller belong to
            pid:{p:100.0, i:10.0, d:1.0}                        # PID values
            
        JointB_EffortController:
            type:effort_controller/JointPositionController
            joint:JointB
            pid:{p:100.0, i:1.0, d:0.0}
            
        JointC_PositionController:
            type:position_controller/JointPositionController
            joint:JointC
             # No PID values defined since this controller simply passes the input position command to the actuators.
 ```

#### [joint_limits.yaml]
```
    joint_limits:
        JointA:
            has_position_limits: true
            min_position: -1.57
            max_position: 1.57
            
            has_velocity_limits:true
            max_velocity:1.5
            
            has_acceleration_limits: false
            max_acceleration:0.0
            
            has_jerk_limits: false
            max_jerk:0.0
            
            has_effort_limits:true
            max_effort:255
            
         JointB:
            has_position_limits: true
            min_position: 0
            max_position: 3.14
            
            has_velocity_limits:true
            max_velocity:1.5
            
            has_acceleration_limits: false
            max_acceleration:0.0
            
            has_jerk_limits: false
            max_jerk:0.0
            
            has_effort_limits:true
            max_effort:255

        JointC:
            has_position_limits: true
            min_position: 0
            max_position: 3.14
            
            has_velocity_limits:false
            max_velocity:1.5
            
            has_acceleration_limits: false
            max_acceleration:0.0
            
            has_jerk_limits: false
            max_jerk:0.0
            
            has_effort_limits:false
            max_effort:0
```
 
 #### [MyRobot_Control.launch]
 ```
    <launch>
        <rosparam file="$(find YOUR_PACKAGE_NAME)/config/controller.yaml" command="load"/>
        <rosparam file="$(find YOUR_PACKAGE_NAME)/config/joint_limits.yaml" command="load/>
        
        <node name="MyRobotHardwareInterface" pkg="YOUR_PACKAGE_NAME" type="MyRobot_hardware_interface_node" output="screen"/>
        
        <node name="robot_state_publisher" pkg="robot_state_publiosher" type="state_publisher"/>
        
        <node name="controller_spawner" pkg="controller_namager" type="spawner" respawn="false" output="screen" 
                args="/MyRobot/joint_update
                      /MyRobot/JointA_EffortController
                      /MyRobot/JointB_EffortController
                      /MyRobot/JointC_PositionController
                    "/>
    </launch>
 ```

 ### 로봇에서 ros_control을 사용하는 데 필요한 사항을 요약하겠습니다.
 ### 1.로봇의 hardware_interface 노드를 작성해야 합니다.
 ### 2. 조인트 인터페이스 및 애플리케이션 및 PID 값 ( 필요한 경우)을 기반으로 조인트에 대한 컨트롤러 유형을 선택하는 구성 파일을 작성합니다(controller.yaml)
 ### 3. 로봇의 관절 한계를 정의하는 구성 파일을 작성하십시오(joint_limits.yaml)
 ### 4. controller_manager를 통해 컨트롤러를 로드하고 시작합니다(launch file)
 ### 5. 토픽 인터페이스를 사용하여 컨트롤러에 입력 목표를 보냅니다.
 #### 예제 로봇을 제어하기 위한 주제 이름입니다.
 #### JointA:  /MyRobot/JointA_EffortController/command
 #### JointB:  /MyRobot/JointB_EffortController/command
 #### JointC:  /MyRobot/JointC_PositionController/command
 
따라서 위의 예제 로봇 코드를 템플릿으로 사용하여 자신의 로봇에 ros_control을 설정할 수 있습니다 
   
   
   
   
   
   
   
   
   
















 
