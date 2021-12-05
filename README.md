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

ros_control 은 아래 그림과 같이 사용자/타사 어플리케이션의 입력으로 관절 상태 데이터와 입력 설정값(목표)를 취하고 
적절한 명령을 출력으로 액추에이터에 보낸다. 


 
