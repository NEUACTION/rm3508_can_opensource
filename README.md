# rm3508_can_opensource
# Robomaster RM3508转接板程序开源
### 欢迎关注ACTION实验室！
### 诚邀招商！
![image](https://github.com/NEUACTION/rm3508_can_opensource/blob/master/image/ACTION%E5%B8%A6%E4%BA%8C%E7%BB%B4%E7%A0%81.png)
### 欢迎star!
### B站账号：东北大学Action实验室
### 微信公众号：ACTION机器人实验室

### 工作模式：
#### 3508电机速度闭环 （Driver[i].unitMode = SPEED_CONTROL_MODE;）
#### 3508电机位置闭环 （Driver[i].unitMode = POSITION_CONTROL_MODE;）
#### 3508位置环零点标定 （Driver[i].unitMode = HOMING_MODE;）

### 控制器：
#### 位置-速度双闭环串极PID控制器
#### 速度斜坡输入
#### 速度环死区
#### 位置环死区

#### 本开源项目，通过CAN协议与C620电调进行通信，进而控制RM3508电机。
#### 硬件环境：stm32F405单片机和CAN芯片/C620电调/RM3508电机

其中，使用CAN2连接C620电调CAN接口，CAN1跟主控通信

控制周期：速度环：1ms，位置环：1ms


![image](https://github.com/NEUACTION/rm3508_can_opensource/blob/master/image/IMG_20190814_175125.jpg)


#### 其中包含了控制代码与CAN转接板原理图，仅供参考。
#### 本开源项目归东北大学ACTION实验室所有。

#### 仓库使用方法：
>* 克隆仓库
>* 编译程序
>* 将程序下载到对应的硬件环境中

#### 有关程序的使用，请主要修改以下程序段
```c
for(int i = 0; i < 8; i++)
	{
		Driver[i].status = ENABLE;//使能电机
		Driver[i].encoder.period = 8192;		//RM3508霍尔返回的脉冲以8192为1圈，具体请参考RM3508手册
		
	  if(Motor[i].type == RM_3508)
		{
		//	Driver[i].unitMode = HOMING_MODE;
		  Driver[i].unitMode = POSITION_CONTROL_MODE;//位置环程序
		//  Driver[i].unitMode = SPEED_CONTROL_MODE;//速度环程序，请打开位置环和速度环中的任意一条
			
			Driver[i].velCtrl.kp = VEL_KP_3508;//请到ctrl.h中修改相应的速度环pid参数，以获得理想的运行效果
			Driver[i].velCtrl.ki = VEL_KI_3508;
			Driver[i].velCtrl.maxOutput = CURRENT_MAX_3508;
			Driver[i].velCtrl.desiredVel[MAX_V] = VEL_MAX_3508;
			Driver[i].posCtrl.kd = POS_KD_3508;//请到ctrl.h中修改相应的位置环pid参数，以获得理想的运行效果
			Driver[i].posCtrl.kp = POS_KP_3508;
			Driver[i].homingMode.current = 0.8f;
			
			Driver[i].velCtrl.acc = 1000.0f;//速度上升环斜坡陡度（加速度）
			Driver[i].velCtrl.dec = 1000.0f;//速度下降斜坡陡度（减速度）
			Driver[i].velCtrl.desiredVel[CMD] = 1000.0f;//如果跑速度环，其初始速度为 1000 / 8.192 = 122rps（转子速度）
			Driver[i].posCtrl.acc = Driver[i].velCtrl.dec;
			Driver[i].posCtrl.posVel = 50.0f;
			Driver[i].homingMode.vel = -160.0f;

		}
		else
		{
			break;
		}
	}
```


#### 欢迎在issue中提出任何问题。

#### 本项目使用Apache License 2.0开源协议（详见LICENSE），商用之前，请先联系作者。


### 关于直接跟使用CAN转接板：

#### 硬件
相应的电子元器件，可以直接查看该板子的原理图和PCB图
##### 懒人模式：如果懒到不想看原理图的，有需要电子元件清单的，请直接联系微信号：wxid_6inww79395dy22

#### 软件
请参考在Action_User文件夹下的elmo.h文件，这里面包含了你们可能使用到的一些命令
##### 懒人模式：不想看elmo.h，或者不愿意自己重写通信协议的，在确保CAN1和CAN2连接正常的情况下，在主控测发送VelCtrl(CANx,8192*10)，这样3508应当以10rps的速度旋转。

##### 有问题请移步到issue提出

参考链接： [RM3508/C620 官方手册与demo](https://www.robomaster.com/zh-CN/products/components/general/M3508?position=download#download)
