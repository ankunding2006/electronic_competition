/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"	
/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         5ms外部中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
入口参数：无
返回  值：无				 
**************************************************************************/

volatile int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
volatile int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Bluetooth_Echo_Test();          //蓝牙回显测试
	//static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
	if(GPIO_Pin==MPU6050_EXTI_Pin)
	{  
			Flag_Target=!Flag_Target;
			Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			Encoder_Left=Read_Encoder(3);            					//读取左轮编码器的值，前进为正，后退为负
			Encoder_Right=-Read_Encoder(4);           					//读取右轮编码器的值，前进为正，后退为负
																													//左轮A相接TIM2_CH1,右轮A相接TIM4_CH2,故这里两个编码器的极性相同
			Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//编码器读数转速度（mm/s）                                       					//10ms控制一次
			if(delay_flag==1)
			{
				delay_50++;
				if(delay_50==10) delay_50=0,delay_flag=0,LD_Successful_Receive_flag=0;  		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
			}
			if(Mode==Ultrasonic_Avoid_Mode||Mode==Ultrasonic_Follow_Mode)		
			   Read_Distane();                           					//获取超声波测量距离值
			if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);   //LED闪烁;常规模式 1s改变一次指示灯的状态	
			if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);     //LED常亮;超声波跟随/避障模式	
			Key();                                    		    //扫描按键状态 单击双击可以改变小车运行状态
			Velocity(Encoder_Left,Encoder_Right);    
            Turn_Pwm = Sensor_PID();  			// 获取PID转向值
			Motor_Left=Velocity_Pwm-Turn_Pwm;       //计算左轮电机最终PWM
			Motor_Right=Velocity_Pwm+Turn_Pwm;      //计算右轮电机最终PWM																							//PWM值正数使小车前进，负数使小车后退
			Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
			Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM限幅
		    if(Flag_Stop==1)							
				Set_Pwm(0,0);                            					
			else
				Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器 
			
			// if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起
			// 	Flag_Stop=1;	                           					//如果被拿起就关闭电机
			// if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下
			// 	Flag_Stop=0;	                           					//如果被放下就启动电机
			//if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常 
	}
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制		
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM		
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
//修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
int Velocity(int encoder_left,int encoder_right)
{  
    volatile static float velocity,Encoder_Least,Encoder_bias,Movement=0;
    volatile static float Encoder_Integral=0;
	//   //================遥控前进后退部分====================// 
	// 	if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //如果进入跟随/避障模式,降低速度
	// 	else 											        Target_Velocity = 50;
	// 	if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //收到前进信号
	// 	else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //收到后退信号
	//     else  Movement=Move_X;	
	
   //=============超声波功能（跟随/避障）==================// 
	    if(Mode==Ultrasonic_Follow_Mode&&(Distance>200&&Distance<500)) //跟随
			 Movement=Target_Velocity/Flag_velocity;
		if(Mode==Ultrasonic_Follow_Mode&&Distance<200) 
			 Movement=-Target_Velocity/Flag_velocity;
		
   //================速度PI控制器=====================//	
		Encoder_Least =Target_Velocity*2-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度-测量速度（左右编码器之和） 
		Encoder_bias *= 0.86;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.14;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，控制前进后退
		if(Encoder_Integral>4000)  	Encoder_Integral=4000;             //积分限幅
		if(Encoder_Integral<-4000)	  Encoder_Integral=-4000;            //积分限幅
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制	
		if(Flag_Stop==1) Encoder_Integral=0;//电机关闭后清除积分
		Velocity_Pwm=velocity;
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制 
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
作    者：轮趣科技（东莞）有限公司
**************************************************************************/
// int Turn(float gyro)
// {
// 	 static float Turn_Target,turn,Turn_Amplitude=54;
// 	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可
// 	//===================遥控左右旋转部分=================//
// 	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
// 	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
// 	 else Turn_Target=0;
// 	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
// 	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
//   //===================转向PD控制器=================//
// 	 turn=Turn_Target*Kp/100+gyro*Kd/100+Move_Z;//结合Z轴陀螺仪进行PD控制
// 	 return turn;								 				 //转向环PWM右转为正，左转为负
// }

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    AIN1=0,			AIN2=1; //修改为反方向
  else           			  AIN1=1,			AIN2=0; //修改为反方向
  PWMA=myabs(motor_left);	
  if(motor_right>0)			BIN1=0,			BIN2=1;	//修改为反方向
  else 	        			  BIN1=1,			BIN2=0; //修改为反方向
  PWMB=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN：Input  max：Maximum value  min：Minimum value
Output  : Output
函数功能：限制PWM赋值 
入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
返回  值：限幅后的值
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    tmp=click();					//单击
    if(tmp==1)
    { 
        Flag_Stop=!Flag_Stop;
        // 打印调试信息
		printf("Flag_Stop:%d\r\n", Flag_Stop);     
    }		//单击控制小车的启停
}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle：Car inclination；voltage：Voltage
Output  : 1：abnormal；0：normal
函数功能：异常关闭电机		
入口参数：angle：小车倾角；voltage：电压
返回  值：1：异常  0：正常
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//电池电压低于10V关闭电机
	{	                                                 //倾角大于40度关闭电机
		temp=1;                                          //Flag_Stop置1，即单击控制关闭电机
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
#pragma diag_suppress 550  // 在函数开始时添加
/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
		accel_x=Accel_X/16384;                           //MPU6050初始化为加速度计为±2g，得到的原始数据为16位数据，并且最高位为符号位，
		accel_y=Accel_Y/16384;                           //所以读取到的数据位±32768，对应着2g，所以16384就是数据分辨率，原始数据除以16384得到数据为m/S^2
		accel_z=Accel_Z/16384;
		gyro_x=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，得到原始数据为整幅32768，对应±2000°/s
		gyro_y=Gyro_Y/16.4;                              //所以32768/2000 = 16.4，也可查看手册得到该数据
		if(Way_Angle==2)		  	
		{
			 Pitch= -Kalman_Filter_x(Accel_Angle_x,gyro_x);//卡尔曼滤波，单位为度
			 Roll = -Kalman_Filter_y(Accel_Angle_y,gyro_y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
	}

}
#pragma diag_default 550   // 在函数结束时恢复
/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //条件1，小车接近静止
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //进入第二步
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时不再等待2000ms，返回第一步
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2，小车是在0度附近被拿起
			flag=2; 
	 } 
	 if(flag==2)                                                       //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                        //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                    //检测到小车被拿起
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //防止误检      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是在0度附近的
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                         //检测到小车被放下
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
函数功能：编码器读数转换为速度（mm/s）
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//求出编码器速度=转速*周长
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : none
函数功能：选择小车运行模式
入口参数：encoder_left：左编码器读数  encoder_right：右编码器读数
返回  值：无
**************************************************************************/
u8 Choose(void)
{
	static int count;
	u8 tmp;
    oled_show_once();
	count += myabs(Read_Encoder(4));
	if(count>6&&count<180)	//普通模式
	{
		Mode = 0;
	}
	if(count>180&&count<360)	//超声波避障模式 
	{	
		Mode = 1;
	}
	if(count>360&&count<540)	//超声波跟随模式
	{		
		Mode = 2;
	}
	if(count>540&&count<720)   //雷达避障
	{
		Mode = 3;
	}
	if(count>720&&count<900)   //雷达跟随
	{
		Mode = 4;
	}
	if(count>900&&count<1080)   //雷达走直线
	{
		Mode = 5;
	}
	if(count>1080&&count<1260)   //CCD巡线
	{
		Mode = 6;
	}
	if(count>1260&&count<1440)   //电磁巡线
	{
		Mode = 7;
	}
	if(count>1440)
		Mode=0,count=0;
	tmp=click_N_Double(50);
	if(tmp==1) 
	{
		OLED_Clear();
		return 0;
	}
	else return 1;
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
函数功能：雷达前进避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Avoid(void)
{
	u8 i;
	u8 avoid_Num=0;//需要避障的点
	float Angle_Sum=0;//确认障碍物在哪一方向的变量
	u8 too_close = 0;//判断障碍物是否太近的变量
	if(Mode==Lidar_Avoid_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			if((Dataprocess[i].angle<avoid_Angle1)||(Dataprocess[i].angle>avoid_Angle2))//小车前进方向100度范围
			{		
				if((Dataprocess[i].distance>0)&&(Dataprocess[i].distance<avoid_Distance))//距离小于300mm需要避障
				{
					Distance=Dataprocess[i].distance;
					avoid_Num++;
					if(Dataprocess[i].angle>310) Angle_Sum += (Dataprocess[i].angle-360);//将310~360转化为-50到0
					else if(Dataprocess[i].angle<50) Angle_Sum+= Dataprocess[i].angle;
					if(Dataprocess[i].distance<150)			too_close++;//靠得太近，需要后退
				}
			}
		}
		if(avoid_Num<8)
		{
		  Move_X=avoid_speed;                                           //给小车一个200mm/s的速度，不要大于800
			Move_Z=0;
		}
		else if(avoid_Num>8)
		{
			 Move_X=0;
	    	if(too_close>10) Move_X=-avoid_speed,Move_Z=0;              //靠的太近，后退一点
				else
				{
					if(Angle_Sum>0)      
					{
						Move_Z=-turn_speed;//障碍物靠右，左转
					}
					else   Move_Z=turn_speed; //障碍物靠左，右转
				}					
		}
 }
}

/**************************************************************************
Function: Lidar_Avoid
Input   : none
Output  : none
函数功能：雷达跟随模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Follow(void)
{
	u8 i;
	u8 follow_num=0;                //判断跟随的点
	u16 mini_distance = 65535;      //要跟随的距离，就是最小距离点的距离
	static float angle =0;                 //跟随点的角度
	static float last_angle = 0;           //跟随点的上一个角度
	u8 data_count = 0;
	if(Mode==Lidar_Follow_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		for(i=0;i<225;i++)
		{
			 if((0<Dataprocess[i].distance) && (Dataprocess[i].distance<Follow_distance))//在0~1500mm中选择最近的点来跟随
			 {
				 follow_num++;
				 if(Dataprocess[i].distance<mini_distance)                  //判断出最小距离的点
				 {
					 mini_distance = Dataprocess[i].distance;
					 angle = Dataprocess[i].angle;
					 Distance = mini_distance;                                     //在oled上显示要跟随点的距离
				 }
			 }
	  }
	if(angle>180)
		  angle -= 360;				//0--360度转换成0--180；-180--0（顺时针）
	if(angle-last_angle>10 ||angle-last_angle<-10)	//做一定消抖，波动大于10度的需要做判断
	{
		if(++data_count == 60)		//连续60次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else							//波动小于10度的可以直接认为是有效值
	{
			data_count = 0;	
			last_angle = angle;
	}
	if(follow_num>5) 	
	{
		Move_X=Lidar_follow_PID(mini_distance,300);//这个的距离pid时直接作用在速度环，所以要变小一点(Move的范围在0~800)
		Move_Z=Follow_Turn_PID(angle,0);//转向PID直接作用在转向环
	}
	else
	{
		Move_X = 0;
		Move_Z = 0;
	}
	if(Move_X>60)    Move_X=60;
 }
}
/**************************************************************************
Function: Lidar_Straight
Input   : none
Output  : none
函数功能：雷达走直线模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Straight(void) 
{
	static u16 target_distance=0;
	u8 i;
	u16 current_distance=target_distance;
	static u16 Limit_distance=0;   //雷达最大的探测距离
	if(Mode==Lidar_Straight_Mode&&Flag_Left!=1&&Flag_Right!=1)
	{
		 Move_X=Initial_speed;//给小车一个初始速度
		 for(i=0;i<225;i++)
	  {
		  if((Dataprocess[i].angle>71)&&(Dataprocess[i].angle<74))//取雷达的70到75度范围的点做比较点
		 {
			 if(determine<Limit_time) //在模式转换到Straight模式3秒后确定我们想要的距离
			 {
				 target_distance=Dataprocess[i].distance;
				 Limit_distance=target_distance+200;//比目标距离大200mm,主要避免参照物的消失导致小车快速转向
				 determine++;
				 if(determine==(Limit_time-1)) determine=Limit_time;
			 }
			 if(Dataprocess[i].distance<(Limit_distance))//限制一下雷达的探测距离
			 {
				 current_distance=Dataprocess[i].distance;//确定距离
			   Distance=Dataprocess[i].distance;
			 }
		 }
	 }
	 Move_Z=Distance_Adjust_PID(current_distance,target_distance);//雷达距离pid
	}
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}

/**************************************************************************
Function: CCD_Mode
Input   : none
Output  : none
函数功能：CCD巡线模式运行
入口参数: 无
返回  值：无
**************************************************************************/	 	
void CCD_Mode(void)
{
	static u8 Count_CCD = 0;								//调节CCD控制频率
	if(Mode == CCD_Line_Patrol_Mode && Flag_Left !=1 &&Flag_Right !=1)
	{
		Move_X = tracking_speed;			//CCD巡线速度
		if(++Count_CCD == 4)								//调节控制频率，4*5 = 20ms控制一次
		{
			RD_TSL(); 
            Find_CCD_Zhongzhi();			
            Count_CCD = 0;			
		}
//		else if(Count_CCD>4)  Count_CCD = 0;
	    
		
	}
}

/**************************************************************************
函数功能：CCD模式转向控制  巡线
入口参数：CCD提取的中线 Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int CCD_turn(u8 CCD,float gyro)//转向控制
{
	  float Turn;     
      float Bias,kp=30,Kd=0.12;	  
	  Bias=CCD-64;
	  Turn=Bias*kp+gyro*Kd;
	  return Turn;
}

/**************************************************************************
Function: ELE_Mode
Input   : none
Output  : none
函数功能：电磁巡线模式运行
入口参数: 无
返回  值：无
**************************************************************************/	 	
// void ELE_Mode(void)
// {
// 	if(Mode == ELE_Line_Patrol_Mode && Flag_Left!=1 &&Flag_Right!=1)
// 	{
// 		int Sum = 0;
// 		Sensor_Left = Get_Adc(2);
//		Sensor_Middle = Get_Adc(1);
// 		Sensor_Right = Get_Adc(9);
// 		Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
// 		Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
// //		if(Detect_Barrier() == No_Barrier)		//检测到无障碍物
// 				Move_X=tracking_speed;       //给小车一个大概300mm/s的速度
// //		else									//有障碍物
// //		{
// //			Move_X = 0;
// //		}	
// 	}
	
// }

/**************************************************************************
函数功能：ELE模式转向控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
// int ELE_turn(float gyro)//转向控制
// {
// 	float Turn;     
// 	float Bias,kp=60,Kd=0.2;	  
// 	Bias=Sensor-100;
// 	Turn=Bias*kp+gyro*Kd;
// 	  return Turn;
// }


/**************************************************************************
Function: Select_Zhongzhi
Input   : none
Output  : none
函数功能：小车机械中值的选择
入口参数：无
返回  值：无
**************************************************************************/
void Select_Zhongzhi(void)                   //机械中值选择，避免安装上电磁巡线、CCD巡线装备时小车往前冲的现象
{
	if(Mode == ELE_Line_Patrol_Mode)
		Middle_angle = -9;
	else if(Mode == CCD_Line_Patrol_Mode)
		Middle_angle = -4;
	else   Middle_angle = 0;
}

