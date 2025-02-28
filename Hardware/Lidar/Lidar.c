/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-09-05

All rights reserved
***********************************************/

#include "Lidar.h"
#include <string.h>
#include "sys.h"



float Follow_KP =10,Follow_KD =1,Follow_KI = 0.001;	

PointDataProcessDef PointDataProcess[225] ;//����225������
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[225];      //����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����


/**************************************************************************
Function: data_process
Input   : none
Output  : none
�������ܣ����ݴ�����
��ڲ�������
����  ֵ����
**************************************************************************/
//���һ֡���պ���д���
void data_process(void) //���ݴ���
{
	static u8 data_cnt = 0;
	u8 i,m,n;
	u32 distance_sum[8]={0};//2����ľ���͵�����
	LD_Successful_Receive_flag=1;
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//����16����Ŀ�ʼ�Ƕ�
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//����16����Ľ����Ƕ�
	float area_angle[8]={0};//һ֡���ݵ�8��ƽ���Ƕ�
	if((start_angle>350)&&(end_angle<13))//��Ϊһ֡������13�ȣ�����350��10��η�Χ��ӣ����angle�������180��Χ
		end_angle +=360;
	for(m=0;m<8;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/8*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
	}
	for(i=0;i<16;i++)
	{
		switch(i)
		{
			case 0:case 1:
				distance_sum[0] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//0~1��ľ����
			  break;
			case 2:case 3:
				distance_sum[1] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//2~3��ľ����
			  break;
			case 4:case 5:
				distance_sum[2] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//4~5��ľ����
			  break;
			case 6:case 7:
				distance_sum[3] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//6~7��ľ����
			  break;
			case 8:case 9:
				distance_sum[4] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//8~9��ľ����
			  break;
			case 10:case 11:
				distance_sum[5] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//10~11��ľ����
			  break;
			case 12:case 13:
				distance_sum[6] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//12~13��ľ����
			  break;
			case 14:case 15:
				distance_sum[7] +=((u16)Pack_Data.point[i].distance_h<<8)+Pack_Data.point[i].distance_l;//14~15��ľ����
			  break;
			default:break;
								
		}
		  
	}
	for(n=0;n<8;n++)
	{
		PointDataProcess[data_cnt+n].angle = area_angle[n];
	  PointDataProcess[data_cnt+n].distance = distance_sum[n]/2;
	}
	data_cnt +=8;
	if(data_cnt>=225)
	{
		for(i=0;i<225;i++)
		{
			Dataprocess[i].angle=PointDataProcess[i].angle;
			Dataprocess[i].distance=PointDataProcess[i].distance;
		}
		data_cnt = 0;
	}
		
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ��״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//�״�������pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	OutPut=-2.5*Bias-100*(Bias-Last_Bias);//λ��ʽPID������  //
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	return OutPut;                                          	
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ��״�������pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	

float Lidar_follow_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	OutPut=-0.15*Bias-0.1*(Bias-Last_Bias);//λ��ʽPID������  //
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
�������ܣ��״�ת��pid
��ڲ���: ��ǰ�ǶȺ�Ŀ��Ƕ�
����  ֵ�����ת���ٶ�
**************************************************************************/	 	
//�״�ת��pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)		                                 				 //���ƫ��Ļ���
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //����ƫ��
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=-Follow_KP*Bias-Follow_KI*Integral_bias-Follow_KD*(Bias-Last_Bias);	//λ��ʽPID������
	Last_Bias=Bias;                                       					 		//������һ��ƫ��
	if(Turn_Off(Angle_Balance,Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���
	
}




