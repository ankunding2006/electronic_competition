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
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/

#include "usart3.h"

void Bluetooth_Echo_Test(void)
{
	static uint8_t test_msg[] = "Bluetooth Test: Hello from STM32!\r\n";
	HAL_UART_Transmit(&huart3, test_msg, sizeof(test_msg) - 1, 100);
}

u8 Usart2_Receive_buf[1];									 // 串口3接收中断数据存放的缓冲区
u8 Usart2_Receive;											 // 从串口3读取的数据
u8 Usart3_Receive_buf[1];									 // 串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;											 // 从串口3读取的数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) // 接收回调函数
{

	if (UartHandle->Instance == USART3)
	{
		/*******************测试代码***********************/
		// if (UartHandle->Instance == USART3)
		// {
		// 	// 立即回显收到的字符
		// 	HAL_UART_Transmit(&huart3, Usart3_Receive_buf, 1, 10);
		// 	PID_Send = 1;
		// }
		/**************************************************/

		static int uart_receive = 0; // 蓝牙接收相关变量
		static u8 Flag_PID, i, j, Receive[30];
		static float Data;
		uart_receive = Usart3_Receive_buf[0];
		Usart3_Receive = uart_receive;
		if (Usart3_Receive == 0x31)
			Flag_Stop = !Flag_Stop;
		if (Usart3_Receive == 0x7B)
			Flag_PID = 1; // APP参数指令起始位
		if (Usart3_Receive == 0x7D)
			Flag_PID = 2; // APP参数指令停止位

		if (Flag_PID == 1) // 采集数据
		{
			Receive[i] = Usart3_Receive;
			i++;
		}
		if (Flag_PID == 2) // 分析数据
		{
			if (Receive[3] == 0x50)
				PID_Send = 1;
			else if (Receive[1] != 0x23)
			{
				for (j = i; j >= 4; j--)
				{
					Data += (Receive[j - 1] - 48) * pow(10, i - j);
				}
				PID_Send = 1;
				switch (Receive[1])
				{
				case 0x30:
					Velocity_Kp = Data;
					break;
				case 0x31:
					Velocity_Ki = Data;
					break;
				case 0x32:
					Sensor_Kp = Data;
					break;
				case 0x33:
					Sensor_KI = Data;
					break;
				case 0x34:
					Sensor_Kd = Data;
					break; // 预留
				case 0x35:
					ZoomRatio = Data;
					break; // 预留
				case 0x36:
					Target_Velocity = Data;
					Update_Base_Velocity(Target_Velocity); // 更新基准速度
					break;								   // 预留
				case 0x37:
					break; // 预留
				case 0x38:
					break; // 预留
				}
			}
			Flag_PID = 0;
			i = 0;
			j = 0;
			Data = 0;
			memset(Receive, 0, sizeof(u8) * 50); // 数组清零
		}

		HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf)); // 串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
	}
	// else if(UartHandle->Instance == USART2)
	// {
	// 	static u8 state = 0;//状态位
	// 	static u8 crc_sum = 0;//校验和
	// 	static u8 cnt = 0;//用于一帧16个点的计数
	// 	u8 temp_data;
	// 	temp_data=Usart2_Receive_buf[0];
	// 	switch(state)
	// 	{
	// 		case 0:
	// 			if(temp_data == HEADER_0)//头固定
	// 			{
	// 				Pack_Data.header_0= temp_data;
	// 				state++;
	// 				//校验
	// 				crc_sum += temp_data;
	// 			} else state = 0,crc_sum = 0;
	// 			break;
	// 		case 1:
	// 			if(temp_data == HEADER_1)//头固定
	// 			{
	// 				Pack_Data.header_1 = temp_data;
	// 				state++;
	// 				crc_sum += temp_data;
	// 			} else state = 0,crc_sum = 0;
	// 			break;
	// 		case 2:
	// 			if(temp_data == Length_)//字长固定
	// 			{
	// 				Pack_Data.ver_len = temp_data;
	// 				state++;
	// 				crc_sum += temp_data;
	// 			} else state = 0,crc_sum = 0;
	// 			break;
	// 		case 3:
	// 			Pack_Data.speed_h = temp_data;//速度高八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;
	// 		case 4:
	// 			Pack_Data.speed_l = temp_data;//速度低八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;
	// 		case 5:
	// 			Pack_Data.start_angle_h = temp_data;//开始角度高八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;
	// 		case 6:
	// 			Pack_Data.start_angle_l = temp_data;//开始角度低八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;

	// 		case 7:case 10:case 13:case 16:
	// 		case 19:case 22:case 25:case 28:
	// 		case 31:case 34:case 37:case 40:
	// 		case 43:case 46:case 49:case 52:
	// 			Pack_Data.point[cnt].distance_h = temp_data;//16个点的距离数据，高字节
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;

	// 		case 8:case 11:case 14:case 17:
	// 		case 20:case 23:case 26:case 29:
	// 		case 32:case 35:case 38:case 41:
	// 		case 44:case 47:case 50:case 53:
	// 			Pack_Data.point[cnt].distance_l = temp_data;//16个点的距离数据，低字节
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;

	// 		case 9:case 12:case 15:case 18:
	// 		case 21:case 24:case 27:case 30:
	// 		case 33:case 36:case 39:case 42:
	// 		case 45:case 48:case 51:case 54:
	// 			Pack_Data.point[cnt].Strong = temp_data;//16个点的强度数据
	// 			state++;
	// 			crc_sum += temp_data;
	// 			cnt++;
	// 			break;

	// 		case 55:
	// 			Pack_Data.end_angle_h = temp_data;//结束角度的高八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;
	// 		case 56:
	// 			Pack_Data.end_angle_l = temp_data;//结束角度的低八位
	// 			state++;
	// 			crc_sum += temp_data;
	// 			break;
	// 		case 57:
	// 			Pack_Data.crc = temp_data;//校验
	// 			state = 0;
	// 			cnt = 0;
	// 			if(crc_sum == Pack_Data.crc)
	// 			{
	// 				data_process();//数据处理，校验正确不断刷新存储的数据
	// 			}
	// 			else
	// 			{
	// 				memset(&Pack_Data,0,sizeof(Pack_Data));//清零
	// 			}
	// 			crc_sum = 0;//校验和清零
	// 			break;
	// 		default: break;
	//   }
	//  HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//开启串口2接收中断
	// }
}
