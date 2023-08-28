#ifndef __YUNTAI_HPP
#define __YUNTAI_HPP

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include<termios.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<string.h>
#include<sys/time.h>
#include<sys/types.h>
#include<unistd.h>
#include<pthread.h>
#include<stdio.h>
#include<stdlib.h>
#include<signal.h>
using 	namespace 	time_literals;

#define SerialPort4	"/dev/ttyS6"
#define SerialPort2 	"/dev/ttyS2"
#define MAX_NUM		64
#define BaudRate	115200
#define DataBits	8
#define StopBits	1
#define CheckBits	'N'
#define ReadLen		MAX_NUM
#define ReadTimeOut	10//1s=10*100ms
#define SelectTime	ReadTimeOut*100
#define _Constrain(var,max,min)		((var<min)?min:((var>max)?max:var));


typedef union {
	unsigned char Byte[4];
	int Int;
} CharToInt;


class UART
{
public:
	//打开串口
	UART(const char *cnDevFile);
	UART(const char *OpenFile, int inSpeed, int inBits, int inEvent, int inStop, int inReadLen, int inTimeout);
	//释放发送和接收数组空间
	//清空输入输出缓存
	//关闭串口
	~UART();
	//配置串口相关函数
	int cfg(int inSpeed, int inBits, int inEvent, int inStop, int inReadLen, int inTimeout);
	//写入数据函数
	int mwrite(unsigned char *pcnBuf, int inLen);
	//读取数据函数
	int mread(unsigned char *pcnBuf, int inLen);
	//设置阻塞时间
	int mselect(int inTimeoutMs);
	//清空输出输出缓存寄存器
	int mflush(void);
	//发送报文数组
	unsigned char TX_Buffer[MAX_NUM];
	//接收报文数组
	unsigned char RX_Buffer[MAX_NUM];
	//计算发送报文校验码
	unsigned char CRC(void);
	//获取相机版本号
	int Get_Camera_Version(unsigned char *Buffer);
	//获取协议版本号
	int Get_Protocol_Version(unsigned char *Buffer);
	//获取当前相机工作模式
	int Get_Current_Camera_Working_Mode(unsigned char *Buffer);
	//获取相机电量
	int Get_Camera_Battery_Level(unsigned char *Buffer);
	//获取云台工作模式
	int Gimbal_Working_Mode(unsigned char *Buffer);
	//获取云台姿态信息
	int Get_Genting_Posture_Information(unsigned char *Buffer);
	//获取SD卡信息
	int Get_SD_information(unsigned char *Buffer);
	//获取设备信息-获取相机时间
	int Get_Camera_Time(unsigned char *Buffer);
	//获取设备信息-获取相机当前倍率
	int Get_Current_Camera_Magnification(unsigned char *Buffer);
	//获取设备信息-放大相机倍率
	int Zoom_in_Camera_Magnification(unsigned char *Buffer);
	//获取设备信息-缩小相机倍率
	int Zoom_out_Camera_Magnification(unsigned char *Buffer);
	//切换相机模式
	int Switch_Camera_Mode(unsigned char *Buffer);
	//设置相机在预览模式下拍照或录像
	int Set_Camera_To_Take_PictureOrVideo_In_Preview_Mode(unsigned char
			*Buffer);
	//开/关相机WiFi
	int Switch_Camera_WiFi_ONorOFF(unsigned char *Buffer);
	//设置相机快捷拍照
	int Set_Camera_To_Take_Quick_Photo(unsigned char *Buffer);
	//设置相机快捷录像
	int Set_Camera_To_Take_Quick_Video(unsigned char *Buffer);
	//格式化SD卡
	int Formate_SDCard(unsigned char *Buffer);
	//设置相机彩色模式
	int Set_Camera_Color_Mode(unsigned char *Buffer);
	//设置相机黑白模式
	int Set_Camera_BlackWhite_Mode(unsigned char *Buffer);
	//设置云台跟随模式
	int Set_Gimbal_Full_Follow_Mode(unsigned char *Buffer);
	//设置云台航向跟随，俯仰锁定
	int Gimbal_HeadingFollow_PitchLock(unsigned char *Buffer);
	//设置云台航向俯仰跟随TF，云台跟随
	int Gimbal_HeadingFollowTF_GimbalLock(unsigned char *Buffer);
	//云台锁定模式
	int Gimbal_Lock_Mode(unsigned char *Buffer);
	//云台命令-回中
	int Pantai_Order_Back_In(unsigned char *Buffer);
	//云台命令-翻转
	int Gimbal_Flip(unsigned char *Buffer);
	//云台命令-校准
	int Gimbal_Calibration(unsigned char *Buffer);
	//云台灵敏度模式0-默认(云台默认)
	int Gimabl_Sensitivity_Mode0_Default(unsigned char *Buffer);
	//云台灵敏度模式1-运动
	int Gimbal_Sensitivity_Mode1_Motion(unsigned char *Buffer);
	//云台模拟摇杆操作
	int Gimbal_Analog_Joystick_Operation(unsigned char *Buffer,
					     unsigned char Course_MSB, unsigned char Course_LSB,
					     unsigned char Pitch_MSB, unsigned char Pitch_LSB);
	//云台绝对角度控制
	int Gimbal_Absolute_Angle_Control(unsigned char *Buffer,
					  unsigned char Course_MSB, unsigned char Course_LSB,
					  unsigned char Roll_MSB, unsigned char Roll_LSB,
					  unsigned char Pitch_MSB, unsigned char Pitch_LSB,
					  unsigned char Speed_Control);
	//云台模拟摇杆控制-直接输入十进制无需转换
	int Gimbal_Analog_Joystick_Operation_DEC(unsigned char *Buffer,
			int Course_Speed,
			int Pitch_Speed);
	//绝对角度控制-直接输入十进制无需转换
	int Gimbal_Absolute_Angle_Control_DEC(unsigned char *Buffer,
					      float Course_Angle,
					      float Roll_Angle,
					      float Pitch_Angle,
					      int SpeedControl);
	int Get_Genting_Posture_Information_DEC(unsigned char *Buffer);

private:
	int mFd;
	fd_set mRd;
	struct timeval mTimeout;


};






#endif
