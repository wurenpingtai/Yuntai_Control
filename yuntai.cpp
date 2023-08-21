/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "yuntai.hpp"
/********************************************************************************************************
【函数名】getBaudrate
【功  能】返回对应波特率宏定义
【参  数】baudrate			波特率大小
【返回值】波特率宏定义
********************************************************************************************************/
static speed_t getBaudrate(int baudrate)
{
	switch (baudrate) {
	case 0: return B0;

	case 50: return B50;

	case 75: return B75;

	case 110: return B110;

	case 134: return B134;

	case 150: return B150;

	case 200: return B200;

	case 300: return B300;

	case 600: return B600;

	case 1200: return B1200;

	case 1800: return B1800;

	case 2400: return B2400;

	case 4800: return B4800;

	case 9600: return B9600;

	case 19200: return B19200;

	case 38400: return B38400;

	case 57600: return B57600;

	case 115200: return B115200;

	case 230400: return B230400;

	case 460800: return B460800;

	case 500000: return B500000;

	case 576000: return B576000;

	case 921600: return B921600;

	case 1000000: return B1000000;

	case 1152000: return B1152000;

	case 1500000: return B1500000;

	case 2000000: return B2000000;

	case 2500000: return B2500000;

	case 3000000: return B3000000;

	case 3500000: return B3500000;

	case 4000000: return B4000000;

	default: return -1;
	}
}
/********************************************************************************************************
【函数名】UART
【功  能】UART的构造函数，阻塞方式打开一个串口设备文件
【参  数】devFile	:表示UART对应的设备文件
【返回值】无
********************************************************************************************************/
UART::UART(const char *devFile)
{
	/*
	    先以非阻塞方式打开一个串口设备文件：
	    O_RDWR：可读可写
	    O_NOCTTY：不以终端设备方式打开
	    O_NDELAY：非阻塞方式读，无数据时直接返回0
	*/
	mFd = open(devFile, O_RDWR | O_NOCTTY | O_NDELAY | O_DIRECT);

	if (mFd > 0) {
		/*恢复串口为阻塞状态*/
		if (fcntl(mFd, F_SETFL, 0) < 0) {
			printf("fcntl failed!\n");

		} else {
			printf("fcntl=%d\n", fcntl(mFd, F_SETFL, 0));
		}

	} else {
		printf("Can't open %s\n", devFile);
		exit(1);
	}
}
UART::~UART()
{
	free(TX_Buffer);
	free(RX_Buffer);
	close(mFd);
}
/********************************************************************************************************
【函数名】UART::cfg
【功  能】配置UART工作参数
【参  数】inSpeed  ：串口波特率
         inBits   ：数据位
         inEvent  ：奇偶校验位
         inStop   ：停止位
         inReadLen: 阻塞方式一次读取字节最大长度
         inTimeout：阻塞超时等待时长，单位：inTimeout*100ms
【返回值】返回0表示配置成功；否则表示配置失败
********************************************************************************************************/
int UART::cfg(int inSpeed, int inBits, int inEvent, int inStop, int inReadLen, int inTimeout)
{
	struct termios newtio, oldtio;

	if (tcgetattr(mFd, &oldtio)  !=  0) {
		perror("Setup Serial");
		return -1;
	}

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch (inBits) {
//设置数据位
	case 7:
		newtio.c_cflag |= CS7;
		break;

	case 8:
		newtio.c_cflag |= CS8;
		break;
	}

//设置奇偶校验位
	switch (inEvent) {
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;

	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;

	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	}

//设置波特率
	cfsetispeed(&newtio, getBaudrate(inSpeed));
	cfsetospeed(&newtio, getBaudrate(inSpeed));

//停止位设置
	if (inStop == 1) {
		newtio.c_cflag &= ~CSTOPB;

	} else if (inStop == 2) {
		newtio.c_cflag |= CSTOPB;
	}

	/* 阻塞读取字节设置：
	    每读取到inReadLen个字节后read函数返回，
	    或者是在接收到不够inReadLen字节时，
	    等待时长超过inTimeout*100ms时函数返回
	 */
	newtio.c_cc[VTIME]  = inTimeout;
	newtio.c_cc[VMIN] 	= inReadLen;
	tcflush(mFd, TCIFLUSH);

	if ((tcsetattr(mFd, TCSANOW, &newtio)) != 0) {
		perror("Set uart error");
		return -1;
	}

	printf("Set uart done\n");
	return 0;
}
int UART::mflush(void)
{
	return tcflush(mFd, TCIOFLUSH);
}
/********************************************************************************************************
【函数名】UART::write
【功  能】往串口设备发送数据
【参  数】pcnBuf ：数据缓冲区
         inLen	：数据缓冲区长度
【返回值】返回0表示写入成功；否则表示写入失败
********************************************************************************************************/
int UART::mwrite(uint8_t *pcnBuf, int inLen)
{
	int i = 0;
	uint32_t tx_len = 0;

	if ((pcnBuf == NULL) || (inLen == 0)) {
		return ERROR;
	}

	for (; i < inLen; i += tx_len) {
		tx_len = write(mFd, &pcnBuf[i], inLen - i);

		if (tx_len == 0) {
			break;
		}
	}

	return OK;
}

/********************************************************************************************************
【函数名】UART::read
【功  能】往串口设备读取数据
【参  数】pcnBuf ：数据缓冲区
         inLen  ：命令帧的长度
【返回值】返回0表示读取成功；否则读取失败
********************************************************************************************************/
int UART::mread(uint8_t *pcnBuf, int inLen)
{
	uint8_t data;
	PX4_INFO("Reading data from SerialPort \n");
	PX4_INFO("pcnBuf:");
	memset(pcnBuf, 0, inLen);
	data = read(mFd, pcnBuf, inLen);

	if (data > 0) {
		tcflush(mFd, TCIFLUSH);
		return 0;

	} else {
		printf("Error read inLen=%d,data=%d\n", inLen, data);
		return -1;
	}
}

/********************************************************************************************************
【函数名】UART::mselect
【功  能】以select机制等待数据
【参  数】inTimeoutMs ：等待时长
【返回值】0:表示等待超时,-1:执行select失败,>0:数据可读
********************************************************************************************************/
int UART::mselect(int inTimeoutMs)
{
	int retval;
	FD_ZERO(&mRd);
	FD_SET(mFd, &mRd);
	mTimeout.tv_sec = inTimeoutMs / 1000;
	mTimeout.tv_usec = (inTimeoutMs % 1000) * 1000;

	retval = select(mFd + 1, &mRd, NULL, NULL, &mTimeout);
	return (retval);
}
/********************************************************************************************************
【函数名】CRC
【功  能】获取发送报文的校验位
【参  数】None
【返回值】校验码
********************************************************************************************************/
unsigned char UART::CRC(void)
{
	unsigned char i, len;
	unsigned char t = 0;
	unsigned char crc = 0x00;
	len = TX_Buffer[1] - 1;

	while (len--) {
		crc ^= TX_Buffer[t++];

		for (i = 8; i > 0; i--) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0xD5;

			} else {
				crc = (crc << 1);
			}
		}
	}

	return crc;
}
/********************************************************************************************************
【函数名】Get_Version
【功  能】获取云台版本信息
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Camera_Version(uint8_t *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x00; //命令字
	mCom->TX_Buffer[3] = 0x01; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC(); //校验码
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, MAX_NUM, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;        //向云台发送获取相机版本失败

	} else {
		int retval = mCom->mselect(1000);//阻塞模式等待1s

		if (retval > 0) {
			//则表示有消息返回
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);

				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return OK;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //等待应答超时则表明没有数据传输
			return -6;

		} else if (-1 == retval) {
			return -7;//执行select等待数据设置失败
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Device_Protocol_Version_Number
【功  能】获取设备信息-获取协议版本号
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Device_Protocol_Version_Number(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x00; //命令字
	mCom->TX_Buffer[3] = 0x02; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mCom->mselect(1000); //阻塞等待1s

		if (retval > 0) { //retval大于0则表明有应答消息
			retval = mCom->mwrite(mCom->RX_Buffer, MAX_NUM);

			if (retval == 0) {//retval等于0则表明数据读取成功
				int length = mCom->RX_Buffer[1];//接收数据中有效的云台消息帧
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) { //应答消息正确
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

			}

		} else if (0 == retval) { //等待应答消息超时则表明没有数据传输
			return -6;

		} else if (-1 == retval) { //执行selece阻塞等待时间设置失败
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Current_Camera_Working_Mode
【功  能】获取设备信息-获取当前相机工作模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Current_Camera_Working_Mode(unsigned char *Buffer)
{
	//创建UART串口类
	UART *mCom = new UART(SerialPort4);
	//初始化发送数组
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x00; //命令字
	mCom->TX_Buffer[3] = 0x03; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC(); //校验码
	//配置串口
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	//判断是否写入成功
	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;//函数执行完若返回-1则表明写入失败

	} else {
		int retval = mCom->mselect(1000); //阻塞等待1s读取

		if (retval > 0) { //retval大于0则表明有数据可读
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);//读取数据

			//数据读取成功
			if (0 == retval) {
				int length = mCom->RX_Buffer[1]; //应答消息中有效云台数据帧
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();//mCom释放
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //无应答消息
			return -6;

		} else if (-1 == retval) { //设置阻塞时间失败
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Camera_Battery_Level
【功  能】获取设备信息-获取相机电量
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Camera_Battery_Level(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4); //创建UART对象
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x04;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;//写入失败

	} else {
		int retval = mCom->mselect(1000); //阻塞模式等待1s

		if (retval > 0) { //有数据可读
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM); //读取数据

			if (0 == retval) { //读取成功
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) { //应答超时
			return -6;

		} else if (-1 == retval) {
			//设置阻塞模式失败
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Gimbal_Working_Mode
【功  能】获取设备信息-云台工作模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Gimbal_Working_Mode(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x05;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;//写入失败

	} else {
		int retval = mCom->mselect(1000); //阻塞等待1s

		if (retval > 0) { //有数据可读
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) { //读取成功
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();//delete mCom
					return 0;
				}

				else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //应答超时
			return -6;

		} else if (-1 == retval) { //设置阻塞时间失败
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Genting_Posture_Information
【功  能】获取设备信息-云台姿态信息(航向、横滚、俯仰)
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Genting_Posture_Information(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x05;
	mCom->TX_Buffer[4] = 0x02;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_SD_information
【功  能】获取设备信息-获取SD卡信息(剩余容量)
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_SD_information(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x06;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Camera_Time
【功  能】获取设备信息-获取相机时间
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Camera_Time(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x07;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_Time
【功  能】获取设备信息-设置相机时间
【参  数】Buffer:存取返回报文;Data1~Data14为设置时间的ASCII码
	如设置相机时间为2020-04-30 16:00:00
	则其ASCII码为：32 30 32 30 30 34 33 30 31 36 30 30 30 30
	将其依次赋予Data1~Data14
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_Time(unsigned char *Buffer, unsigned char Data1, unsigned char Data2, unsigned char Data3,
		    unsigned char Data4, unsigned char Data5, unsigned char Data6, unsigned char Data7,
		    unsigned char Data8, unsigned char Data9, unsigned char Data10, unsigned char Data11,
		    unsigned char Data12, unsigned char Data13, unsigned char Data14)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x14; //20
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x07;
	mCom->TX_Buffer[4] = 0x02;
	mCom->TX_Buffer[5] = Data1;
	mCom->TX_Buffer[6] = Data2;
	mCom->TX_Buffer[7] = Data3;
	mCom->TX_Buffer[8] = Data4;
	mCom->TX_Buffer[9] = Data5;
	mCom->TX_Buffer[10] = Data6;
	mCom->TX_Buffer[11] = Data7;
	mCom->TX_Buffer[12] = Data8;
	mCom->TX_Buffer[13] = Data9;
	mCom->TX_Buffer[14] = Data10;
	mCom->TX_Buffer[15] = Data11;
	mCom->TX_Buffer[16] = Data12;
	mCom->TX_Buffer[17] = Data13;
	mCom->TX_Buffer[18] = Data14;
	mCom->TX_Buffer[19] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Current_Camera_Magnification
【功  能】获取设备信息-获取相机当前倍率
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Get_Current_Camera_Magnification(unsigned char *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x08;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Zoom_in_Camera_Magnification
【功  能】获取设备信息-放大相机倍率
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Zoom_in_Camera_Magnification(unsigned char *Buffer) //获取设备信息-放大相机倍率
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x08;
	mCom->TX_Buffer[4] = 0x02;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Zoom_out_Camera_Magnification
【功  能】获取设备信息-缩小相机倍率
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Zoom_out_Camera_Magnification(unsigned char *Buffer)//获取设备信息-缩小相机倍率
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x00;
	mCom->TX_Buffer[3] = 0x08;
	mCom->TX_Buffer[4] = 0x03;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Switch_Camera_Mode
【功  能】获取设备信息-切换相机模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Switch_Camera_Mode(unsigned char *Buffer)//切换相机模式
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x01;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;

}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_PictureOrVideo_In_Preview_Mode
【功  能】设置相机在预览模式下拍照或录像
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_To_Take_PictureOrVideo_In_Preview_Mode(unsigned char
		*Buffer) //设置相机在预览模式下拍照或录像
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x02;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Switch_Camera_WiFi_ONorOFF
【功  能】开关相机WiFi
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Switch_Camera_WiFi_ONorOFF(unsigned char *Buffer) //开/关相机WiFi
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x03;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_Quick_Photo
【功  能】设置相机快捷拍照
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_To_Take_Quick_Photo(unsigned char *Buffer) //设置相机快捷拍照
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x04;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_Quick_Video
【功  能】设置相机快捷录像
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_To_Take_Quick_Video(unsigned char *Buffer)//设置相机快捷录像
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x05;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Format_SDCard
【功  能】格式化SD卡
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Formate_SDCard(unsigned char *Buffer) //格式化SD卡
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x05;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x06;
	mCom->TX_Buffer[4] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_Color_Mode
【功  能】设置相机彩色模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_Color_Mode(unsigned char *Buffer)//设置相机彩色模式
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x07;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_BlackWhite_Mode
【功  能】设置相机黑白模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Camera_BlackWhite_Mode(unsigned char *Buffer) //设置相机黑白模式
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x01;
	mCom->TX_Buffer[3] = 0x07;
	mCom->TX_Buffer[4] = 0x02;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Gimbal_Full_Follow_Mode
【功  能】设置云台跟随模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Set_Gimbal_Full_Follow_Mode(unsigned char *Buffer) //设置云台跟随模式
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x01;
	mCom->TX_Buffer[4] = 0x00;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_HeadingFollow_PitchLock
【功  能】设置云台航向跟随，俯仰锁定
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_HeadingFollow_PitchLock(unsigned char *Buffer) //设置云台航向跟随，俯仰锁定
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x01;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_HeadingFollowTF_GimbalLock
【功  能】设置云台航向俯仰跟随TF，云台跟随
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_HeadingFollowTF_GimbalLock(unsigned char *Buffer) //设置云台航向俯仰跟随TF，云台跟随
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x01;
	mCom->TX_Buffer[4] = 0x02;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Lock_Mode
【功  能】云台锁定模式
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Lock_Mode(unsigned char *Buffer) //云台锁定模式
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x01;
	mCom->TX_Buffer[4] = 0x03;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Pantai_Order_Back_In
【功  能】云台命令-回中
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Pantai_Order_Back_In(uint8_t *Buffer)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x05; //命令字
	mCom->TX_Buffer[3] = 0x02; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC(); //校验码
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, MAX_NUM, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;        //向云台发送获取相机版本失败

	} else {
		int retval = mCom->mselect(1000);//阻塞模式等待1s

		if (retval > 0) {
			//则表示有消息返回
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);

				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return OK;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //等待应答超时则表明没有数据传输
			return -6;

		} else if (-1 == retval) {
			return -7;//执行select等待数据设置失败
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Flip
【功  能】云台命令-翻转
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Flip(unsigned char *Buffer) //云台命令-翻转
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x05; //命令字
	mCom->TX_Buffer[3] = 0x03; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC(); //校验码
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, MAX_NUM, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;        //向云台发送获取相机版本失败

	} else {
		int retval = mCom->mselect(1000);//阻塞模式等待1s

		if (retval > 0) {
			//则表示有消息返回
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);

				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return OK;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //等待应答超时则表明没有数据传输
			return -6;

		} else if (-1 == retval) {
			return -7;//执行select等待数据设置失败
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Calibration
【功  能】云台命令-校准
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Calibration(unsigned char *Buffer) //云台命令-校准
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa; //帧头
	mCom->TX_Buffer[1] = 0x05; //总字节数
	mCom->TX_Buffer[2] = 0x05; //命令字
	mCom->TX_Buffer[3] = 0x07; //执行命令参数
	mCom->TX_Buffer[4] = mCom->CRC(); //校验码
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, MAX_NUM, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return -1;        //向云台发送获取相机版本失败

	} else {
		int retval = mCom->mselect(1000);//阻塞模式等待1s

		if (retval > 0) {
			//则表示有消息返回
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);

				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					mCom->~UART();
					return OK;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) { //等待应答超时则表明没有数据传输
			return -6;

		} else if (-1 == retval) {
			return -7;//执行select等待数据设置失败
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimabl_Sensitivity_Mode0_Default
【功  能】云台灵敏度模式0-默认(云台默认)
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimabl_Sensitivity_Mode0_Default(unsigned char *Buffer) //云台灵敏度模式0-默认(云台默认)
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x04;
	mCom->TX_Buffer[4] = 0x00;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Sensitivity_Mode1_Motion
【功  能】云台灵敏度模式1-运动
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Sensitivity_Mode1_Motion(unsigned char *Buffer)//云台灵敏度模式1-运动
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x06;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x04;
	mCom->TX_Buffer[4] = 0x01;
	mCom->TX_Buffer[5] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Analog_Joystick_Operation
【功  能】云台模拟摇杆操作
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Analog_Joystick_Operation(unsigned char *Buffer,
				     unsigned char Course_MSB, unsigned char Course_LSB,
				     unsigned char Pitch_MSB, unsigned char Pitch_LSB) //云台模拟摇杆操作
{
	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x09;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x06;
	mCom->TX_Buffer[4] = Course_MSB;
	mCom->TX_Buffer[5] = Course_LSB;
	mCom->TX_Buffer[6] = Pitch_MSB;
	mCom->TX_Buffer[7] = Pitch_LSB;
	mCom->TX_Buffer[8] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);

	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Absolute_Angle_Control
【功  能】云台绝对角度控制
【参  数】Buffer:存取返回报文
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Absolute_Angle_Control(unsigned char *Buffer,
				  unsigned char Course_MSB, unsigned char Course_LSB,
				  unsigned char Roll_MSB, unsigned char Roll_LSB,
				  unsigned char Pitch_MSB, unsigned char Pitch_LSB,
				  unsigned char Speed_Control)  //云台绝对角度控制
{

	UART *mCom = new UART(SerialPort4);
	memset(mCom->TX_Buffer, 0, MAX_NUM);
	mCom->TX_Buffer[0] = 0xaa;
	mCom->TX_Buffer[1] = 0x0c;
	mCom->TX_Buffer[2] = 0x05;
	mCom->TX_Buffer[3] = 0x05;
	mCom->TX_Buffer[4] = Course_MSB;
	mCom->TX_Buffer[5] = Course_LSB;
	mCom->TX_Buffer[6] = Roll_MSB;
	mCom->TX_Buffer[7] = Roll_LSB;
	mCom->TX_Buffer[8] = Pitch_MSB;
	mCom->TX_Buffer[9] = Pitch_LSB;
	mCom->TX_Buffer[10] = Speed_Control;
	mCom->TX_Buffer[11] = mCom->CRC();
	mCom->cfg(Baudrate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);


	if (-1 == mCom->mwrite(mCom->TX_Buffer, mCom->TX_Buffer[1])) {
		return  -1;

	} else {
		int retval = mCom->mselect(1000);

		if (retval > 0) {
			retval = mCom->mread(mCom->RX_Buffer, MAX_NUM);

			if (0 == retval) {
				int length = mCom->RX_Buffer[1];
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < length; i++) {
					printf(" 0x%x ", mCom->RX_Buffer[i]);
				}

				if (0x00 == mCom->RX_Buffer[3]) {
					for (int i = 0; i < length; i++) {
						Buffer[i] = mCom->RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == mCom->RX_Buffer[3]) {
					return -2;

				} else if (0x02 == mCom->RX_Buffer[3]) {
					return -3;

				} else if (0x03 == mCom->RX_Buffer[3]) {
					return -4;

				} else if (0x04 == mCom->RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");
			}

		} else if (0 == retval) {
			return -6;

		} else if (-1 == retval) {
			return -7;
		}
	}

	return NULL;
}
/********************************************************************************************************
【函数名】uart_test_main
【功  能】线程运行函数
【参  数】None
【返回值】None
********************************************************************************************************/
/********************************************************************************************************
【函数名】Gimbal_Absolute_Angle_Control_DEC
【功  能】云台绝对角度控制
【参  数】Buffer:存取返回报文;Course_Angle:设定航向角度;Roll_Angle:设定横滚角度;Pitch_Angle:设定俯仰角度;SpeedControl:设定转向速度
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Absolute_Angle_Control_DEC(unsigned char *Buffer,
				      float Course_Angle, float Roll_Angle, float Pitch_Angle, int SpeedControl)//云台绝对角度控制
{
	FloatToChar Course, Roll, Pitch;
	Course.Int = Course_Angle * 10;
	Roll.Int = Roll_Angle * 10;
	Pitch.Int = Pitch_Angle * 10;
	return Gimbal_Absolute_Angle_Control(Buffer, Course.Byte[1], Course.Byte[0], Roll.Byte[1], Roll.Byte[0], Pitch.Byte[1],
					     Pitch.Byte[0], SpeedControl);
}
/********************************************************************************************************
【函数名】Gimbal_Analog_Joystick_Operation_DEC
【功  能】云台绝对角度控制
【参  数】Buffer:存取返回报文;Course_Angle:设定航向速度;Pitch_Angle:设定俯仰速度
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int Gimbal_Analog_Joystick_Operation_DEC(unsigned char *Buffer, int Course_Speed, int Pitch_Speed)
{
	FloatToChar Course, Pitch;
	Course.Int = Course_Speed;
	Pitch.Int = Pitch_Speed;
	return Gimbal_Analog_Joystick_Operation(Buffer, Course.Byte[1], Course.Byte[0], Pitch.Byte[1], Pitch.Byte[0]);
}
int uart_main(int argc, char *argv[])
{
	uint8_t Buffer[256];
	printf("id:%d\n",  pthread_self());
	Gimbal_Analog_Joystick_Operation_DEC(Buffer, 10, 0);

	switch (Gimbal_Absolute_Angle_Control_DEC(Buffer, 0, 40, 90, 0)) {
	case 0: {
			PX4_INFO("成功写入数据并获得正确应答消息\n");

			for (int i = 0; i < Buffer[1]; i++) {
				printf(" 0x%x ", Buffer[i]);
			}

			printf("\n");
		}; break;

	case -1: PX4_INFO("写入失败\n"); break;

	case -2: PX4_INFO("没有0xaa头字节\n"); break;

	case -3: PX4_INFO("没有接收到正确的命令\n"); break;

	case -4: PX4_INFO("输入参数不等于计算总字节数\n"); break;

	case -5: PX4_INFO("校验出来的结果与校验位不相等，校验错误\n"); break;

	case -6: PX4_INFO("应答超时，没有数据传输\n"); break;

	case -7: PX4_INFO("设置等待时间失败\n"); break;
	}


	return 1;
}
/********************************************************************************************************
【函数名】yuntai_main
【功  能】线程初始化函数
【参  数】None
【返回值】None
********************************************************************************************************/
extern "C" __EXPORT int yuntai_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		px4_task_spawn_cmd("yuntai",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT + 4, //调度优先级
				   2048,//堆栈分配大小
				   uart_main,
				   (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;

	}

	if (!strcmp(argv[1], "stop")) {
		printf("Yuntai_process will be stopped\n");
		return 1;
	}

	return 1;
}
