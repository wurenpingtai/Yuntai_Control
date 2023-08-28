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
/********************************************************************************************************
【函数名】UART(重载)
【功  能】UART的构造函数，阻塞方式打开一个串口设备文件
【参  数】devFile	:表示UART对应的设备文件
【返回值】无
********************************************************************************************************/
UART::UART(const char *OpenFile, int inSpeed, int inBits, int inEvent, int inStop, int inReadLen, int inTimeout)
{
	mFd = open(OpenFile, O_RDWR | O_NOCTTY | O_NDELAY | O_DIRECT);

	if (mFd > 0) {
		if (fcntl(mFd, F_SETFL, 0) < 0) {
			printf("fcntl failed!\n");

		} else {
			printf("fcntl:%d\n", fcntl(mFd, F_SETFL, 0));
		}

	} else {
		printf("Can't open%s\r\n", OpenFile);
		exit(1);
	}

	struct termios newtio, oldtio;

	if (tcgetattr(mFd, &oldtio)  !=  0) {
		perror("Setup Serial");
		exit(1);
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
		exit(1);
	}

	printf("Set uart done\n");

}
/********************************************************************************************************
【函数名】~UART
【功  能】UART的析构函数，释放发送和接收数组空间，关闭串口空间
【参  数】None
【返回值】None
********************************************************************************************************/
UART::~UART()
{
	free(TX_Buffer);
	free(RX_Buffer);
	mflush();
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
/********************************************************************************************************
【函数名】mflush
【功  能】清空输入输出缓存空间
【参  数】None
【返回值】无
********************************************************************************************************/
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
int UART::mwrite(unsigned char *pcnBuf, int inLen)
{
	int i = 0;
	uint32_t tx_len = 0;

	if ((pcnBuf == NULL) || (inLen == 0)) {
		return -1;
	}

	for (; i < inLen; i += tx_len) {
		tx_len = write(mFd, &pcnBuf[i], inLen - i);

		if (tx_len == 0) {
			break;
		}
	}

	return 0;
}
/********************************************************************************************************
【函数名】UART::read
【功  能】往串口设备读取数据
【参  数】pcnBuf ：数据缓冲区
         inLen  ：命令帧的长度
【返回值】返回0表示读取成功；否则读取失败
********************************************************************************************************/
int UART::mread(unsigned char *pcnBuf, int inLen)
{
	unsigned char data;
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
【函数名】Get_Camera_Version
【功  能】获取相机版本号
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Camera_Version(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Protocol_Version
【功  能】获取协议版本号
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Protocol_Version(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x02; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Current_Camera_Working_Mode
【功  能】获取当前相机工作模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Current_Camera_Working_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x03; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Camera_Battery_Level
【功  能】获取相机电量
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Camera_Battery_Level(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x04; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Working_Mode
【功  能】获取云台工作模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Working_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x05; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Genting_Posture_Information
【功  能】获取云台姿态信息
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Genting_Posture_Information(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x05; //执行命令参数
	TX_Buffer[4] = 0x02;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_SD_information
【功  能】获取SD卡信息
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_SD_information(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x06; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Camera_Time
【功  能】获取相机时间
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Camera_Time(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x07; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Get_Camera_Time
【功  能】获取相机当前倍率
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Current_Camera_Magnification(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x08; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Zoom_in_Camera_Magnification
【功  能】放大相机倍率
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Zoom_in_Camera_Magnification(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x08; //执行命令参数
	TX_Buffer[4] = 0x02;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Zoom_out_Camera_Magnification
【功  能】缩小相机倍率
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Zoom_out_Camera_Magnification(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x00; //命令字
	TX_Buffer[3] = 0x08; //执行命令参数
	TX_Buffer[4] = 0x03;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Switch_Camera_Mode
【功  能】切换相机模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Switch_Camera_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_PictureOrVideo_InPreview_Mode
【功  能】在阅览模式下拍照/录像
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Camera_To_Take_PictureOrVideo_In_Preview_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x02; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Switch_Camera_WiFi_ONorOFF
【功  能】开关WiFi
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Switch_Camera_WiFi_ONorOFF(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x03; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_Quick_Photo
【功  能】快捷拍照
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Camera_To_Take_Quick_Photo(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x04; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_To_Take_Quick_Video
【功  能】快捷录像
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Camera_To_Take_Quick_Video(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x05; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Formate_SDCard
【功  能】格式化SD卡
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Formate_SDCard(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x06; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_Color_Mode
【功  能】设置相机彩色模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Camera_Color_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x07; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Camera_BlackWhite_Mode
【功  能】设置相机黑白模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Camera_BlackWhite_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x01; //命令字
	TX_Buffer[3] = 0x07; //执行命令参数
	TX_Buffer[4] = 0x02;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Set_Gimbal_Full_Follow_Mode
【功  能】设置云台全跟随模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Set_Gimbal_Full_Follow_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = 0x00;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_HeadingFollow_PitchLock
【功  能】设置云台航向跟随，俯仰锁定
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_HeadingFollow_PitchLock(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_HeadingFollowTF_GimbalLock
【功  能】设置云台航向俯仰跟随TF，云台跟随
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_HeadingFollowTF_GimbalLock(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = 0x02;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Lock_Mode
【功  能】云台锁定模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Lock_Mode(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x01; //执行命令参数
	TX_Buffer[4] = 0x03;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Pantai_Order_Back_In
【功  能】云台回中模式
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Pantai_Order_Back_In(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x02; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Flip
【功  能】云台命令-翻转
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Flip(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x03; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Calibration
【功  能】云台命令-校准
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Calibration(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x05; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x07; //执行命令参数
	TX_Buffer[4] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimabl_Sensitivity_Mode0_Default
【功  能】云台灵敏度模式0-默认(云台默认)
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimabl_Sensitivity_Mode0_Default(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x04; //执行命令参数
	TX_Buffer[4] = 0x00;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Sensitivity_Mode1_Motion
【功  能】云台灵敏度模式1-运动
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Sensitivity_Mode1_Motion(unsigned char *Buffer)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa; //帧头
	TX_Buffer[1] = 0x06; //总字节数
	TX_Buffer[2] = 0x05; //命令字
	TX_Buffer[3] = 0x04; //执行命令参数
	TX_Buffer[4] = 0x01;
	TX_Buffer[5] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Analog_Joystick_Operation
【功  能】云台模拟摇杆控制
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Analog_Joystick_Operation(unsigned char *Buffer,
		unsigned char Course_MSB, unsigned char Course_LSB,
		unsigned char Pitch_MSB, unsigned char Pitch_LSB)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa;
	TX_Buffer[1] = 0x09;
	TX_Buffer[2] = 0x05;
	TX_Buffer[3] = 0x06;
	TX_Buffer[4] = Course_MSB;
	TX_Buffer[5] = Course_LSB;
	TX_Buffer[6] = Pitch_MSB;
	TX_Buffer[7] = Pitch_LSB;
	TX_Buffer[8] = CRC();

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Absolute_Angle_Control
【功  能】云台绝对角度控制
【参  数】Buffer
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Gimbal_Absolute_Angle_Control(unsigned char *Buffer,
					unsigned char Course_MSB, unsigned char Course_LSB,
					unsigned char Roll_MSB, unsigned char Roll_LSB,
					unsigned char Pitch_MSB, unsigned char Pitch_LSB,
					unsigned char Speed_Control)
{
	memset(TX_Buffer, 0, sizeof(TX_Buffer));//发送数组设置为0
	TX_Buffer[0] = 0xaa;
	TX_Buffer[1] = 0x0c;
	TX_Buffer[2] = 0x05;
	TX_Buffer[3] = 0x05;
	TX_Buffer[4] = Course_MSB;
	TX_Buffer[5] = Course_LSB;
	TX_Buffer[6] = Roll_MSB;
	TX_Buffer[7] = Roll_LSB;
	TX_Buffer[8] = Pitch_MSB;
	TX_Buffer[9] = Pitch_LSB;
	TX_Buffer[10] = Speed_Control;
	TX_Buffer[11] = CRC();
	printf("TX_Buffer:");

	for (int i = 0; i < TX_Buffer[1]; i++) {
		printf(" 0x%x ", TX_Buffer[i]);
	}

	printf("\r\n");

	if (-1 == mwrite(TX_Buffer, TX_Buffer[1])) {
		return -1;//发送失败

	} else {
		int retval = mselect(SelectTime);//设置阻塞时间

		if (retval > 0) {
			memset(RX_Buffer, 0, sizeof(RX_Buffer));
			retval = mread(RX_Buffer, sizeof(RX_Buffer));//读取标志

			if (0 == retval) { //读取成功
				PX4_INFO("ReceiveBuffer:");

				for (int i = 0; i < RX_Buffer[1]; i++) {
					printf(" 0x%02x ", RX_Buffer[i]);
				}

				if (0x00 == RX_Buffer[3]) {
					for (int i = 0; i < RX_Buffer[1]; i++) {
						Buffer[i] = RX_Buffer[i];
					}

					return 0;

				} else if (0x01 == RX_Buffer[3]) {
					return -2;

				} else if (0x02 == RX_Buffer[3]) {
					return -3;

				} else if (0x03 == RX_Buffer[3]) {
					return -4;

				} else if (0x04 == RX_Buffer[3]) {
					return -5;
				}

				PX4_INFO("\n");

			}

		} else if (0 == retval) {
			PX4_INFO("Answer timeout.No data");
			return -6;

		} else if (-1 == retval) {
			PX4_INFO("Failed to set time of mselect");
			return -7;
		}

	}

	return NULL;
}
/********************************************************************************************************
【函数名】Gimbal_Analog_Joystick_Operation_DEC
【功  能】云台模拟摇杆控制-直接输入十进制
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
int UART::Gimbal_Analog_Joystick_Operation_DEC(unsigned char *Buffer, int Course_Speed,
		int Pitch_Speed)//云台模拟摇杆控制-直接输入十进制无需转换
{
	CharToInt Course, Pitch;
	Course.Int = Course_Speed;
	Pitch.Int = Pitch_Speed;
	return Gimbal_Analog_Joystick_Operation(Buffer, Course.Byte[1], Course.Byte[0], Pitch.Byte[1], Pitch.Byte[0]);

}
/********************************************************************************************************
【函数名】Gimbal_Absolute_Angle_Control_DEC
【功  能】云台绝对角度控制-直接输入十进制
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
int UART::Gimbal_Absolute_Angle_Control_DEC(unsigned char *Buffer,
		float Course_Angle, float Roll_Angle, float Pitch_Angle,
		int SpeedControl)//绝对角度控制-直接输入十进制无需转换
{
	CharToInt Course, Roll, Pitch;
	Course_Angle = _Constrain(Course_Angle, 175, -175);
	Roll_Angle = _Constrain(Roll_Angle, 35, -35);
	Pitch_Angle = _Constrain(Pitch_Angle, 85, -85);
	Course.Int = (Course_Angle * 10);
	Roll.Int = (Roll_Angle * 10);
	Pitch.Int = (Pitch_Angle * 10);
	return Gimbal_Absolute_Angle_Control(Buffer,
					     Course.Byte[1], Course.Byte[0],
					     Roll.Byte[1], Roll.Byte[0],
					     Pitch.Byte[1], Pitch.Byte[0],
					     SpeedControl);
}
/********************************************************************************************************
【函数名】Get_Genting_Posture_Information_DEC
【功  能】获取姿态信息，并将其转换成十进制
【参  数】Buffer:接收数组；
	Course_Angle:对地航向角
	Roll_Angle:对地横滚角
	Pitch_Angle:对地俯仰角
【返回值】0：写入成功并获得正确返回报文
       -1：写入失败
       -2：没有0xaa头字节
       -3：没有接收到正确的命令
       -4：输入参数不等于计算总字节数
       -5：校验出来的结果与校验位不相等，校验错误
       -6：应答超时，没有数据
       -7：设置等待时间失败
********************************************************************************************************/
int UART::Get_Genting_Posture_Information_DEC(unsigned char *Buffer)
{
	memset(Buffer, 0, MAX_NUM);//清零数组
	int flag = Get_Genting_Posture_Information(Buffer); //获取姿态姿态角度
	CharToInt Course, Roll, Pitch;

	switch (flag) {
	//0:写入成功并获取数据
	case 0: {
			printf("Posture_Information:");

			for (int i = 0; i < Buffer[1]; i++) {
				printf("0x%2x ", Buffer[i]);
			}

			printf("\r\n");
			Course.Byte[0] = Buffer[3];
			Course.Byte[1] = Buffer[2];

			Roll.Byte[0] = Buffer[5];
			Roll.Byte[1] = Buffer[4];

			Pitch.Byte[0] = Buffer[7];
			Pitch.Byte[1] = Buffer[6];
			printf("Course_Angle:%2f;Roll_Angle:%2f;Pitch_Angle:%2f\r\n",
			       Course.Int / 100.0,
			       Roll.Int / 100.0,
			       Pitch.Int / 100.0);
			return 0;
		}; break;

	//-1：写入失败
	case -1: {
			return -1;
		}; break;

	//-2：没有0xaa头字节
	case -2: {
			return -2;
		}; break;

	//-3：没有接收到正确的命令
	case -3: {
			return -3;
		}; break;

	//-4：输入参数不等于计算总字节数
	case -4: {
			return -4;
		}; break;

	//-5：校验出来的结果与校验位不相等，校验错误
	case -5: {
			return -5;
		}; break;

	//-6：应答超时，没有数据
	case -6: {
			return -6;
		}; break;

	//-7：设置等待时间失败
	case -7: {
			return -7;
		}; break;

	}

	return NULL;
}
/********************************************************************************************************
【函数名】uart_main
【功  能】线程执行主函数
【参  数】None
【返回值】校验码
********************************************************************************************************/
int uart_main(int argc, char *argv[])
{
	printf("ID:%d\r\n", pthread_self());
	UART *mCom = new UART(SerialPort4, BaudRate, DataBits, CheckBits, StopBits, ReadLen, ReadTimeOut);
	unsigned char Buffer[MAX_NUM];
	mCom->Gimbal_Absolute_Angle_Control_DEC(Buffer, -180, 40, -90, 0);
	mCom->Get_Genting_Posture_Information_DEC(Buffer);
	return NULL;
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

