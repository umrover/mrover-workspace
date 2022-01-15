#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include<lcm/lcm-cpp.hpp>
#include <rover_msgs/IMUData.hpp>

static int ret;
static int fd;

// creates lcm object
auto lcm_ = new lcm::LCM();

#define BAUD 9600 //115200 for JY61 ,9600 for others

int uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("open %s success!\n",pathname);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 
    return fd;
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
     struct termios newtio,oldtio; 
     if  ( tcgetattr( fd,&oldtio)  !=  0) {  
      perror("SetupSerial 1");
	  printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
      return -1; 
     } 
     bzero( &newtio, sizeof( newtio ) ); 
     newtio.c_cflag  |=  CLOCAL | CREAD;  
     newtio.c_cflag &= ~CSIZE;  
     switch( nBits ) 
     { 
     case 7: 
      newtio.c_cflag |= CS7; 
      break; 
     case 8: 
      newtio.c_cflag |= CS8; 
      break; 
     } 
     switch( nEvent ) 
     { 
     case 'o':
     case 'O': 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag |= PARODD; 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      break; 
     case 'e':
     case 'E': 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag &= ~PARODD; 
      break;
     case 'n':
     case 'N': 
      newtio.c_cflag &= ~PARENB; 
      break;
     default:
      break;
     } 

     /*设置波特率*/ 

switch( nSpeed ) 
     { 
     case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 
     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 
     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 
     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 
     case 460800: 
      cfsetispeed(&newtio, B460800); 
      cfsetospeed(&newtio, B460800); 
      break; 
     default: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 
     } 
     if( nStop == 1 ) 
      newtio.c_cflag &=  ~CSTOPB; 
     else if ( nStop == 2 ) 
      newtio.c_cflag |=  CSTOPB; 
     newtio.c_cc[VTIME]  = 0; 
     newtio.c_cc[VMIN] = 0; 
     tcflush(fd,TCIFLUSH); 

if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
     { 
      perror("com set error"); 
      return -1; 
     } 
     printf("set done!\n"); 
     return 0; 
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    return 0;
}
int send_data(int  fd, char *send_buffer,int length)
{
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;
}
int recv_data(int fd, char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	return length;
}
float a[3],w[3],Angle[3],h[3];
void ParseData(char chr, bool &accelCollected, bool &gyroCollected, bool &angleCollected, bool &magCollected, rover_msgs::IMUData &data)
{
		static char chrBuf[100];
		static unsigned char chrCnt=0;
		signed short sData[4];
		unsigned char i;
		char cTemp=0;
		time_t now;
		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return;
		for (i=0;i<10;i++) cTemp+=chrBuf[i];
		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);chrCnt--;return;}

        // Uncomment commented code for fuller implementation
        // moved code below to outside of ParseData
        // bool accelCollected = false, gyroCollected = false, angleCollected = false, magCollected = false;
		// rover_msgs::IMUData data;

		memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
					// Acceleration pack
                    for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
					time(&now);
					printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
                    
                    // Code for adding accelereometer data to struct
                    data.accel_x_g = a[0];
                    data.accel_y_g = a[1];
                    data.accel_z_g = a[2];

                    accelCollected = true;


					break;
				case 0x52:
                    // Angular velocity pack
					for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
					printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);

                    // Code for adding accelereometer data to struct
                    data.gyro_x_dps = w[0];
                    data.gyro_y_dps = w[1];
                    data.gyro_z_dps = w[2];
                    
                    gyroCollected = true;

					break;
				case 0x53:
                    // Angle Pack
					for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
					printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
                    
                    

                    // For now - set all other imu struct data to 0
                    //-=-=-=-==-=-=-=-=-=-=Remove for actual=-=-=-=-==-=-=-=-=
                    //data.accel_x_g = 0.0;
                    //data.accel_y_g = 0.0;
                    //data.accel_z_g = 0.0;
                    //data.gyro_x_dps = 0.0;
                    //data.gyro_y_dps = 0.0;
                    //data.gyro_z_dps = 0.0;
                    //data.mag_x_uT = 0.0;
                    //data.mag_y_uT = 0.0;
                    //data.mag_z_uT = 0.0;
                    
                    //data.bearing_deg = 0.0;
                    //-=-==-=-=-=-=-=--=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

                    data.roll_rad = Angle[0];
                    data.pitch_rad = Angle[1];
                    data.yaw_rad = Angle[2];
		    if(Angle[2] < 0){
			data.bearing_deg = -Angle[2];
	  	    }
		    else{
		    	data.bearing_deg = (-Angle[2] + 360.0);
	            }	
                    angleCollected = true;

					break;
				case 0x54:
                    //Magnetic Pack
					for (i=0;i<3;i++) h[i] = (float)sData[i];
					printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);

                    // Code for adding magnetometer data to struct
		    // Not using for 6-axis mode
                    data.mag_x_uT = 0; // h[0];
                    data.mag_y_uT = 0; // h[1];
                    data.mag_z_uT = 0; // h[2];

                    magCollected = true;

					break;
		}		


		chrCnt=0;
}

int main(void)
{
    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,"/dev/ttyUSB0");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */ 
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,BAUD,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

	FILE *fp;
	fp = fopen("Record.txt","w");
	
    // Start with none collected
    bool accelCollected = false, gyroCollected = false, angleCollected = false, magCollected = false;
	rover_msgs::IMUData data;
    printf("Starting while loop\n");
    while(1)
    {
        ret = recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) {fprintf(fp,"%2X ",r_buf[i]);ParseData(r_buf[i], accelCollected, gyroCollected, angleCollected, magCollected, data);}
        usleep(1000);

        // If all data collected, publish
        if (accelCollected && gyroCollected && angleCollected && magCollected) {
            //Push
            lcm_->publish("/imu_data", &data);

            //Reset for next collection
            accelCollected = false;
            gyroCollected = false;
            angleCollected = false;
            magCollected = false;
        }

    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
