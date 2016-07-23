#include <pthread.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include "server.h"

#define PRINT_RX
//#define LOCAL_DEBUG
#define DEV_PATH "/dev/input/event0"
#define BAUD 115200

int16_t x,y,vz,h,p_out,p,i,d;
int count,close_sign;
int sign_up,sign_down,sign_left,sign_right,sign_w,sign_a,sign_s,sign_d;
pi_frame tx_data,rx_data;
pthread_t thr_serial,thr_control,thr_build_txdata,thr_save_to_file;
pthread_mutex_t txdata_mutex,pid_mutex,loop_mutex,print_mutex;
int ack,start_flag,stop_flag;

void *build_txdata(void *arg)
{
    char *check;

    while(1){
        if(close_sign)break;

        pthread_mutex_lock(&pid_mutex);
        pthread_mutex_unlock(&pid_mutex);
        pthread_mutex_lock(&txdata_mutex);
        pthread_mutex_lock(&loop_mutex);

        if(stop_flag){
            tx_data.flag=0x22;
            stop_flag=0;
        }
		else if(start_flag){
            tx_data.flag=0x11;
            start_flag=0;
        }
		if(tx_data.flag==0x01){
            tx_data.data1=x;
			tx_data.data2=y;
			tx_data.data3=vz;
			tx_data.data4=h;
		}
		else{
			pthread_mutex_lock(&pid_mutex);
			tx_data.data1=p_out;
			tx_data.data2=p;
			tx_data.data3=i;
			tx_data.data4=d;
		}

        tx_data.end=0x00;
		for(check=&tx_data.flag;check<&tx_data.end;++check)tx_data.end=(char)(tx_data.end+(*check));

		pthread_mutex_unlock(&txdata_mutex);

		delay(3);
    }

    return (void *)0;
}

void *save_to_file(void *arg)
{
    while(1){
        if(close_sign)break;
    }

    return (void *)0;
}

void *serial(void *arg)
{
	int serial_fd,cnt,len,left;
	char *tx,*rx,*check;

	if (wiringPiSetup () == -1){
   		printf ("Unable to start wiringPi: %s\n", strerror (errno)) ;
   		exit(1);
  	}
	if ((serial_fd = serialOpen ("/dev/ttyAMA0", BAUD)) < 0){
    	printf ("Unable to open serial device: %s\n", strerror (errno)) ;
    	exit(1);
  	}

    left=0;
    len=sizeof(pi_frame)-2;
	tx_data.head[0]=0xff;
    tx_data.head[1]=0x00;

	while(1){
		if(close_sign)break;

		pthread_mutex_lock(&loop_mutex);

		if(left<=0){
            if(stop_flag){
                tx_data.flag=0x22;
                stop_flag=0;
                left=5;
            }
            else if(start_flag){
                tx_data.flag=0x11;
                start_flag=0;
                left=5;
            }
            if(tx_data.flag==0x01){
                tx_data.data1=x;
                tx_data.data2=y;
                tx_data.data3=vz;
                tx_data.data4=h;
                left=0;
            }
            else{
			//pthread_mutex_lock(&pid_mutex);
                tx_data.data1=p_out;
                tx_data.data2=p;
                tx_data.data3=i;
                tx_data.data4=d;
                left=5;
            }
        }
        else{
            --left;
        }

        tx_data.end=0x00;
        for(check=&tx_data.flag;check<&tx_data.end;++check)tx_data.end=(char)(tx_data.end+(*check));

#ifndef LOCAL_DEBUG
        while(1){
            rx_data.head[0]=rx_data.head[1];
            rx_data.head[1]=serialGetchar(serial_fd);
            if(rx_data.head[0]==0xff&&rx_data.head[1]==0x00)break;
        }
        cnt=len;
        rx=&rx_data.flag;
        while(cnt--){
            *rx=serialGetchar(serial_fd);
            ++rx;
        }

        #ifdef PRINT_RX
        for(rx=(char *)&rx_data;rx<=&rx_data.end;++rx)printf("%02x ",*rx);
        printf("\n");
        #endif
#else
        delay(500);
#endif

        for(tx=(char *)&tx_data;tx<=&tx_data.end;++tx)serialPutchar(serial_fd,*tx);
        serialFlush(serial_fd);

#ifdef PRINT_TX
        if(EBUSY!=pthread_mutex_trylock(&print_mutex)){
            for(tx=(char *)&tx_data;tx<=&tx_data.end;++tx)printf("%02x ",*tx);
            printf("\n");
            pthread_mutex_unlock(&print_mutex);
        }
#endif

        if(left==0)tx_data.flag=0x01;

        pthread_mutex_unlock(&loop_mutex);

        usleep(1);
	}

	serialClose(serial_fd);
	pthread_mutex_unlock(&loop_mutex);

	return (void *)0;
}

void *control(void *arg)
{
	while(1)
	{
		if(close_sign)break;

		if(sign_up==1&&y>-150)--y;
		else if(sign_down==1&&y<150)++y;
		else if(sign_up==0&&sign_down==0)y=0;

		if(sign_left==1&&x>-150)--x;
		else if(sign_right==1&&x<150)++x;
		else if(sign_left==0&&sign_right==0)x=0;

		if(sign_a==1&&vz<150)++vz;
		else if(sign_d==1&&vz>-150)--vz;
		else if(sign_a==0&&sign_d==0)vz=0;

		if(sign_w==1&&h<4000)++h;
		else if(sign_s==1&&h>0)--h;

		usleep(20000);
	}

	return (void *)0;
}

int main()
{
	int keyboard_fd,res1,res2;
	struct input_event t;

	keyboard_fd=open(DEV_PATH,O_RDONLY);
	if(keyboard_fd<=0)
	{
		printf("Open keyboard device error!\n");
		exit(1);
	}

	pinMode(7,OUTPUT);
	digitalWrite(7,1);

	close_sign=0;

	pthread_mutex_init(&txdata_mutex,NULL);
	pthread_mutex_init(&pid_mutex,NULL);
	pthread_mutex_init(&loop_mutex,NULL);
	pthread_mutex_init(&print_mutex,NULL);

	res1=pthread_create(&thr_serial,NULL,serial,(void *)0);
	if(res1!=0)
	{
		printf("Can't creat pthread serial!\n");
		exit(1);
	}
	res2=pthread_create(&thr_control,NULL,control,(void *)0);
	if(res2!=0)
	{
		printf("Can't creat pthread keyboard!\n");
		exit(1);
	}
	/*res2=pthread_create(&thr_build_txdata,NULL,build_txdata,(void *)0);
	if(res2!=0)
	{
		printf("Can't creat pthread build_txdata!\n");
		exit(1);
	}*/
	res2=pthread_create(&thr_save_to_file,NULL,save_to_file,(void *)0);
	if(res2!=0)
	{
		printf("Can't creat pthread save_to_file!\n");
		exit(1);
	}

	start_flag=0,stop_flag=0;

	while(1)
	{
		if(read(keyboard_fd,&t,sizeof(t))==sizeof(t))
		{
			if(t.type==EV_KEY)
			{
				if(t.value==0||t.value==1)
				{
					switch(t.code)
					{
						case KEY_W:sign_w=t.value;break;
						case KEY_A:sign_a=t.value;break;
						case KEY_S:sign_s=t.value;break;
						case KEY_D:sign_d=t.value;break;
						case KEY_UP:sign_up=t.value;break;
						case KEY_DOWN:sign_down=t.value;break;
						case KEY_LEFT:sign_left=t.value;break;
						case KEY_RIGHT:sign_right=t.value;break;
						default:break;
					}
				}
				if((t.code==KEY_R||t.code==KEY_Z||t.code==KEY_H)&&t.value==1)
				{
                    pthread_mutex_lock(&print_mutex);
					printf("Please enter p_out and p,i,d:(int,the actual value*100)\n");
					scanf("%d %d %d %d",(int *)&p_out,(int *)&p,(int *)&i,(int *)&d);
					pthread_mutex_unlock(&print_mutex);
					pthread_mutex_lock(&loop_mutex);
					switch(t.code)
					{
						case KEY_R:tx_data.flag=0x02;break;
						case KEY_Z:tx_data.flag=0x03;break;
						case KEY_H:tx_data.flag=0x04;break;
						default:break;
					}
					pthread_mutex_unlock(&loop_mutex);
				}
				else if(t.code==KEY_T&&t.value==1)
				{
					digitalWrite(7,0);
					start_flag=1;
				}
				else if(t.code==KEY_P&&t.value==1)
				{
					digitalWrite(7,1);
					stop_flag=1;
				}
				else if(t.code==KEY_ESC)break;
			}
		}
	}
	close_sign=1;
	printf("Control pthread return number:%d\n",pthread_join(thr_control,NULL));
	printf("Serial pthread return number:%d\n",pthread_join(thr_serial,NULL));
	//printf("Build_txdata pthread return number:%d\n",pthread_join(thr_build_txdata,NULL));
	printf("Save_to_file pthread return number:%d\n",pthread_join(thr_save_to_file,NULL));
	close(keyboard_fd);
	return 0;
}
