
#ifndef __ZONGHE__
#define __ZONGHE__

#pragma pack(1)
typedef struct
{
    char head[2];
    char flag;
    int16_t data1;
    int16_t data2;
    int16_t data3;
    int16_t data4;
    char end;
}pi_frame;
#pragma pack()

void *serial(void *arg);
void *serialrecv(void *arg);
void *control(void *arg);

#endif
