#include <pthread.h>
#include "../../include/interface/rt_rc_interface.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <unistd.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <errno.h>  
#include <linux/joystick.h>  
#include <iostream>
static pthread_mutex_t lcm_get_set_mutex =
PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */
// Controller Settings
rc_control_settings rc_control;

#define XBOX_TYPE_BUTTON    0x01  
#define XBOX_TYPE_AXIS      0x02  

#define XBOX_BUTTON_A       0x00  
#define XBOX_BUTTON_B       0x01  
#define XBOX_BUTTON_X       0x02  
#define XBOX_BUTTON_Y       0x03  
#define XBOX_BUTTON_LB      0x04  
#define XBOX_BUTTON_RB      0x05  
// MOCUTE
// #define XBOX_BUTTON_START   0x06  
// #define XBOX_BUTTON_BACK    0x07  
#define XBOX_BUTTON_START   0x07  
#define XBOX_BUTTON_SELECT  0x06

// #define XBOX_BUTTON_HOME    0x08

#define XBOX_BUTTON_LO      0x08    // 左侧控制下压
#define XBOX_BUTTON_RO      0x09    //右侧下压

#define XBOX_BUTTON_ON      0x01  
#define XBOX_BUTTON_OFF     0x00  
//      /\ y
// x    |
// <-----
// 
#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */  
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */  
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */  
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */  
#define XBOX_AXIS_LT        0x02  
#define XBOX_AXIS_RT        0x05  
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */  
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */  

#define XBOX_AXIS_VAL_UP        -32767  
#define XBOX_AXIS_VAL_DOWN      32767  
#define XBOX_AXIS_VAL_LEFT      -32767  
#define XBOX_AXIS_VAL_RIGHT     32767  

#define XBOX_AXIS_VAL_MIN       -32767  
#define XBOX_AXIS_VAL_MAX       32767  
#define XBOX_AXIS_VAL_MID       0x00  

typedef struct xbox_map  
{  
    int     time;  
    int     a;  
    int     b;  
    int     x;  
    int     y;  
    int     lb;  
    int     rb;  
    int     start;  
    int     back;  
    int     select;
    int     home;  
    int     lo;  
    int     ro;  

    int     lx;  
    int     ly;  
    int     rx;  
    int     ry;  
    int     lt;  
    int     rt;  
    int     xx;  
    int     yy;  

}xbox_map_t;  

void get_rc_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}
void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}

int xbox_open(const char *file_name)  
{  
    int xbox_fd;  

    xbox_fd = open(file_name, O_RDONLY);  
    if (xbox_fd < 0)  
    {  
        perror("open");  
        return -1;  
    }  

    return xbox_fd;  
}  

int xbox_map_read(int xbox_fd, xbox_map_t *map)  
{  
    int len, type, number, value;  
    struct js_event js;  

    len = read(xbox_fd, &js, sizeof(struct js_event));  
    if (len < 0)  
    {  
        perror("read");  
        return -1;  
    }  

    type = js.type;  
    number = js.number;  
    value = js.value;  

    map->time = js.time;  

    if (type == JS_EVENT_BUTTON)  
    {  
        switch (number)  
        {  
            case XBOX_BUTTON_A:  
                map->a = value;  
                break;  

            case XBOX_BUTTON_B:  
                map->b = value;  
                break;  

            case XBOX_BUTTON_X:  
                map->x = value;  
                break;  

            case XBOX_BUTTON_Y:  
                map->y = value;  
                break;  

            case XBOX_BUTTON_LB:  
                map->lb = value;  
                break;  

            case XBOX_BUTTON_RB:  
                map->rb = value;  
                break;  

            case XBOX_BUTTON_START:  
                map->start = value;  
                break;  

            case XBOX_BUTTON_SELECT:  
                map->select = value;  
                break;  

            // case XBOX_BUTTON_HOME:  //no
            //     map->home = value;  
            //     break;  

            case XBOX_BUTTON_LO:  
                map->lo = value;  
                break;  

            case XBOX_BUTTON_RO:  
                map->ro = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else if (type == JS_EVENT_AXIS)  
    {  
        switch(number)  
        {  
            case XBOX_AXIS_LX:  
                map->lx = value;  
                break;  

            case XBOX_AXIS_LY:  
                map->ly = value;  
                break;  

            case XBOX_AXIS_RX:  
                map->rx = value;  
                break;  

            case XBOX_AXIS_RY:  
                map->ry = value;  
                break;  

            case XBOX_AXIS_LT:  
                map->lt = value;  
                break;  

            case XBOX_AXIS_RT:  
                map->rt = value;  
                break;  

            case XBOX_AXIS_XX:  //方向键
                map->xx = value;  
                break;  

            case XBOX_AXIS_YY:  
                map->yy = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else  
    {  
        /* Init do nothing */  
    }  

    return len;  
}  

void xbox_close(int xbox_fd)  
{  
    close(xbox_fd);  
    return;  
}  

xbox_map_t map;  
int js_gait=3;
    
void js_complete(int port){
    try
    {
        int len = xbox_map_read(port, &map);  
        if (len < 0) return;
        // printf("joy  lt=%d lx=%d  ly=%d  rt=%d rx=%d  ry=%d start=%d select:%d\n",map.lt,map.lx,map.ly,map.rt,map.rx,map.ry,map.start,map.select);    

        // printf("               rc_control.mode =%f \n",rc_control.mode);
        
        if (map.rt>30000 && map.a)
            rc_control.mode = RC_mode::RECOVERY_STAND;
        if (map.lt>30000 && map.b)
            rc_control.mode = RC_mode::READY;
        if (rc_control.mode!=RC_mode::OFF && map.lt>30000 && map.a)
            rc_control.mode = RC_mode::SITDOWN;

         if (map.rt>30000 && map.b)
            rc_control.mode = RC_mode::OFF;

         if (map.rt>30000 && map.lt>30000)
            rc_control.mode = RC_mode::NMPC;

         if (map.rt>30000 && map.x)
            rc_control.mode = RC_mode::ONLINE_JUMP_OPT;

        if (rc_control.mode!=RC_mode::OFF && map.start)
            rc_control.mode = RC_mode::LOCOMOTION;
        // if ((rc_control.mode==RC_mode::QP_STAND||rc_control.mode==RC_mode::RECOVERY_STAND) && map.lt>30000 && map.x )
        //     rc_control.mode = RC_mode::BACKFLIP;

        if ((rc_control.mode==RC_mode::QP_STAND||rc_control.mode==RC_mode::RECOVERY_STAND) && map.lt>30000 && map.y )
          rc_control.mode = RC_mode::FRONT_JUMP_OPT;

        // std::cout<<"show: "<<rc_control.mode<<" "<<map.rt<<" "<<map.select<<std::endl;
        if ((rc_control.mode == RC_mode::RECOVERY_STAND || rc_control.mode==RC_mode::QP_STAND) && map.rt>30000 && map.x )
            rc_control.mode = RC_mode::SHOWDEMO;

        if (rc_control.mode!=RC_mode::OFF && map.select)
            rc_control.mode = RC_mode::QP_STAND;
        if (rc_control.mode == RC_mode::QP_STAND){
                        rc_control.rpy_des[0] = (float)map.rx/32768;
                        rc_control.rpy_des[1] = -(float)map.ry/32768;
                        rc_control.rpy_des[2] = (float)map.lx/32768;
                        rc_control.height_variation = -(float)map.ly/32768;
                        rc_control.omega_des[0] = 0;
                        rc_control.omega_des[1] = 0;
                        rc_control.omega_des[2] = 0;
            //printf(" rpy_des[0] =%f",rc_control.rpy_des[0] );
            //printf(" rpy_des[1] =%f",rc_control.rpy_des[1] );
            //printf(" rpy_des[2] =%f",rc_control.rpy_des[2] );
            //printf(" height=%f\n",rc_control.height_variation );

            // for jumping motion
            int jump_id = 0; //front jump 默认是
            if (map.yy>30000)jump_id = 2;
            if (map.yy<-30000)jump_id = 1;
            if (map.xx<-30000)jump_id = 3;
            if (map.xx>30000)jump_id = 4;
            if (jump_id>0)
            {
              rc_control.variable[0] = jump_id;
            }
        }
        if (rc_control.mode == RC_mode::LOCOMOTION){
                        if (map.y) js_gait=2;
                        if (map.a) js_gait=3;  //slowing trot
                        if (map.x) js_gait=5;  //trotRunning
                        if (map.b) js_gait=6;  //walking
                        if (map.rb)js_gait=100;//jumping
                        
                        if(map.rt>30000 && map.start) js_gait=4;                                
                        rc_control.variable[0] =js_gait;
                        rc_control.v_des[0] = -(float)map.ly/32768;
                        rc_control.v_des[1] = -1.0 * (float)map.rx/32768;
                        rc_control.v_des[2] = 0;
                        rc_control.omega_des[0] = 0;
                        rc_control.omega_des[1] = 0;//(float)map.ry/32768;//pitch
                        rc_control.omega_des[2] = 1 * (float)map.lx/32768;
                        rc_control.rpy_des[0]=0;
                        rc_control.height_variation = (float)map.ry/32768;
                        if (map.xx<-30000) rc_control.step_height-=0.3;
                        if (map.xx> 30000) rc_control.step_height+=0.3;   //dm

                        // if (map.yy> 30000) rc_control.height_variation = -0.6;
                        // if (map.yy<-30000) rc_control.height_variation = 0;  

        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"Something error! in Xbox controller " << e.what() << '\n';
    }
    
}

int init_xbox_js(){
    int fd = xbox_open("/dev/input/js0");
      
    if (fd>0) {
        memset(&map, 0, sizeof(xbox_map_t));
        printf("xbox joystik open successfull!\n\r");
    }else{
        printf("xbox joystik open failed!\n\r");
    } 
    rc_control.step_height=0.08;
    rc_control.height_variation = 0;
    js_gait=3;
    return fd; 

}
