#ifndef _XBOX_CONTROLLER_H
#define _XBOX_CONTROLLER_H


typedef struct{
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
}st_Controller;

extern void ControllerInit(void);

extern void RC_Analyse(void);
#endif