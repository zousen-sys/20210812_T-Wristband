#include <Hardware.h>
#define BUTTON_NA       0
#define BUTTON_CLICK    1
#define BUTTON_CLICK_2  2
#define BUTTON_CLICK_3  3
#define BUTTON_HOLD_1S  10
#define BUTTON_HOLD_2S  20
#define BUTTON_HOLD_3S  30
#define BUTTON_HOLD_5S  50

#define BUTTON_HOLD_1S_Time  1000
#define BUTTON_HOLD_2S_Time  2000
#define BUTTON_HOLD_3S_Time  3000
#define BUTTON_HOLD_5S_Time  5000


#define BUTTON_RELASE_TIME 250
#define DISLAY_OVER_TIME 20 // 单位：秒，初始设定为20秒屏幕关闭 
#define SLEEP_OVER_TIME  30 // 单位：分， 初始设定为30分钟进入待机

void TouchButton_Setup();
uint8_t TouchButton_Process();
bool TouchButton_DispalyOff_Overtime();
bool TouchButton_DispalyOff_Wakup();
void TouchButton_DispalyOff_Overtime_Reset();
bool TouchButton_Sleep_Overtime();
void TouchButton_Sleep_Overtime_Reset();
void TouchButton_Sleep_Overtime_Set(uint32_t overtime);
void TouchButton_DisplayOff_Overtime_Set(uint32_t overtime);
