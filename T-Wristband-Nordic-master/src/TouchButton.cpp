#include <TouchButton.h>

bool pressed = false;
bool relased = true;
uint32_t pressedTime = 0;
uint32_t pressedCounter = 0;
uint32_t releaseTime = 0;
uint32_t ButtonOptput = BUTTON_NA;
uint32_t DisplayOffOverTimer;
uint32_t SleepOverTimer;

bool DisplayOffOverTimeFlag = false;
bool DisplayOffOverTimeFlag_old = true;
bool TouchPassOnce = false;
uint32_t Sleep_Over_Time_Target;
uint32_t DisplayOff_Over_Time_Target;

void TouchButton_Setup() {

    pinMode(TOUCH_PW, OUTPUT);
    pinMode(TP_PIN_PIN, INPUT);
    digitalWrite(TOUCH_PW, HIGH);
    Sleep_Over_Time_Target = (SLEEP_OVER_TIME*1000*60); // 变量 X 1000ms x 60 = 分钟
    DisplayOff_Over_Time_Target = (DISLAY_OVER_TIME*1000); // 变量 X 1000ms = 秒
    SleepOverTimer = millis() + Sleep_Over_Time_Target;
    DisplayOffOverTimer = millis() + DisplayOff_Over_Time_Target;
}

//触控按键的主要处理过程
uint8_t TouchButton_Process() {
    if (digitalRead(TP_PIN_PIN) == HIGH) {  //如果硬件检测到有按键被按下
        relased = false;                    //按键释放状态设置为false
        if (!pressed) {                     //按键按下状态切换
            pressed = true;                 //切换按键的状态  //
            pressedTime = millis();         //记录按键被按下的时间
            ++pressedCounter;               //按键按下次数
        } else {                            //如果按键按下状态已经切换为true，记录按下的时间
            if (millis() - pressedTime > BUTTON_HOLD_1S_Time) ButtonOptput = BUTTON_HOLD_1S;
            if (millis() - pressedTime > BUTTON_HOLD_2S_Time) ButtonOptput = BUTTON_HOLD_2S;
            if (millis() - pressedTime > BUTTON_HOLD_3S_Time) ButtonOptput = BUTTON_HOLD_3S;
            if (millis() - pressedTime > BUTTON_HOLD_5S_Time) ButtonOptput = BUTTON_HOLD_5S;    
        }
    } else {                                //如果硬件检测到没有按键被按下
        pressed = false;                    //初始化按下状态
        if(!relased) {                      //如果按键释放状态为false
            relased = true;                 //初始化释放状态
            releaseTime =  millis();        //初始化释放时间
        } else {                            //如果按键释放状态已经切换为true
            if (millis() - releaseTime > BUTTON_RELASE_TIME) {  //按键释放时间超过0.25秒再做判断
                
                if (ButtonOptput == BUTTON_HOLD_5S) {
                    ButtonOptput = BUTTON_NA;
                    pressedCounter = 0;     //按下次数初始化
                    NVIC_SystemReset();     //系统复位
                    return(BUTTON_HOLD_5S);
                }
                if (ButtonOptput == BUTTON_HOLD_3S) {
                    ButtonOptput = BUTTON_NA;
                    pressedCounter = 0;
                    return(BUTTON_HOLD_3S);
                }
                if (ButtonOptput == BUTTON_HOLD_2S) {
                    ButtonOptput = BUTTON_NA;
                    pressedCounter = 0;
                    return(BUTTON_HOLD_2S);
                }
                if (ButtonOptput == BUTTON_HOLD_1S) {
                    ButtonOptput = BUTTON_NA;
                    pressedCounter = 0;
                    return(BUTTON_HOLD_1S);
                }
                 /* 1.在按键被被释放后且超过0.25后再做处理
                    2.没有对按下多少次作区分处理
                    3.此种情况下返回的是按下多少次，和先前的返回值单位不同 */
                if(pressedCounter) {       
                    DisplayOffOverTimer = millis() + DisplayOff_Over_Time_Target;   //重置显示器关闭时间
                    SleepOverTimer = millis() + Sleep_Over_Time_Target;             //重置系统睡眠时间
                    DisplayOffOverTimeFlag = false;                                 //显示器关闭标志重置
                    ButtonOptput = pressedCounter;                                  //返回值设置
                    pressedCounter = 0;
                    //处理显示器关闭后首次唤醒时的的情况(不认为有按键按下)
                    if (TouchPassOnce) {
                        TouchPassOnce = false;
                        ButtonOptput = 0;
                        pressedCounter = 0;
                    }
                    return(ButtonOptput);
                }
            }
        }
    }
    return(BUTTON_NA);
}

//显示器20s后关闭
bool TouchButton_DispalyOff_Overtime() {    
    if (DisplayOffOverTimer < millis()) {                                       //预先设置的显示器关闭时间+20s<当前时间
        DisplayOffOverTimer = millis() + DisplayOff_Over_Time_Target;           //重置显示器关闭时间
        if(!DisplayOffOverTimeFlag) {
            DisplayOffOverTimeFlag = true;                                      //关闭显示器，关闭标志位置为1
            TouchPassOnce = true;
            return true;
        }
    }
    return false;
}

//如果关闭标志位为0且和上一次保存的为开启状态，输出唤醒请求(每一次按键 DisplayOffOverTimeFlag 初始化)
bool TouchButton_DispalyOff_Wakup() {
    bool output = false;
    if (DisplayOffOverTimeFlag == false) {
        if (DisplayOffOverTimeFlag  != DisplayOffOverTimeFlag_old) { 
            output = true;
        }
    }
    DisplayOffOverTimeFlag_old = DisplayOffOverTimeFlag;
    return output;      /* 返回了一个函数内的局部变量 */
}

void TouchButton_DispalyOff_Overtime_Reset() {
    SleepOverTimer = millis() + Sleep_Over_Time_Target;
    DisplayOffOverTimer = millis() + DisplayOff_Over_Time_Target;
    DisplayOffOverTimeFlag = false;
}

void TouchButton_Sleep_Overtime_Set(uint32_t overtime) {
    Sleep_Over_Time_Target = overtime*1000*60; // 变量 X 1000ms x 60 = 分钟(睡眠的时间使用AT指令重置)
    SleepOverTimer = millis() + Sleep_Over_Time_Target;

}

void TouchButton_DisplayOff_Overtime_Set(uint32_t overtime) {
    DisplayOff_Over_Time_Target = overtime*1000; // 变量 X 1000ms = 秒(关闭显示器的时间使用AT指令重置)
    DisplayOffOverTimer = millis() + DisplayOff_Over_Time_Target;
    DisplayOffOverTimeFlag = false;
}

bool TouchButton_Sleep_Overtime() {
    bool output = false;
    if (SleepOverTimer < millis()) {
        SleepOverTimer = millis() + Sleep_Over_Time_Target;
        output = true;
    }
    return output;
}

void TouchButton_Sleep_Overtime_Reset() {
    SleepOverTimer = millis() + Sleep_Over_Time_Target;
} 

