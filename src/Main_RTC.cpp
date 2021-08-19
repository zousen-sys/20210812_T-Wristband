#include <Arduino.h>
#include <Main_RTC.h>
#define MESSAGE_DELAY_TIME 1000

void Main_RTC_Setup(){
    
}

uint8_t Main_RTC_Process(){
    bool initail = true;
    bool Message_Duty = false;
    bool ATCommand = false;
    uint8_t Display_Index = INDEX_RTC;
    Display_initial_refresh();
    while(1) {
        //触控按键处理
        if (TouchButton_DispalyOff_Overtime()) {
            Display_OFF();
        }

        if (TouchButton_DispalyOff_Wakup()) {
            Display_ON();
        }
         
        switch (TouchButton_Process()) {
            case BUTTON_CLICK:                  //检测到一次按键
                if(Display_Index < INDEX_EULER) {
                    ++Display_Index;
                    Hardware_Timer_Reset(TIMER_ONE,10);     //当前页面的显示周期10ms，刷新一次
                }else{
                    Display_Index = INDEX_RTC;
                    Hardware_Timer_Reset(TIMER_ONE,10);
                }
                Display_initial_refresh();
                initail = true;
                break;

            case BUTTON_CLICK_2:                //检测到连续的两次按键
                break;
        
            case BUTTON_CLICK_3:                //检测到连续的三次按键
                break;

            case BUTTON_HOLD_1S:
            case BUTTON_HOLD_2S:
            case BUTTON_HOLD_3S:
                if(Hardware_IMU_Get_Enabled()) 
                    Hardware_IMU_Set_Enabled(false);                    //IMU失能，是否需要初始化屏幕显示？
                else 
                    Hardware_IMU_Set_Enabled(true);                     //IMU使能，唤醒传感器(该处设置取反)

                Display_IMU_Switch(Hardware_IMU_Get_Enabled());         //显示IMU开关页面
                Hardware_Timer_Reset(TIMER_TWO,MESSAGE_DELAY_TIME);     //设置当前页面的显示周期
                Message_Duty = true;                                    //页面显示中标志
                break;

            case BUTTON_HOLD_5S:
                break;
        
            default:
                break;
        }
        
        // 九轴传感器数据解析 及 侦测是否手环停滞不动
        if(Hardware_BLE_Send_IMU()) {
            Hardware_IMU_Set_Enabled(false);
            Display_IMU_Switch(false);      //开关状态是否需要显示？
            Hardware_Timer_Reset(TIMER_TWO,MESSAGE_DELAY_TIME);
            TouchButton_DispalyOff_Overtime_Reset(); 
            Display_ON();
            Message_Duty = true;
        }
        //如果设备连接上且使能，睡眠时间重置
        if(Hardware_Get_BLE_Statu() && Hardware_IMU_Get_Enabled()) {
            TouchButton_Sleep_Overtime_Reset();
        }
        //进入睡眠模式
        if(TouchButton_Sleep_Overtime()) {
            Display_OFF();
            Hardware_Set_SystemSleep();
        }
        //首次进入执行
        if(initail) {
            
        }
        //页面显示周期中
        if(Message_Duty) {  
            if(Hardware_Timer_Refresh(TIMER_TWO, MESSAGE_DELAY_TIME)) { //1s时间到达
                Message_Duty = false;
                Display_initial_refresh();
                initail = 1;
            }
        }
        
        //每秒更新 RTC 日期/时间 及 电池电量..等显示
        if (!Message_Duty) {
            switch((Display_Index)) {
                case INDEX_RTC:
                    if (Hardware_Timer_Refresh(TIMER_ONE,1000) || initail) {
                        Display_RTC(Hardware_Get_RTC());
                        Display_BLE_Name(Hardware_Get_BLE_Name());
                        Display_Battery(Hardware_Get_Battery_Percent());
                    }
                    break;

                case INDEX_ACCEL:
                case INDEX_GYRO:
                case INDEX_COMPASS:
                case INDEX_EULER:
                    if (Hardware_Timer_Refresh(TIMER_ONE,100) || initail) {
                        Display_Sensor(Hardware_Get_IMU(),Display_Index);
                        Display_Battery(Hardware_Get_Battery_Percent());
                    }
                    break;

                default:
                    break;
            }

            //更新蓝芽显示图标
            Display_BLEStatus(Hardware_Get_BLE_Statu());
            //更新锂电池充电状态显示图标
            if(Hardware_Get_ChargeIndication() || initail) {
                Display_ChargeStatus(Hardware_Get_ChargeStatu());
            }
        }

        //BLE UART AT Command 处理
        switch (Hardware_BLE_ATCommand_Decode()) {
            case ATCMD_RESET:
                break;
            case ATCMD_DATE:
                Display_BLE_ATCommand("RTC Date/Time modified");
                ATCommand = true;
                break;

            case ATCMD_SYS_DISPLAY_OFF_TIME:
                Display_BLE_ATCommand("Display Off Time modified");
                ATCommand = true;
                break;

            case ATCMD_SYS_SLEEP_TIME:
                Display_BLE_ATCommand("System Sleep Time modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_STOPMOVE_DEFFERENT:
                Display_BLE_ATCommand("StopMove Defference modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_STOPMOVE_OVERTIME:
                Display_BLE_ATCommand("StopMove OverTime modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_ACCEL_FSR:
                Display_BLE_ATCommand("Accel FSR modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_GYRO_FSR:
                Display_BLE_ATCommand("Gyro FSR modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_AG_LPF:
                Display_BLE_ATCommand("Accel/Gyro LPF modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_SAMPLERATE:
                Display_BLE_ATCommand("IMU SampleRate modified");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_3D:
                Display_BLE_ATCommand("IMU Mode switch to Accel");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_GYRO:
                Display_BLE_ATCommand("IMU Mode switch to Gyro");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_MAG:
                Display_BLE_ATCommand("IMU Mode switch to Magent");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_6D:
                Display_BLE_ATCommand("IMU Mode switch to 6D");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_9D:
                Display_BLE_ATCommand("IMU Mode switch to 9D");
                ATCommand = true;
                break;

            case ATCMD_IMU_MODE_ALL:
                Display_BLE_ATCommand("IMU Mode switch to 9D+4Q");
                ATCommand = true;
                break;
            
            case ATCMD_IMU_ON:
                Hardware_IMU_Set_Enabled(true);
                Display_IMU_Switch(true);
                ATCommand = true;
                break;

            case ATCMD_IMU_OFF:
                Hardware_IMU_Set_Enabled(false);
                Display_IMU_Switch(false);
                ATCommand = true;
                break;

            default:
                break;
        }
        if(ATCommand) {
            ATCommand = false;
            Hardware_Timer_Reset(TIMER_TWO,MESSAGE_DELAY_TIME);
            TouchButton_DispalyOff_Overtime_Reset();
            Display_ON();
            Message_Duty = true;
        }
        initail = false;
    }
}
