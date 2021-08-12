#include <Hardware.h>
#include <Display.h>
#include <TouchButton.h>
#include <Main_RTC.h>

// Process Mode definition
uint8_t Process_Mode=0;

void setup(){
    Serial.begin(230400);
    Display_Setup();
    Hardware_Setup();
    TouchButton_Setup();
    Process_Mode = PM_RTC;
}

void loop(){
    switch (Process_Mode) {
        case PM_RTC:
            Main_RTC_Process();
            Process_Mode = PM_RTC;
            break;
        case PM_Sensor:

            break;

        default:
            break;
    }
}


