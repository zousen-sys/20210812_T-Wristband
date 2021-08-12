#include <Hardware.h>

#define INDEX_RTC       0
#define INDEX_ACCEL     1
#define INDEX_GYRO      2
#define INDEX_COMPASS   3
#define INDEX_EULER     4

bool Display_Setup(void);
void Display_ChargeStatus(bool status);
void Display_Battery(uint8_t battery_percent);
void Display_Sensor(IMU_DATA imu,uint8_t Index);
void Display_BLE_Name(String ble_name);
void Display_RTC(RTC_Date datetime);
void Display_OFF();
void Display_ON(); 
void Display_initial_refresh();

void Display_IMU_Switch(bool status);
void Display_BLEStatus(bool status);

void Display_BLE_ATCommand(String display);
