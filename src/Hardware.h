#include <pcf8563.h>
#include <SoftSPI.h>
#include <SerialFlash.h>
#include <SparkFunMPU9250-DMP.h>

#define SOFTWARE_VERSION  "0.01"
#define MANUFACTURER_NAME "Johnson Health Tech"
#define MODEL_NAME "Smart Wristband"


#define SPI_FLASH_MOSI  16
#define SPI_FLASH_MISO  11
#define SPI_FLASH_SCLK  17
#define SPI_FLASH_CS    30

#define CHARGE_PIN      27

#define TFT_MOSI        13
#define TFT_MISO        14
#define TFT_SCLK        12
#define TFT_DC          2
#define TFT_RST         3
#define TFT_CS          4
#define TFT_BL          7

#define RTC_INT_PIN     15

#define RDYM_PIN        19
#define INT2_PIN_DRDY   22
#define INT1_PIN_THS    23
#define INTM_PIN_THS    24

#define TP_PIN_PIN      5
#define TOUCH_PW        29

#define TIMER_ONE           0
#define TIMER_TWO           1
#define TIMER_THREE         2
#define TIMER_FOUR          3
#define TIMER_FIVE          4
#define TIMER_HTTPUPDATE    9

#define IMU_STOPMOVE_OVERTIME 5     // 单位分钟   
#define IMU_STOPMOVE_DEFFERENCE 400  // 单位 mg (0.001g)zyz加速传感的移动差值


#define ATCMD_STR_RESET                     "RESET"
#define ATCMD_STR_SLEEP                     "SLEEP"
#define ATCMD_STR_DATE                      "DATE"

#define ATCMD_STR_SYS_DISPLAY_OFF_TIME      "DIS-OFF-TIME"
#define ATCMD_STR_SYS_SLEEP_TIME            "SLEEP-TIME"


#define ATCMD_STR_IMU_ON                    "IMU-ON"
#define ATCMD_STR_IMU_OFF                   "IMU-OFF"

#define ATCMD_STR_IMU_MODE_ALL              "IMU-MODE-ALL"
#define ATCMD_STR_IMU_MODE_3D               "IMU-MODE-ACCEL"
#define ATCMD_STR_IMU_MODE_GYRO             "IMU-MODE-GYRO"
#define ATCMD_STR_IMU_MODE_MAG              "IMU-MODE-MAG"
#define ATCMD_STR_IMU_MODE_4Q               "IMU-MODE-4Q"
#define ATCMD_STR_IMU_MODE_6D               "IMU-MODE-6D"
#define ATCMD_STR_IMU_MODE_9D               "IMU-MODE-9D"


#define ATCMD_STR_IMU_STOPMOVE_DEFFERENT    "IMU-NOMOVE-DEFF"
#define ATCMD_STR_IMU_STOPMOVE_OVERTIME     "IMU-NOMOVE-OVERTIME"

#define ATCMD_STR_IMU_ACCEL_FSR             "IMU-ACCEL-FSR"
#define ATCMD_STR_IMU_GYRO_FSR              "IMU-GYRO-FSR"
#define ATCMD_STR_IMU_AG_LPF                "IMU-AG-LPF"
#define ATCMD_STR_IMU_SAMPLERATE            "IMU-SAMPLERATE"
#define ATCMD_STR_IMU_AG_SAMPLERATE         "IMU-AG-SAMPLERATE"
#define ATCMD_STR_IMU_MAG_SAMPLERATE        "IMU-MAG-SAMPLERATE"
#define ATCMD_STR_IMU_DMP_SAMPLERATE        "IMU-DMP-SAMPLERATE"


#define ATCMD_NA                        0
#define ATCMD_RESET                     0x01
#define ATCMD_SLEEP                     0x02
#define ATCMD_DATE                      0x03

#define ATCMD_SYS_DISPLAY_OFF_TIME      0x10
#define ATCMD_SYS_SLEEP_TIME            0x11


#define ATCMD_IMU_ON                    0x20
#define ATCMD_IMU_OFF                   0x21
#define ATCMD_IMU_STOPMOVE_DEFFERENT    0x22
#define ATCMD_IMU_STOPMOVE_OVERTIME     0x23

#define ATCMD_IMU_ACCEL_FSR             0x30
#define ATCMD_IMU_GYRO_FSR              0x31
#define ATCMD_IMU_AG_LPF                0x32
#define ATCMD_IMU_SAMPLERATE            0x33
#define ATCMD_IMU_AG_SAMPLERATE         0x34
#define ATCMD_IMU_MAG_SAMPLERATE        0x35
#define ATCMD_IMU_DMP_SAMPLERATE        0x36

#define ATCMD_IMU_MODE_ALL              0x40
#define ATCMD_IMU_MODE_3D               0x41
#define ATCMD_IMU_MODE_GYRO             0x42
#define ATCMD_IMU_MODE_MAG              0x43
#define ATCMD_IMU_MODE_4Q               0x44
#define ATCMD_IMU_MODE_6D               0x45
#define ATCMD_IMU_MODE_9D               0x46

#define ATCMD_ERROR                     0xff


void Hardware_Setup(void);
bool Hardware_Timer_Refresh(uint8_t channel, uint32_t ms);
void Hardware_Timer_Reset(uint8_t channel, uint32_t ms);
bool Hardware_RTC_Setup(void);
RTC_Date Hardware_Get_RTC(void);
bool Hardware_Get_ChargeStatu();
bool Hardware_Get_ChargeIndication(); 

bool Hardware_BLE_Setup(void);
void Hardware_BLE_startAdv(void);
void Hardware_BLE_Connect_Callback(uint16_t conn_handle);
void Hardware_BLE_Disconnect_Callback(uint16_t conn_handle, uint8_t reason);
void Hardware_BLE_UART_RxCallback(uint16_t conn_handle);
String Hardware_Get_BLE_Name(void);
bool Hardware_Get_BLE_Statu(void) ;

float Hardware_Get_VBAT(void);
uint8_t Hardware_Get_Battery_Percent(void);
bool Hardware_IMU_setup();
IMU_DATA Hardware_Get_IMU(void);
bool Hardware_IMU_Get_Enabled(void);
void Hardware_IMU_Set_Enabled(bool enabled);
void Hardware_IMU_Reset(void);
bool Hardware_Check_IMU_StopMove(void);
void Hardware_Set_SystemSleep(void) ;
bool Hardware_BLE_Send_IMU(void);
uint8_t Hardware_BLE_ATCommand_Decode(void);

void Hardware_Serial_println(bool ln,String monitor);





