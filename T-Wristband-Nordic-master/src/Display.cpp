#include <Display.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Display_Picture.h>



Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

bool Display_Setup(void)
{
    // OR use this initializer (uncomment) if using a 0.96" 160x80 TFT:


    if (!SerialFlash.begin(SPI_FLASH_CS)) {
        Serial.println("setupSerialFlash fail");
    } else {
        Serial.println("setupSerialFlash pass");
    }

    tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
    tft.setRotation(1);
    tft.drawRGBBitmap(0, 0, ICON_Johnson_logo, 160, 80);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    return true;
}

void Display_IMU_Switch(bool status) {

    tft.fillScreen(ST77XX_BLACK);
    if(status) {
        tft.drawRGBBitmap(80-25, 30 , ICON_Switch_ON, 50, 20);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(20, 15);
        tft.setTextSize(1);
        tft.print("Motion Capture On..");
        Hardware_Serial_println(1,"Motion Capture On..");  

    }
    else {
        tft.drawRGBBitmap(80-25, 30 , ICON_Switch_OFF, 50, 20);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(20, 15);
        tft.setTextSize(1);
        tft.print("Motion Capture Off..");
        Hardware_Serial_println(1,"Motion Capture Off..");  
    }

}

void Display_BLE_ATCommand(String display) {
    
    tft.fillScreen(ST77XX_BLACK);
    tft.drawRGBBitmap(80-30, 20 , ICON_Setting, 60, 60);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(0, 5);
    tft.setTextSize(1);
    tft.print(display);
    Hardware_Serial_println(1,display);  

}

void Display_OFF() {
    Hardware_Serial_println(1,"Display OFF");
    tft.sendCommand(ST77XX_SLPIN);
    tft.sendCommand(ST77XX_DISPOFF);
    digitalWrite(TFT_BL, LOW);
    //Hardware_Set_TFT_Backlit(TFT_BACKLIT_OFF);
}

void Display_ON() {
    Hardware_Serial_println(1,"Display ON");
    tft.sendCommand(ST77XX_SLPOUT);
    tft.sendCommand(ST77XX_DISPON);
    digitalWrite(TFT_BL, HIGH);
    //Hardware_Set_TFT_Backlit(TFT_BACKLIT_ON);
}

bool ChargeStatus_initial = 1;
bool chager_statu_old;

void Display_ChargeStatus(bool status) {
    
    if(status != chager_statu_old || ChargeStatus_initial) {
        if (status) {
            Hardware_Serial_println(1,"Battery Charge ON");
            tft.drawRGBBitmap(115, 64, ICON_Charge, 16, 16);
        } 
        else {
            Hardware_Serial_println(1,"Battery Charge OFF");
            tft.fillRect(115, 64, 16, 16, ST77XX_BLACK);
        }    
    }
    chager_statu_old = status;
    ChargeStatus_initial = 0;
}

bool BLEStatu_inital = 1;
bool BLEStatu_old;

void Display_BLEStatus(bool status) {
    
    if(status != BLEStatu_old|| BLEStatu_inital) {
        if (status) {
            Hardware_Serial_println(1,"BLE ON");
            tft.drawRGBBitmap(95, 64, ICON_BLE_Statu, 16, 16);

        } 
        else {
            Hardware_Serial_println(1,"BLE OFF");
            tft.fillRect(95, 64, 16, 16, ST77XX_BLACK);
        }    
    }      
    BLEStatu_old = status;
    BLEStatu_inital = 0;
}

bool Battery_initial = 1;
float battery_percent_old;
void Display_Battery(uint8_t battery_percent) {
    
    if(Battery_initial) {
        
        tft.drawRGBBitmap(160-28, 64, ICON_Battery, 28, 16);
    }
    
    if(battery_percent != battery_percent_old || Battery_initial) {
        tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
        tft.setCursor (135, 69);
        tft.setTextSize(1);
        tft.print(battery_percent);
        tft.print("%");  
    }

    battery_percent_old =  battery_percent;
    Battery_initial = 0; 
}

bool BLE_Name_initial = 1;
void Display_BLE_Name(String ble_name) {

    if(BLE_Name_initial)  {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor (10, 0);
        tft.print("BLE Name: " + ble_name);
    }
    BLE_Name_initial = 0;
}

uint8_t omm = 99;
uint8_t odate = 100;
uint8_t date_now, year_now, month_now, hh, mm, ss;
bool RTC_initial = 1;
void Display_RTC(RTC_Date datetime) {
    static int16_t x, y;
    
    date_now = datetime.day;
    month_now = datetime.month;
    year_now = datetime.year;
    hh = datetime.hour;
    mm = datetime.minute;
    ss = datetime.second;

    if(date_now != odate || RTC_initial) { 
        
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.setCursor (8, 70);
        tft.printf("%d/%d/%d", datetime.year,month_now, date_now);
        odate = date_now;
    }

        
    tft.setTextSize(4);

    // Update digital time
    uint8_t xpos = 19;
    uint8_t ypos = 20;

    if (omm != mm ||  RTC_initial) {
        tft.setTextColor(0x39C4, ST77XX_BLACK);
        tft.setTextSize(4);
        tft.setCursor(xpos, ypos);
        tft.println("88:88");
        tft.setCursor(xpos, ypos);
        tft.setTextColor(0xFBE0, ST77XX_BLACK); // Orange
        omm = mm;

        if (hh < 10) {
            tft.print('0');
        }

        tft.print(hh);
        x = tft.getCursorX();
        y = tft.getCursorY();

        tft.print(':');

        if (mm < 10) {
            tft.print('0');
        }
        tft.print(mm);
    }

    if (ss % 2 ||  RTC_initial) {
        tft.setTextColor(0x39C4, ST77XX_BLACK);
        tft.setCursor(x, y);
        tft.print(':');
        tft.setTextColor(0xFBE0, ST77XX_BLACK);
    } else {
        tft.setTextColor(0xFBE0, ST77XX_BLACK);
        tft.setCursor(x, y);
        tft.print(':');
    }

    RTC_initial = 0;
}

#define IUM_TITLE_Y  0
#define IUM_TITLE_X  0
#define IUM_DIS_GAP_Y  12
bool IMU_inital = 1;

void Display_Sensor(IMU_DATA imu,uint8_t Index)
{
    if (IMU_inital) {
        
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setTextSize(1);
        tft.setCursor(IUM_TITLE_Y, IUM_TITLE_X);
        
        switch(Index) {
            case INDEX_ACCEL:
            tft.print("Accelerometer(g)");  tft.print(" - FSR:");  tft.print(imu.AccelFSR);
            tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + IUM_DIS_GAP_Y);
            tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            tft.print("SampleRate:");  tft.print(imu.All_SampleRate);  tft.print("Hz ");
            tft.print("LPF:");  tft.print(imu.AG_LPF);   tft.print("Hz");
            break;

            case INDEX_GYRO:
            tft.print("Gyroscope(dps)");   tft.print(" - FSR:");  tft.print(imu.GyroFSR); 
            tft.setCursor(IUM_TITLE_X  , IUM_TITLE_Y + IUM_DIS_GAP_Y);
            tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            tft.print("SampleRate:");  tft.print(imu.All_SampleRate);  tft.print("Hz ");
            tft.print("LPF:");  tft.print(imu.AG_LPF);   tft.print("Hz");
            break;

            case INDEX_COMPASS:
            tft.print("Magnetometer(uT)");  
            tft.setCursor(IUM_TITLE_X  , IUM_TITLE_Y + IUM_DIS_GAP_Y);
            tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            tft.print("SampleRate:");  tft.print(imu.All_SampleRate);  tft.print("Hz ");
            break;

            case INDEX_EULER:
            tft.println("Roll/Pitch/Yaw(o)");  
            tft.setCursor(IUM_TITLE_X  , IUM_TITLE_Y + IUM_DIS_GAP_Y);
            tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
            tft.print("SampleRate:");  tft.print(imu.All_SampleRate);  tft.print("Hz ");
            break;

            default:
            break;
        }

    }
    
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*2));

    switch(Index) {

        case INDEX_ACCEL:
        tft.print("Accel X: "); tft.println(String(imu.ax,4));  
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*3));
        tft.print("Accel Y: "); tft.println(String(imu.ay,4)); 
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*4));
        tft.print("Accel Z: "); tft.println(String(imu.az,4));
        break;

        case INDEX_GYRO:
        tft.print("Gyro X: "); tft.println(String(imu.gx,4));   
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*3));
        tft.print("Gyro Y: "); tft.println(String(imu.gy,4));  
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*4));
        tft.print("Gyro Z: "); tft.println(String(imu.gz,4));
        break;

        case INDEX_COMPASS:
        tft.print("Magneto X: "); tft.println(String(imu.mx,2));   
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*3));
        tft.print("Magneto Y: "); tft.println(String(imu.my,2));  
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*4));
        tft.print("Magneto Z: "); tft.println(String(imu.mz,2)); 
        break;

        case INDEX_EULER:
        tft.print("Roll:  ");  tft.println(String(imu.roll,2));   
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*3));
        tft.print("Pitch: "); tft.println(String(imu.pitch,2));  
        tft.setCursor(IUM_TITLE_X , IUM_TITLE_Y + (IUM_DIS_GAP_Y*4));
        tft.print("Yaw:   ");   tft.println(String(imu.yaw,2));
        break;

        default:
        break;
    }
    IMU_inital = 0;
}


void Display_initial_refresh() {
    // HR_initial = 1;
    BLE_Name_initial = 1;
    RTC_initial  = 1;
    IMU_inital = 1;
    Battery_initial = 1;
    ChargeStatus_initial = 1;
    BLEStatu_inital = 1;
}