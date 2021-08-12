
#include <Hardware.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <TouchButton.h>


#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <nrf_rtc.h>

// RTC PCF8563 相关定义
#define PCF8563_ADDRESS     0x51

// 9D Sensor 九轴传感器 相关定义
#define MPU9250_ADDRESS     0x69
#define PWR1_SLEEP_BIT      6
#define PWR1_RESET_BIT      7
#define MPU9250_PWR_MGMT_1  0x6B

// 电池电量 相关定义
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

bool charge_indication = false;
bool found_imu = false, found_rtc = false, rtcIrq = false;
bool sleep_enable = false;
bool IMU_Mode_Accel = true;
bool IMU_Mode_Gyro = true;
bool IMU_Mode_Magent = true;
bool IMU_Mode_4Q = true;

// 电池电量 相关变量


// BLE 相关变量
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery
bool BLE_Device_Connected = false;
bool BLE_UART_RxCallback =false;
String  BLE_UART_RxData;
String BLE_Name;

// RTC 相关变量
PCF8563_Class rtc;

// 9D Sensor 九轴传感器 相关变量
MPU9250_DMP imu;
IMU_DATA IMU_Data;
float IMU_Stop_Move_Defference;
uint32_t StopMove_OverTime_Target,StopMove_OverTime;
bool IMU_Enable;
bool IMU_Stop_Move;

void Hardware_Setup(void) {
    
    dbgPrintVersion();
    dbgMemInfo();

    Serial.println("Start...");

    Wire.begin();

    found_imu =  Hardware_IMU_setup();
    if (found_imu) {
        Serial.println("setupIMU pass");  
        
        IMU_Stop_Move = false;
    } else {
        Serial.println("setupIMU fail");    
    }

    

    Hardware_IMU_Set_Enabled(false);
    IMU_Enable = false;


    found_rtc = Hardware_RTC_Setup();
    if (found_rtc) {
        Serial.println("RTC deteced Pass");
    } else {
        Serial.println("RTC deteced fail");
    }

    Hardware_BLE_Setup();

    pinMode(CHARGE_PIN, INPUT_PULLUP);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }

}

static uint32_t targetTime[10];
bool Hardware_Timer_Refresh(uint8_t channel, uint32_t ms) {
    if (targetTime[channel] < millis()) {
        targetTime[channel] = millis() + ms;
        return true;
    }
    return false;
}

void Hardware_Timer_Reset(uint8_t channel, uint32_t ms)  {
    targetTime[channel] = millis() + ms;
}

void Hardware_Set_SystemSleep(void) {

    
    Hardware_Serial_println(1,"System into Sleep Mode");
    digitalWrite(TFT_BL, LOW);
    pinMode(TFT_BL, INPUT);
    Hardware_IMU_Set_Enabled(false);
    SerialFlash.sleep();
    sleep_enable = true;

    sd_power_mode_set(NRF_POWER_MODE_LOWPWR); 
    NRF_UARTE0->ENABLE = 0; //disable UART  
    NRF_SAADC ->ENABLE = 0; //disable ADC
    NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
    NRF_PWM1  ->ENABLE = 0;
    NRF_PWM2  ->ENABLE = 0;
    NRF_TWIM1 ->ENABLE = 0; //disable TWI Master
    NRF_TWIS1 ->ENABLE = 0; //disable TWI Slave
    NRF_SPI0 -> ENABLE = 0; //disable SPI
    NRF_SPI1 -> ENABLE = 0; //disable SPI
    NRF_SPI2 -> ENABLE = 0; //disable SPI

    pinMode(TP_PIN_PIN, INPUT_PULLDOWN_SENSE);   //深度睡眠，低电平唤醒后系统重置
    sd_power_system_off(); 
    
}

bool Hardware_RTC_Setup(void) {
    bool output = false;
    Wire.beginTransmission(PCF8563_ADDRESS);
    if (Wire.endTransmission() == 0) {
        rtc.begin();
        output = true;
    } 
    return output;
}

RTC_Date Hardware_Get_RTC(void) {
    
    return (rtc.getDateTime());
}


String Hardware_Get_BLE_Name(void)
{
    return BLE_Name;
}

bool Hardware_BLE_Setup(void) {
    
    bool output = false; 

    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behavior, but provided
    // here in case you want to control this LED manually via PIN 19
    Bluefruit.autoConnLed(true);

    // Config the peripheral connection with maximum bandwidth 
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); 

    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

    BLE_Name = getMcuUniqueID();
    BLE_Name = BLE_Name.substring(BLE_Name.length() - 4);
    BLE_Name = "Johnson-" + BLE_Name;
    Bluefruit.setName(BLE_Name.c_str()); // useful testing with multiple central connections
    Bluefruit.Periph.setConnectCallback(Hardware_BLE_Connect_Callback);
    Bluefruit.Periph.setDisconnectCallback(Hardware_BLE_Disconnect_Callback);

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and Start Device Information Service
    bledis.setManufacturer(MANUFACTURER_NAME);
    bledis.setModel(MODEL_NAME);
    bledis.setSoftwareRev(SOFTWARE_VERSION);
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();
    bleuart.setRxCallback(Hardware_BLE_UART_RxCallback);

    // Start BLE Battery Service
    blebas.begin();
    blebas.write(100);
    Hardware_BLE_startAdv();
    Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
    Serial.println("Once connected, enter character(s) that you wish to send");

    return output;
}

void Hardware_BLE_startAdv(void)
{
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();
  
    /* Start Advertising
     * - Enable auto advertising if disconnected
    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    * - Timeout for fast mode is 30 seconds
    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
    * 
    * For recommended advertising interval
    * https://developer.apple.com/library/content/qa/qa1931/_index.html   
    */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
    
}

void Hardware_BLE_UART_RxCallback(uint16_t conn_handle) {
    (void) conn_handle;
  
  // Forward data from Mobile to our peripheral
    char str[64+1] = { 0 };
    bleuart.read(str, 64);


    BLE_UART_RxData = str;
    BLE_UART_RxCallback = true;

    Serial.print("[Prph] RX: ");
    Serial.println(BLE_UART_RxData); 
}

void Hardware_BLE_Connect_Callback(uint16_t conn_handle)
{
    
    // Get the reference to current connection
    BLEConnection* connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = { 0 };
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);
    BLE_Device_Connected = true;
}

    
void Hardware_BLE_Disconnect_Callback(uint16_t conn_handle, uint8_t reason)
{
    /**
    * Callback invoked when a connection is dropped
    * @param conn_handle connection where this event happens
    * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
     */

    (void) conn_handle;
    (void) reason;

    Serial.println();
    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    BLE_Device_Connected = false;
}

bool Hardware_IMU_setup()
{
    bool output = true;

    // Call imu.begin() to verify communication with and
    // initialize the MPU-9250 to it's default values.
    // Most functions return an error code - INV_SUCCESS (0)
    // indicates the IMU was present and successfully set up

    
    if (imu.begin() != INV_SUCCESS) {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        output = false;
    }

    
    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    IMU_Data.GyroFSR = 2000;
    imu.setGyroFSR(IMU_Data.GyroFSR); // Set gyro to 2000 dps

    // Accel options are +/- 2, 4, 8, or 16 g
    IMU_Data.AccelFSR = 2;
    imu.setAccelFSR(IMU_Data.AccelFSR); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    IMU_Data.AG_LPF = 42;
    imu.setLPF(IMU_Data.AG_LPF); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    IMU_Data.All_SampleRate = 10;
    imu.setSampleRate(IMU_Data.All_SampleRate); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(IMU_Data.All_SampleRate); // Set mag rate to 10Hz

    
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 IMU_Data.All_SampleRate);


    IMU_Stop_Move_Defference = float(IMU_STOPMOVE_DEFFERENCE)/1000;
    StopMove_OverTime = IMU_STOPMOVE_OVERTIME;
    StopMove_OverTime_Target = StopMove_OverTime * (1000/(1000/IMU_Data.All_SampleRate)) * 60; 

    return output;
}

String Hardware_Print_IMU_Data(void)
{  
    String buffer;


  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.

    IMU_Data.time = imu.time;

    IMU_Data.ax = imu.calcAccel(imu.ax);
    IMU_Data.ay = imu.calcAccel(imu.ay);
    IMU_Data.az = imu.calcAccel(imu.az);    
    
    if(IMU_Mode_Gyro) {
        IMU_Data.gx = imu.calcGyro(imu.gx);
        IMU_Data.gy = imu.calcGyro(imu.gy);
        IMU_Data.gz = imu.calcGyro(imu.gz);
    }

    if(IMU_Mode_Magent) {
        IMU_Data.mx = imu.calcMag(imu.mx);
        IMU_Data.my = imu.calcMag(imu.my);
        IMU_Data.mz = imu.calcMag(imu.mz);
    }

    if(IMU_Mode_4Q) {
        IMU_Data.qw = imu.calcQuat(imu.qw);
        IMU_Data.qx = imu.calcQuat(imu.qx);
        IMU_Data.qy = imu.calcQuat(imu.qy);
        IMU_Data.qz = imu.calcQuat(imu.qz);

        IMU_Data.roll = imu.roll;
        IMU_Data.pitch = imu.pitch;
        IMU_Data.yaw = imu.yaw;
    }



    buffer =    "Time: "+ String(IMU_Data.time) + " ms" + " \n";

    if(IMU_Mode_Accel) {
        buffer +=   "Accelerometer(g):"
                    + String(IMU_Data.ax) + ","
                    + String(IMU_Data.ay) + "," 
                    + String(IMU_Data.az) + "\n";
    }
    if(IMU_Mode_Gyro) {
        buffer +=   "Gyroscope(dps):" 
                    + String(IMU_Data.gx) + ","
                    + String(IMU_Data.gy) + "," 
                    + String(IMU_Data.gz) + "\n";
    }
    if(IMU_Mode_Magent) {
        buffer +=  "Magnetometer(μT):"
                    + String(IMU_Data.mx) + ","
                    + String(IMU_Data.my) + "," 
                    + String(IMU_Data.mz) + "\n";
    }
    if(IMU_Mode_4Q) {
        buffer +=  "Quaternion:"
                    + String(IMU_Data.qw, 4) + "," 
                    + String(IMU_Data.qx, 4) + "," 
                    + String(IMU_Data.qy, 4) + "," 
                    + String(IMU_Data.qz, 4) + "\n"
                    + "Roll/Pitch/Yaw(°):" 
                    + String(IMU_Data.roll) + ","
                    + String(IMU_Data.pitch) + "," 
                    + String(IMU_Data.yaw) + "\n";
    }

    return buffer;            

}

bool Hardware_BLE_Send_IMU(void) {

    bool output = false;
    if(!IMU_Enable) return output;
    
    // Check for new data in the FIFO
    if( imu.fifoAvailable() ) {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        bool update = false;
        if(imu.dmpUpdateFifo() == INV_SUCCESS) update = true;
        if(IMU_Mode_Accel && !IMU_Mode_Gyro && !IMU_Mode_Magent && !IMU_Mode_4Q && IMU_Data.All_SampleRate > 100) {
            update = false;
            if (imu.dataReady()) update = true;
        }
        
        if(update)  {
            imu.update(UPDATE_ACCEL);
            if(IMU_Mode_Gyro)   imu.update(UPDATE_GYRO );
            if(IMU_Mode_Magent) imu.update(UPDATE_COMPASS);
            if(IMU_Mode_4Q)  imu.computeEulerAngles();

            String Buffer = "";
            Buffer = Hardware_Print_IMU_Data();
            output = Hardware_Check_IMU_StopMove();    

            if(!BLE_Device_Connected) return output;
            char buf[256];
            int count = Buffer.length();
            Buffer.toCharArray(buf,count+1);
            bleuart.write( buf, count );      
        }
    }
    return output; 
}



float ax,ay,az;
uint32_t  StopMove_Count = 0;
bool IMU_Stop_Move_old;

bool Hardware_Check_IMU_StopMove(void) {
    bool output = false;
    if (abs(ax - IMU_Data.ax) < IMU_Stop_Move_Defference && 
        abs(ay - IMU_Data.ay) < IMU_Stop_Move_Defference && 
        abs(az - IMU_Data.az) < IMU_Stop_Move_Defference) 
    {
        if(StopMove_Count >= StopMove_OverTime_Target) {
            StopMove_Count = 0;
            IMU_Stop_Move = true;
            output = true;
            Hardware_Serial_println(1,"IMU Stop Move Over Time");
        }
        else {
            ++StopMove_Count;
            if(StopMove_Count%100 == 0) { 
                Serial.println("StopMove_OverTime_Target is "+String(StopMove_OverTime_Target));
                Serial.println("StopMove_Count is "+String(StopMove_Count));
            }    
        }    
    }
    else 
    {
        IMU_Stop_Move = false;
        StopMove_Count = 0;
    }
    ax = IMU_Data.ax;
    ay = IMU_Data.ay;
    az = IMU_Data.az;
    return output;
}


IMU_DATA Hardware_Get_IMU(void) {

    return (IMU_Data);
}

float Hardware_Get_VBAT(void)
{
    float raw;
    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);
    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14
    // Let the ADC settle
    delay(1);
    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(PIN_VBAT);
    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);
    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    return raw * REAL_VBAT_MV_PER_LSB;
}

#define BT_FULL_VOLTAGE 3.78
#define BT_EMPTY_VOLTAGE 2.78

uint8_t Hardware_Get_Battery_Percent() {

    float battery_voltage;
    uint8_t Battery_Percent;

    battery_voltage = Hardware_Get_VBAT() / 1000;

    if (battery_voltage >= BT_FULL_VOLTAGE)  Battery_Percent = 100;
    if (battery_voltage <= BT_EMPTY_VOLTAGE)  Battery_Percent = 0;
    
    if (battery_voltage < BT_FULL_VOLTAGE && battery_voltage > BT_EMPTY_VOLTAGE) {
        Battery_Percent = (battery_voltage-BT_EMPTY_VOLTAGE)/(BT_FULL_VOLTAGE-BT_EMPTY_VOLTAGE) * 100 ;
    } 
    blebas.write(Battery_Percent);
    return Battery_Percent;
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b = readByte(devAddr, regAddr);
    printf("rb:%x\n", b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    printf("wb:%x\n", b);
    writeByte(devAddr, regAddr, b);
}

bool Hardware_IMU_Get_Enabled(void) {

    return IMU_Enable;
}

void Hardware_IMU_Set_Enabled(bool enabled)
{
    IMU_Enable = enabled;
    if(enabled) enabled = false;
     else enabled = true;
    writeBit(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, PWR1_SLEEP_BIT , enabled);
}

void Hardware_IMU_Reset(void) {

    writeBit(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, PWR1_RESET_BIT , IMU_Enable);
        
    if (imu.begin() != INV_SUCCESS) {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
    }

    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setGyroFSR(IMU_Data.GyroFSR); 
    imu.setAccelFSR(IMU_Data.AccelFSR); 
    imu.setLPF(IMU_Data.AG_LPF);
    imu.setSampleRate(IMU_Data.All_SampleRate); 
    imu.setCompassSampleRate(IMU_Data.All_SampleRate); 
    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 IMU_Data.All_SampleRate);
}

void Hardware_Serial_println(bool ln,String monitor) {
    
    if (ln) Serial.println(monitor);
        else Serial.print(monitor);

    if (BLE_Device_Connected) {
        if(ln) monitor += '\n';
        if (ln) bleuart.print(monitor);
         else bleuart.println(monitor);
    }
}

bool Hardware_Get_BLE_Statu(void) {

    return BLE_Device_Connected;
}

bool Hardware_Get_ChargeStatu() {

    if (digitalRead(CHARGE_PIN) == LOW) { 
        return true;
    } else {
        return false;
    }    
}

bool Hardware_Get_ChargeIndication() {

    bool output;
    output = charge_indication;
    charge_indication = false;
    return output;
}

uint8_t Hardware_BLE_ATCommand_Decode(void) {
    
    uint8_t output = ATCMD_NA; 
    String RX_Data,RX_Index;
    uint16_t Year;
    uint8_t Month, Day, Hour, Minute, Second;
    int Data_Length;
    
    if(!BLE_UART_RxCallback) return output;

    BLE_UART_RxCallback = false;
    RX_Data = BLE_UART_RxData;

    RX_Data.toUpperCase();
    if(RX_Data.startsWith("AT+"))  {
        
        RX_Data = RX_Data.substring(3);
        RX_Index = RX_Data.substring(0,RX_Data.indexOf("#"));

        if(RX_Index.equals(ATCMD_STR_RESET)) {

            Hardware_Serial_println(1,"System to Reset...");
            sd_nvic_SystemReset();        
        }

        if(RX_Index.equals(ATCMD_STR_SLEEP)) {

            Hardware_Set_SystemSleep();   
        }

        if(RX_Index.equals(ATCMD_STR_IMU_ON)) {
            Hardware_IMU_Set_Enabled(true);
            output = ATCMD_IMU_ON;
            Hardware_Serial_println(1,"AT+IMU-ON#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_OFF)) {
            Hardware_IMU_Set_Enabled(false);
            output = ATCMD_IMU_OFF;
            Hardware_Serial_println(1,"AT+IMU-OFF#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_3D)) {
            IMU_Mode_Accel = true;
            IMU_Mode_Gyro = false;
            IMU_Mode_Magent = false;
            IMU_Mode_4Q = false;
            output = ATCMD_IMU_MODE_3D;
            Hardware_Serial_println(1,"AT+MODE-ACCEL#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_GYRO)) {
            IMU_Mode_Accel = false;
            IMU_Mode_Gyro = true;
            IMU_Mode_Magent = false;
            IMU_Mode_4Q = false;
            output = ATCMD_IMU_MODE_GYRO;
            Hardware_Serial_println(1,"AT+MODE-GYRO#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_MAG)) {
            IMU_Mode_Accel = false;
            IMU_Mode_Gyro = false;
            IMU_Mode_Magent = true;
            IMU_Mode_4Q = false;
            output = ATCMD_IMU_MODE_MAG;
            Hardware_Serial_println(1,"AT+MODE-MAG#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_4Q)) {
            IMU_Mode_Accel = false;
            IMU_Mode_Gyro = false;
            IMU_Mode_Magent = false;
            IMU_Mode_4Q = true;
            output = ATCMD_IMU_MODE_4Q;
            Hardware_Serial_println(1,"AT+MODE-4Q#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_6D)) {
            IMU_Mode_Accel = true;
            IMU_Mode_Gyro = true;
            IMU_Mode_Magent = false;
            IMU_Mode_4Q = false;
            output = ATCMD_IMU_MODE_6D;
            Hardware_Serial_println(1,"AT+MODE-6D#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_9D)) {
            IMU_Mode_Accel = true;
            IMU_Mode_Gyro = true;
            IMU_Mode_Magent = true;
            IMU_Mode_4Q = false;
            output = ATCMD_IMU_MODE_9D;
            Hardware_Serial_println(1,"AT+MODE-9D#");
        }

        if(RX_Index.equals(ATCMD_STR_IMU_MODE_ALL)) {
            IMU_Mode_Accel = true;
            IMU_Mode_Gyro = true;
            IMU_Mode_Magent = true;
            IMU_Mode_4Q = true;
            output = ATCMD_IMU_MODE_ALL;
            Hardware_Serial_println(1,"AT+MODE-ALL#");
        }


        if(RX_Index.equals(ATCMD_STR_SYS_DISPLAY_OFF_TIME)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            
            TouchButton_DisplayOff_Overtime_Set(RX_Index.toInt());
            Hardware_Serial_println(1,"AT+DISPLAY-OFF-TIME#" + RX_Index + "#(Seconds)");
            output = ATCMD_SYS_DISPLAY_OFF_TIME;
        }    

        if(RX_Index.equals(ATCMD_STR_SYS_SLEEP_TIME)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            
            TouchButton_Sleep_Overtime_Set(RX_Index.toInt());
            Hardware_Serial_println(1,"AT+SLEEP-TIME#" + RX_Index + "#(Minutes)");
            output = ATCMD_SYS_SLEEP_TIME;
        }   

        if(RX_Index.equals(ATCMD_STR_IMU_ACCEL_FSR)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            
            IMU_Data.AccelFSR = RX_Index.toInt();
            Hardware_IMU_Reset();

            Hardware_Serial_println(1,"AT+IMU-ACCEL-FSR#" + RX_Index + "#(g)");
            output = ATCMD_IMU_ACCEL_FSR;
        }   

        if(RX_Index.equals(ATCMD_STR_IMU_GYRO_FSR)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            
            IMU_Data.GyroFSR = RX_Index.toInt();
            Hardware_IMU_Reset();

            Hardware_Serial_println(1,"AT+IMU-GYRO-FSR#" + RX_Index + "#(dps)");
            output = ATCMD_IMU_GYRO_FSR;
        }      

        if(RX_Index.equals(ATCMD_STR_IMU_AG_LPF)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);

            IMU_Data.AG_LPF = RX_Index.toInt();
            Hardware_IMU_Reset();

            Hardware_Serial_println(1,"AT+IMU-AG-LPF#" + RX_Index + "#(Hz)");
            output = ATCMD_IMU_AG_LPF;
        }  

        if(RX_Index.equals(ATCMD_STR_IMU_SAMPLERATE)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);

            IMU_Data.All_SampleRate = RX_Index.toInt();
            Hardware_IMU_Reset();

            StopMove_OverTime_Target = StopMove_OverTime * (1000/(1000/IMU_Data.All_SampleRate)) * 60; 
            Hardware_Serial_println(1,"AT+IMU-SAMPLERATE#" + RX_Index + "#(Hz)");
            output = ATCMD_IMU_SAMPLERATE;
        }  

        if(RX_Index.equals(ATCMD_STR_IMU_STOPMOVE_DEFFERENT)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);

            IMU_Stop_Move_Defference = float(RX_Index.toInt())/1000;

            Hardware_Serial_println(1,"AT+IMU-STOPMOVE-DEFFERENT#" + RX_Index + "#(mg)");
            output = ATCMD_IMU_STOPMOVE_DEFFERENT;
        }  

        if(RX_Index.equals(ATCMD_STR_IMU_STOPMOVE_OVERTIME)) {
            //清除 命令 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);

            StopMove_OverTime = RX_Index.toInt();
            StopMove_OverTime_Target = StopMove_OverTime * (1000/(1000/IMU_Data.All_SampleRate)) * 60; 
            StopMove_Count =0;

            Hardware_Serial_println(1,"AT+IMU-STOPMOVE-OVERTIME#" + RX_Index + "#(Minutes)");
            output = ATCMD_IMU_STOPMOVE_OVERTIME;
        }  

        if(RX_Index.equals(ATCMD_STR_DATE)) {
            
            //清除 "DATE" 字符在 RX_Data里，便于解析 数值
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Year data
            Data_Length = RX_Data.indexOf(".");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Year = RX_Index.toInt();
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Month data
            Data_Length = RX_Data.indexOf(".");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Month = RX_Index.toInt();
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Day data
            Data_Length = RX_Data.indexOf(".");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Day = RX_Index.toInt();
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Hour data
            Data_Length = RX_Data.indexOf(".");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Hour = RX_Index.toInt();
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Minute data
            Data_Length = RX_Data.indexOf(".");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Minute = RX_Index.toInt();
            RX_Data = RX_Data.substring(RX_Index.length()+1);

            // Second data
            Data_Length = RX_Data.indexOf("#");
            if(Data_Length == -1)  return ATCMD_ERROR;
            RX_Index = RX_Data.substring(0,Data_Length);
            Second = RX_Index.toInt();

            rtc.setDateTime(Year, Month, Day, Hour, Minute, Second);
            
            Hardware_Serial_println(1,"AT+DATE#" + String(Year) + "."
                                                    + String(Month) + "."
                                                    + String(Day) + "."
                                                    + String(Hour) + "."
                                                    + String(Minute) + "."
                                                    + String(Second) + "#");
            output = ATCMD_DATE;

        }

    }      

    return output;
}
