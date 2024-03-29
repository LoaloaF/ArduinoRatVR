#include <Arduino.h>
#include <mbed.h>
#include <Arduino_PortentaBreakout.h>

/*
================================================================================
define shared memory variables accessed by both M7 and M4 
================================================================================
*/
struct shared_data
{
  char startNewAction;  // can be A S F P
  uint16_t actionLength;  // relevant for S and P
  uint16_t actionDelay;  // relevant for S
  char startedM4Action; // can be A S F P AND R (when reward is opened)
  
  // // written to serial by command like Y100,1,100
  // uint16_t reward_length;
  // uint16_t sound_delay; // delay between sound end to reward start
  
  // bool reward_sound; // if it is a reward sound -> high, if low -> punish sound
  // bool new_data; //status : 1 = new data available; 0 = no new data available, set when noew serial command arrives, activates M4 actions
  
  // // i think feedback?
  // bool air;

  // // for feedback that M4 executed something
  // bool new_action_m4; // new action from M4
  // bool reward;
  // bool sound;
};

#ifdef CORE_CM7
/*
================================================================================
define M7 variables
================================================================================
*/
#include <SPI.h>
#include <avr/pgmspace.h>
#include "ADNS9800.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "pdm2pcm_glo.h" // needed on linux

#pragma region variables

#define BAUDRATE 2000000

// shared memory variables
#define HSEM_ID 0
#define HSEM_PROCESS 0
volatile struct shared_data * const buff= (struct shared_data *)0x38001000;
int c=1;

// ball sensor readout variables
byte initComplete=0;
byte Motion = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xydat[2];
int xy2dat[2];
byte ballBurstReadout[6];
double dP;
double dR;
double dY;
uint32_t dPRY_globalID;
int pCum = 0;
int rCum = 0;
int yCum = 0;

breakoutPin ncs = SPI1_CS;
breakoutPin ncs2 = I2S_WS;
const double px1 = 0.8151;
const double rx1 = 0.1849;
const double yx1 = -0.5959;
const double py1 = 0.2414;
const double ry1 = -0.2414;
const double yy1 = -0.7779;
const double px2 = -0.1849;
const double rx2 = -0.8151;
const double yx2 = 0.5959;
const double py2 = -0.2414;
const double ry2 = 0.2414;
const double yy2 = -0.7779;

char local_startNewAction;
uint16_t local_actionLength;
uint16_t local_actionDelay;
char local_startedM4Action;

// both M4 and M7 define these
// uint16_t local_reward_length;
// uint16_t local_sound_length;
// uint16_t local_sound_delay; // delay between sound end to reward start
// bool local_reward_sound = false;
// bool local_air = false;

// main air valve state
bool airON = 0;

// ball sensor logging variables
int ballVel[3];
uint32_t ballVelTimestamp;
uint32_t ballVelPckgID = 0;
uint32_t globalID = 0;
String ballVelPckgBase;
String ballVelPckgValue;


// m4 action processing 
bool local_new_M4_action;
bool local_sound;
bool local_reward;

// M4 action logging
String m4actionPckgBase;
String m4actionPckgValue;
int m4actionPckgID=0;
int m4actionTimestamp;

uint32_t pckgID = 0;
uint32_t successPckgID = 0;
uint32_t failurePckgID = 0;
uint32_t rewardPckgID = 0;
uint32_t punishmentPckgID = 0;


uint16_t m7_reward_length;
bool m7_reward_sound;
uint16_t m7_sound_delay; // delay between sound end to reward start


// lick sensor processing and logging
#define LICK_THRESHOLD 240
bool is_licking=false;
uint32_t start_lick_timestamp;
uint32_t end_lick_timestamp;
uint32_t lickPckgID=0;
String lickPckgBase;
String lickPckgValue;


// Registers for screen?
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
#pragma endregion

/*
================================================================================
define M7 helper functions related to screen, ball sensor, serial comm
================================================================================
*/
#pragma region Helper functions
void dispRegisters(breakoutPin pin);

void performStartup(breakoutPin pin);

void adns_write_reg(byte reg_addr, byte data, breakoutPin pin);

void adns_com_begin(breakoutPin pin){
  digitalWrite(pin, LOW);
}

void adns_com_end(breakoutPin pin){
  digitalWrite(pin, HIGH);
}

byte adns_read_reg(byte reg_addr, breakoutPin pin){
  adns_com_begin(pin);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end(pin);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data, breakoutPin pin){
  adns_com_begin(pin);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end(pin);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(breakoutPin pin){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 1...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02, pin); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d, pin); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18, pin); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(pin);
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end(pin);
}

void performStartup(breakoutPin pin){
  adns_com_end(pin); // ensure that the serial port is reset
  adns_com_begin(pin); // ensure that the serial port is reset
  adns_com_end(pin); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a, pin); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion, pin);
  adns_read_reg(REG_Delta_X_L, pin);
  adns_read_reg(REG_Delta_X_H, pin);
  adns_read_reg(REG_Delta_Y_L, pin);
  adns_read_reg(REG_Delta_Y_H, pin);
  // upload the firmware
  adns_upload_firmware(pin);
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0, pin);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0, pin);
  
  delay(10);

  Serial.println("Optical Chip with following number initialized: TODO");
}

void dispRegisters(breakoutPin pin){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  }; // TODO change CPI to CPI2 if ncs2 pin
  byte regres;

  digitalWrite(pin,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delayMicroseconds(50);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delayMicroseconds(50);
  }
  digitalWrite(pin,HIGH);
}

void readXY(int *xy, breakoutPin pin){
  digitalWrite(ncs,LOW);

  adns_com_begin(pin);
  SPI.transfer(REG_Motion_Burst & 0x7F);
  delayMicroseconds(100); // t_SRAD

  // burst readout mode
  Motion = SPI.transfer(0);
  Motion = SPI.transfer(0);
  xL = SPI.transfer(0);
  xH = SPI.transfer(0);
  yL = SPI.transfer(0);
  yH = SPI.transfer(0);
  
  digitalWrite(ncs,HIGH); 
  delayMicroseconds(1); // ncs High for at least 500ns to end burst mode readout

  adns_com_end(pin);
  
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
}


String constr_pckg_base_str(String pckgName, int pckgID, int pckgTimestamp) {
  String pckgBaseString;
  pckgBaseString.reserve(100);
  pckgBaseString = "N:" + pckgName +
                   ",ID:" + String(pckgID) +
                   ",T:" + String(pckgTimestamp) +
                   ",V:";
  return pckgBaseString;
}

// String constr_value_single_str(const int value, const String name) {
//   String valueString = "{" + name + ":" + String(value) + "}";
//   return valueString;
// }

void serialwrite_package(String baseString, String valueString) {
  Serial.println("<{" + baseString + valueString + "}>");
}

// char validActions[] = {'A', 'S', 'F', 'P'};
// int nValidActions = 4;
// bool isValidAction(char target) {
//     for (int i = 0; i < nValidActions; i++) {
//         if (validActions[i] == target) {
//             return true; // If the target char is found in the array, return true
//         }
//     }
//     return false; // If the target char is not found in the array, return false
// }

void processSerialInput(){
  
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()==0) return;
    local_startNewAction = cmd.charAt(0);
    int comma1;

    switch (local_startNewAction) {

      case 'A': // turn on main air supply for levitating ball
        Breakout.digitalWrite(PWM3, (airON) ? LOW : HIGH);
        airON = !airON;
        return;
      
      // M4 actions
      case 'S': // trigger sucessful outcome, success-sound + open reward valve
        comma1 = cmd.indexOf(',',1);
        local_actionLength = cmd.substring(1,comma1).toInt();
        local_actionDelay = cmd.substring(comma1+1).toInt();
        break;
        
      case 'F': // trigger failure outcome, only failure-sound
        break;
      
      case 'P': // trigger punishment, open airpuff airvalve
        local_actionLength = cmd.substring(1).toInt();
        break;

      default:
        return;
    }

    // safe SHM read/write done
    while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
    SCB_CleanInvalidateDCache_by_Addr(buff, sizeof(*buff));

    buff->startNewAction = local_startNewAction;
    buff->actionLength = local_actionLength;
    buff->actionDelay = local_actionDelay;
    buff->startedM4Action = local_startedM4Action;

    SCB_CleanDCache_by_Addr(buff, sizeof(*buff));
    HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);
    // safe SHM read/write done
}

void processM4actions(){
  while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
  SCB_CleanInvalidateDCache_by_Addr(buff, sizeof(*buff));
  
  local_startedM4Action = buff->startedM4Action;
  if (local_startedM4Action != '\0'){
    buff->startedM4Action = '\0';
  }

  SCB_CleanDCache_by_Addr(buff, sizeof(*buff));
  HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

  if (local_startedM4Action!='\0') 
  {
    m4actionTimestamp = micros();
    switch (local_startedM4Action) {
      case 'S': // success sound
        pckgID = successPckgID;
        successPckgID++;
        break;
      case 'F': // success sound
        pckgID = failurePckgID;
        failurePckgID++;
        break;
      case 'R': // success sound
        pckgID = rewardPckgID;
        rewardPckgID++;
        break;
      case 'P': // success sound
        pckgID = punishmentPckgID;
        punishmentPckgID++;
        break;
    }
    m4actionPckgBase = constr_pckg_base_str(String(local_startedM4Action), pckgID, m4actionTimestamp);
    serialwrite_package(m4actionPckgBase, "1");
  }
}

// black magic
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
#pragma endregion



/*
================================================================================
setup M7: screen, ball sensor, lick, serial comm
================================================================================
*/
void setup() {
  Serial.begin(BAUDRATE);
  LL_RCC_ForceCM4Boot();
  __HAL_RCC_HSEM_CLK_ENABLE();

  // photodiode pin (deprecated)
  // Breakout.pinMode(ANALOG_A2, INPUT);
  
  // Peter's code:
  Breakout.I2C_1.begin();
  Breakout.pinMode(ncs, OUTPUT);
  Breakout.pinMode(ncs2, OUTPUT);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(1000);
  display.clearDisplay();

  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  //  SPI.setDataMode(SPI_MODE3);
  //  SPI.setBitOrder(MSBFIRST);
  //  SPI.setClockDivider(2);
  //  
  delay(200);
  performStartup(ncs);
  delay(10);
  performStartup(ncs2);
  delay(10);
  //adns_write_reg(REG_Configuration_I, 0x29); // maximum resolution
  adns_write_reg(REG_Configuration_I, 0x09, ncs); // default resolution
  //adns_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(10);
  adns_write_reg(REG_Configuration_I, 0x09, ncs2); // default resolution
  
  //adns2_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(200);  
  dispRegisters(ncs);
  delay(10);
  dispRegisters(ncs2);
  delay(1000);
  initComplete=9;


  // hardware semaphore to block simultanious read and write of shared memory location for m4 and m7
  while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
  // Invalidate the cache so new value is read
  SCB_CleanInvalidateDCache_by_Addr(buff, sizeof(*buff));

  // set initial values for shared memory
  buff->startNewAction = '\0';
  buff->actionLength = 0;
  buff->actionDelay = 0;
  buff->startedM4Action = '\0';

  SCB_CleanDCache_by_Addr(buff, sizeof(*buff));
  // release hardware semaphore
  HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);


  // TTL MEAK1 sync
  
  // MAPPING:
  // PWM9 (M7) -> USER 1 -> DSub 1 -> Ball Sensor TTL                             // ASSIGNED, IMPLEMENTED, TESTED

  // PWM4 -> VALVE5 -> DSub 4 ->  main air valve                                  // ASSIGNED, IMPLEMENTED, TESTED
  // PWM3 -> VALVE4 -> DSub 5 -> 3.3V -> air puff trigger                         // ASSIGNED, IMPLEMENTED, TESTED
  
  // GPIO_4 (M4) USER_2 -> valve/reward trigger                                   // ASSIGNED, IMPLEMENTED, TESTED
  // PWM2 (M4) VALVE3 reward TTL                                                  // ASSIGNED, IMPLEMENTED, TESTED
  // PWM7 (M4) -> Sound generator                                                 // ASSIGNED, IMPLEMENTED, TESTED
  // GPIO_5 (M4) -> Sound TTL                                                     // ASSIGNED, IMPLEMENTED, TESTED

  // PWM8 (M7) -> USER 3 -> DSub 2 -> Airpuff Sensor TTL                          // ASSIGNED, IMPLEMENTED, TESTED



  // TTL pin setup for ball and lick
  Breakout.pinMode(PWM9, OUTPUT); // ball sensor ttl
  Breakout.pinMode(PWM8, OUTPUT); // lick sensor ttl

  // main airvalve -- working
  Breakout.pinMode(PWM3, OUTPUT);
  
  Breakout.pinMode(GPIO_6, OUTPUT);
  Breakout.digitalWrite(GPIO_6, HIGH);
  


}



/*
================================================================================
loop M7: screen, ball sensor, lick, serial comm
================================================================================
*/

void loop2() {
  // test loop to test pins, change from loop2 to loop and change regular loop to loop2
  Breakout.digitalWrite(PWM3, HIGH);
  delay(2000);
  Breakout.digitalWrite(PWM3, LOW);
  delay(2000);
}

void loop() {
  globalID++;

  /*
  ================================================================================
  (1) check for serial commands from PC
  ================================================================================
  */
  if (Serial.available()) {
    processSerialInput();
  }

  /*
  ================================================================================
  (2) parse and send of the ball sensor
  ================================================================================
  */

  Breakout.digitalWrite(PWM9, HIGH); // ball velocity ttl high
  readXY(&xydat[0], ncs);
  readXY(&xy2dat[0], ncs2);  
  ballVelTimestamp = micros();
  Breakout.digitalWrite(PWM9, LOW); // ball velocity ttl low
  // calculate ball velocity 
  dP = px1*xydat[0] + py1*xydat[1] + px2*xy2dat[0] + py2*xy2dat[1];
  dR = rx1*xydat[0] + ry1*xydat[1] + rx2*xy2dat[0] + ry2*xy2dat[1];
  dY = yx1*xydat[0] + yy1*xydat[1] + yx2*xy2dat[0] + yy2*xy2dat[1];
  rCum = rCum*0.9 + dR;
  yCum = yCum*0.9 + dY;
  pCum = pCum*0.9 + dP;
  // Send off ball velocity via serial port 
  ballVelPckgBase = constr_pckg_base_str("B", ballVelPckgID, ballVelTimestamp);
  ballVelPckgValue = String(rCum) +"_"+ String(yCum) +"_"+ String(pCum);
  serialwrite_package(ballVelPckgBase, ballVelPckgValue);
  ballVelPckgID++;
  
  /*
  ================================================================================
  (4) check animals licking, send package when lick over 
  ================================================================================
  */
  if (Breakout.analogRead(ANALOG_A1)>LICK_THRESHOLD){
    if (!is_licking){
      // animal just started licking
      start_lick_timestamp = micros();
      is_licking = true;
      digitalWrite(LEDR, LOW);
    }
  } else if (is_licking){
    is_licking = false;
    digitalWrite(LEDR, LOW);
    end_lick_timestamp = micros();
    lickPckgBase = constr_pckg_base_str("L", lickPckgID, end_lick_timestamp);
    lickPckgValue = String(end_lick_timestamp-start_lick_timestamp);
    // serialwrite_package(lickPckgBase, lickPckgValue);
    lickPckgID++;
  } else{
    digitalWrite(LEDR, HIGH);
  }
  /*
  ================================================================================
   (5) M4 action check and sending
  ================================================================================
  */
  processM4actions();

  /*
  ================================================================================
   (6) screen
  ================================================================================
  */
  // screen didn't update and locked the m7, so disabled atm

  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Virtual Reality"));
  display.println("Row:" + String(rCum));
  display.println("Yaw:" + String(yCum));
  display.println("Pitch:" + String(pCum));
  display.display();
  // Serial.println("reached display end");
  //*/

  // delay(2);
  }
#endif









/*
================================================================================
  M4 variables - read to-be-executed actions from shared memoery (written by M7)
  execute them, and write back to shm that they have been executed
  M4 controls output:  opening the reward valve, init playing sound, opens the 
  main air valve, and delivers air puff
================================================================================
*/
#ifdef CORE_CM4
#define HSEM_ID 0
#define HSEM_PROCESS 0

char local_startNewAction;  // can be A S F P
uint16_t local_actionLength;  // relevant for S and P
uint16_t local_actionDelay;  // relevant for S
char local_startedM4Action; // can be A S F P

bool PWM_state = false;
#define HALF_PERIOD_REWARD 360
#define HALF_PERIOD_PUNISH 1020

#define SOUND_LENGTH 400 // ms
#define PAUSE_LENGTH 100 // ms

// uint32_t sound_end_t;
// uint32_t reward_start_t;
// uint32_t reward_end_t;
// uint32_t next_change_sound_t = 0;

bool progress = false;
bool new_action=false;

volatile struct shared_data * const buff = (struct shared_data *)0x38001000;

breakoutPin REWARD_PIN = GPIO_4; // reward/ valve pin
/*
================================================================================
  M4 function
================================================================================
*/

void play_sound(){
  
  /*if (reward_sound || punish_sound){
    uint32_t current_t = micros();
    if ((current_t-next_change_sound_t)>0){
      Breakout.digitalWrite(PWM3, (PWM_state) ? HIGH : LOW);
      PWM_state = !PWM_state;
      if (reward_sound){
        next_change_sound_t = current_t + HALF_PERIOD_REWARD;
      } else{
        next_change_sound_t = current_t + HALF_PERIOD_PUNISH;
      }
    }
  }*/
}

/*
================================================================================
  M4 setup
================================================================================
*/
void setup() {
  
  // sound_end_t = 0;
  // reward_start_t = 0;
  // reward_end_t = 0;
  
  // pin setup
  Breakout.pinMode(REWARD_PIN, OUTPUT);
  Breakout.digitalWrite(REWARD_PIN, LOW);

  Breakout.pinMode(PWM7, OUTPUT);
  Breakout.pinMode(GPIO_5, OUTPUT); //sound TTL
  Breakout.pinMode(PWM2, OUTPUT);  //reward TTL
  
  //Breakout.analogWrite(PWM7, 255);
  Breakout.digitalWrite(GPIO_5, LOW);

  if (REWARD_PIN == -1) {
    // error indication if reward is not attached to board
    for (int i=0;i<10;i++){
      digitalWrite(LEDR, LOW);
      delay(500);
      digitalWrite(LEDR,HIGH);
      delay(500);
    }
  }

  Breakout.pinMode(PWM4, OUTPUT); //air puff
  
  // Breakout.pinMode(PWM2, OUTPUT);
  // Breakout.pinMode(PWM0, OUTPUT);
}

// void loop2() {
//   // test loop to test pins, change from loop2 to loop and change regular loop to loop2
//   Breakout.digitalWrite(PWM2, HIGH);
//   Breakout.digitalWrite(PWM1, HIGH);
//   Breakout.digitalWrite(PWM0, HIGH);
//   delay(200);
//   Breakout.digitalWrite(PWM2, LOW);
//   Breakout.digitalWrite(PWM1, LOW);
//   Breakout.digitalWrite(PWM0, LOW);
  
//   Breakout.analogWrite(PWM7, 20);
//   delay(1000);
//   Breakout.analogWrite(PWM7, 0);

//    //photodiodePckgBase = "<{N:BV,ID:21731,T:" + String(micros()) + ",V:{R:0,Y:0,P:0}}>";
//    //Serial.println(photodiodePckgBase);
//    //photodiodePckgBase = "<{N:BV22323,ID:21731,T:" + String(micros()) + ",V:{R:0,Y:0,P:0}}>";
//    //Serial.println(photodiodePckgBase);
// }
/*
================================================================================
  M4 loop
================================================================================
*/

void loop2(){
  Breakout.digitalWrite(PWM4, (PWM_state) ? HIGH : LOW);
  PWM_state = !PWM_state;
  delay(1020);

}

void loop(){
  while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
  
  // read new action command from shared memory
  local_startNewAction = buff->startNewAction;

  if (local_startNewAction != '\0'){
    digitalWrite(LEDR, LOW);
    local_actionLength = buff->actionLength;
    local_actionDelay = buff->actionDelay;
    local_startedM4Action = buff->startedM4Action;
    
    buff->startNewAction = '\0';
    digitalWrite(LEDR, HIGH);
  }
  HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS); // release HW semaphore
  
  if (local_startNewAction == 'S') {

    // message M7 that action (sucess-sound) is being executed
    while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
    buff->startedM4Action = 'S';
    HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

    // LEDS are lighting up
    digitalWrite(LEDG, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    
    Breakout.digitalWrite(GPIO_5, HIGH);  // TTL HIGH reward sound start
    for (uint16_t i=0; i < (SOUND_LENGTH*1000) / HALF_PERIOD_REWARD; i++){
      Breakout.digitalWrite(PWM7, (PWM_state) ? HIGH : LOW);
      PWM_state = !PWM_state;
      delayMicroseconds(HALF_PERIOD_REWARD);
    }
    
    delay(PAUSE_LENGTH);
    
    for (uint16_t i=0; i < (SOUND_LENGTH*1000) / HALF_PERIOD_REWARD; i++){
      Breakout.digitalWrite(PWM7, (PWM_state) ? HIGH : LOW);
      PWM_state = !PWM_state;
      delayMicroseconds(HALF_PERIOD_REWARD);
    }
    Breakout.digitalWrite(GPIO_5, LOW); // TTL LOW reward sound stop

    // sleep for sound_delay
    delay(local_actionDelay);

    // message M7 that action (reward-opening) is being executed
    while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
    buff->startedM4Action = 'R';
    HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);
    
    // if led low, it lights up, so inverted
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    // turn on reward valve
    Breakout.digitalWrite(PWM2, HIGH); // TTL HIGH vaalve opened
    digitalWrite(REWARD_PIN, HIGH);
    delay(local_actionLength);
    digitalWrite(REWARD_PIN, LOW);
    Breakout.digitalWrite(PWM2, LOW); // TTL LOW vaalve opened
    
    digitalWrite(LEDG, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);



  } else if (local_startNewAction == 'F') {

    // message M7 that action (reward-opening) is being executed
    while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
    buff->startedM4Action = 'F';
    HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

    Breakout.digitalWrite(GPIO_5, HIGH); // TTL HIGH failure sound start
    for (uint16_t i=0; i < (SOUND_LENGTH*2000) / HALF_PERIOD_PUNISH; i++){
      Breakout.digitalWrite(PWM7, (PWM_state) ? HIGH : LOW);
      PWM_state = !PWM_state;
      delayMicroseconds(HALF_PERIOD_PUNISH);
    }
    Breakout.digitalWrite(GPIO_5, LOW); // TTL LOW failure sound stop



  } else if (local_startNewAction == 'P') {

    // message M7 that action (reward-opening) is being executed
    while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
    buff->startedM4Action = 'P';
    HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

    // airpuff
    Breakout.digitalWrite(PWM4, HIGH);  // Airpuff valve open
    Breakout.digitalWrite(PWM8, HIGH);  // Airpuff start TTL HIGH
    delay(local_actionLength);
    Breakout.digitalWrite(PWM4, LOW); // Airpuff valve closed
    Breakout.digitalWrite(PWM8, LOW); // Airpuff stop TTL LOW
  }

}

// /*
// ================================================================================
// M4 old loop
// ================================================================================
// */

// void loop_old() {
//   // check HW semaphore
//   while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
//   // read new action command from shared memory
//   if (buff->new_data){
//     digitalWrite(LEDG, LOW);
//     local_reward_length = buff->reward_length;
//     local_sound_delay = buff->sound_delay;
//     local_sound_length = buff->sound_length;
    
//     buff->new_data = false; // read  
//     new_action=true;
//     digitalWrite(LEDG, HIGH);
//   }
//   HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS); // release HW semaphore

//   // execute the action, next iteration progress=True
//   if (new_action){
//     // first play sound and set sound_end_t
//     new_action=false; // tag set by the M7 action, setup for the complicated time checking
//     while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
//     buff->sound = true;
//     buff->reward = false;
//     buff->air = false;
//     buff->new_action_m4 = true;
//     HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

//     //digitalWrite(pinout[juiceline], HIGH);
//     digitalWrite(LEDG, LOW);
//     digitalWrite(LED_BUILTIN, LOW);
//     //digitalWrite(REWARD_PIN, HIGH);
    
//     /*if (counter%2==0){
//       Breakout.analogWrite(PWM7, 0);
//       delayMicroseconds(1000);
//       Breakout.analogWrite(PWM7, 255);
//     } else {
//       Breakout.analogWrite(PWM3, 0);
//       delayMicroseconds(1000);
//       Breakout.analogWrite(PWM3, 255);
      
//     }
//     counter++;*/
//     /*Breakout.analogWrite(PWM7, 0);
//     delayMicroseconds(40000);
//     Breakout.analogWrite(PWM7, 255);

//     Breakout.analogWrite(PWM3, 255);
//     delayMicroseconds( 0000);
//     Breakout.analogWrite(PWM3, 0);*/
    
//     progress = true;
//     sound_end_t = millis() + local_sound_length;
//     reward_start_t = sound_end_t + local_sound_delay;
//     reward_end_t = reward_start_t + local_reward_length;
//   }
//   // output seqnece in proggress
//   else if (progress){
    
//     // play sound function
//     play_sound();

//     if (sound_end_t!=0 && millis()>=sound_end_t){
//       digitalWrite(LEDR, LOW);
//       digitalWrite(LED_BUILTIN, HIGH);
//       //Breakout.analogWrite(PWM7, 0);
//       sound_end_t = 0;
//       reward_sound = true;

//       while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
//       buff->sound = false;
//       buff->new_action_m4 = true;
//       HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);
//     }
//     if (reward_start_t!=0 && millis()>=reward_start_t){
//       while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
//       buff->sound = false;
//       buff->reward = true;
//       buff->air = false;
//       buff->new_action_m4 = true;
//       HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

//       // if led low, it lights up, so inverted
//       digitalWrite(LEDG, LOW);
//       digitalWrite(LEDR, HIGH);
//       digitalWrite(LED_BUILTIN, LOW);
//       digitalWrite(REWARD_PIN, HIGH);
//       //Breakout.analogWrite(PWM7, 0);
//       reward_sound = false;
//       punish_sound = true;

//       reward_start_t = 0;
//     }
//     if (reward_end_t!=0 && millis()>=reward_end_t){
//       while(HAL_HSEM_Take(HSEM_ID, HSEM_PROCESS) != HAL_OK){};
//       buff->sound = false;
//       buff->reward = false;
//       buff->air = false;
//       buff->new_action_m4 = true;
//       HAL_HSEM_Release(HSEM_ID, HSEM_PROCESS);

//       //digitalWrite(pinout[juiceline], HIGH);
//       digitalWrite(LEDG, HIGH);
//       digitalWrite(LED_BUILTIN, HIGH);
//       digitalWrite(REWARD_PIN, LOW);
//       //Breakout.analogWrite(PWM7, 0);
//       punish_sound = false;
//       progress = false;
//       reward_end_t=0;
//     }
//   }
//   else {

//     // TODO
//     // switch PWM4/VALVE_5 to open main air valve
//   }
// }

// #ifdef CORE_CM4
#endif