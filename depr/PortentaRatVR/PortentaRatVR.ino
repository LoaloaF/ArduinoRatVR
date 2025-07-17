#include "mbed.h"
#include "Arduino.h"
#include <SPI.h>
#include <avr/pgmspace.h>
#include <Arduino_PortentaBreakout.h>

//#define byte uint8_t

byte initComplete=0;
byte Motion = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xydat[2];
int xy2dat[2];
double dP;
double dR;
double dY;
int pCum = 0;
int rCum = 0;
int yCum = 0;

breakoutPin ncs = SPI1_CS;
breakoutPin ncs2 = I2S_WS;

// This is temporary lines for the pump, -> move to M4 core
const long nchan = 4;
breakoutPin pin = GPIO_4;
breakoutPin EN = GPIO_6;
// temporary end

//const int pVelPin = ANALOG_A7;
//const int rVelPin = ANALOG_A6;
//const int yVelPin = ANALOG_A5;

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

//const double px1 = 1;
//const double rx1 = 0;
//const double yx1 = 0;
//const double py1 = 0;
//const double ry1 = 0;
//const double yy1 = -1.557;
//const double px2 = 0;
//const double rx2 = -1;
//const double yx2 = 1.1918;
//const double py2 = 0;
//const double ry2 = 0;
//const double yy2 = 0;

// Registers
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
//
extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];

const int num_variables = 5;
unsigned int* communication_ptr_array[num_variables];

void setup() {
  bootM4();
  for(int i=0;i<num_variables;i++){
    communication_ptr_array[i] = (unsigned int*)malloc(sizeof(unsigned int));
    *communication_ptr_array[i] = 0;
  }

  Serial.begin(115200);
  //  analogWriteFrequency(pVelPin,11500);
  //  analogWriteFrequency(rVelPin,11500);
  //  analogWriteFrequency(yVelPin,11500);
  //  analogWriteResolution(12);
  //  pinMode (ncs, OUTPUT);
  //  pinMode (ncs2, OUTPUT);

 int res = Breakout.pinMode(ncs, OUTPUT);
 int res1 = Breakout.pinMode(ncs2, OUTPUT);
 int reward = Breakout.pinMode(pin, OUTPUT);
 int en= Breakout.pinMode(EN, OUTPUT);
 
   if (reward == -1) {
    Serial.println("Error: reward is not connected");
  }
  
  Breakout.digitalWrite(pin, LOW);
  Breakout.digitalWrite(EN , HIGH);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  //  SPI.setDataMode(SPI_MODE3);
  //  SPI.setBitOrder(MSBFIRST);
  //  SPI.setClockDivider(2);
  //  
  delay(1000);
  performStartup();
  delay(10);
  performStartup2();
  delay(10);
  //adns_write_reg(REG_Configuration_I, 0x29); // maximum resolution
  adns_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  delay(10);
  adns2_write_reg(REG_Configuration_I, 0x09); // default resolution
  //adns2_write_reg(REG_Configuration_I, 0x01); // minimum resolution
  
  core_share_init(); // must be called on one of the cores, here M7
  
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LEDR, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LEDR, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LEDR, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LEDR, LOW);
  

  delay(1000);  
  dispRegisters();
  delay(10);
  dispRegisters2();
  delay(1000);
  initComplete=9;

}

void adns_com_begin(){
  digitalWrite(ncs, LOW);
}

void adns2_com_begin(){
  digitalWrite(ncs2, LOW);
}

void adns_com_end(){
  digitalWrite(ncs, HIGH);
}

void adns2_com_end(){
  digitalWrite(ncs2, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

byte adns2_read_reg(byte reg_addr){
  adns2_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns2_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns2_write_reg(byte reg_addr, byte data){
  adns2_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns2_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 1...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}

void adns2_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 2...");
  // set the configuration_IV register in 3k firmware mode
  adns2_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns2_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns2_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns2_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns2_com_end();
}


void performStartup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(10);

  Serial.println("Optical Chip 1 Initialized");
}

void performStartup2(void){
  adns2_com_end(); // ensure that the serial port is reset
  adns2_com_begin(); // ensure that the serial port is reset
  adns2_com_end(); // ensure that the serial port is reset
  adns2_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns2_read_reg(REG_Motion);
  adns2_read_reg(REG_Delta_X_L);
  adns2_read_reg(REG_Delta_X_H);
  adns2_read_reg(REG_Delta_Y_L);
  adns2_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns2_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0_2 = adns2_read_reg(REG_LASER_CTRL0);
  adns2_write_reg(REG_LASER_CTRL0, laser_ctrl0_2 & 0xf0 );
  
  delay(10);

  Serial.println("Optical Chip 2 Initialized");
}


void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  };
  byte regres;

  digitalWrite(ncs,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs,HIGH);
}

void dispRegisters2(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  char* oregname[] = {
    "Product_ID2","Inverse_Product_ID2","SROM_Version2","CPI2"  };
  byte regres;

  digitalWrite(ncs2,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs2,HIGH);
}

int readXY(int *xy){
  digitalWrite(ncs,LOW);
  
  Motion = (adns_read_reg(REG_Motion) & (1 << 8-1)) != 0;
  xL = adns_read_reg(REG_Delta_X_L);
  xH = adns_read_reg(REG_Delta_X_H);
  yL = adns_read_reg(REG_Delta_Y_L);
  yH = adns_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  digitalWrite(ncs,HIGH);     
}

int readXY2(int *xy){
  digitalWrite(ncs2,LOW);
  
  Motion = (adns2_read_reg(REG_Motion) & (1 << 8-1)) != 0;
  xL = adns2_read_reg(REG_Delta_X_L);
  xH = adns2_read_reg(REG_Delta_X_H);
  yL = adns2_read_reg(REG_Delta_Y_L);
  yH = adns2_read_reg(REG_Delta_Y_H);
  xy[0] = (xH << 8) + xL;
  xy[1] = (yH << 8) + yL;

  if(xy[0] & 0x8000){
    xy[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy[1] & 0x8000){
    xy[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  
  digitalWrite(ncs2,HIGH);     
}

void loop() {
    if (0 < Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (0==cmd.length()) return;

      String result = "P1";  // Command not understood
      switch (cmd.charAt(0)) {

        case 'Q':  // query current state
          result = "READY";
          break;
        
        case 'R':  // reset
          result = "OK";
          break;

        case 'Y': {  // deliver reward, Y<juice line>,<duration>,<num reward>,<pause time>
          int comma1 = cmd.indexOf(',',1);
          int comma2 = cmd.indexOf(',',comma1+1);
          int comma3 = cmd.indexOf(',',comma2+1);
          if (comma1<1 || comma2<1 | comma3<1) {
            result = "P3";  // Parameter error
            break;
          }

          unsigned int juiceline = cmd.substring(1,comma1).toInt() - 1;  // 0-based
          unsigned int duration = cmd.substring(comma1+1,comma2).toInt();
          unsigned int numreward = cmd.substring(comma2+1,comma3).toInt();
          unsigned int pausetime = cmd.substring(comma3+1).toInt();
          
          *communication_ptr_array[0] = 1-*communication_ptr_array[0];
          *communication_ptr_array[1] = juiceline;
          *communication_ptr_array[2] = duration;
          *communication_ptr_array[3] = numreward;
          *communication_ptr_array[4] = pausetime;

          //put_to_M4(*communication_ptr_array, num_variables);
          
          /*if (nchan<=juiceline) {
            result = "P2";  // Solenoid does not exist
            break;
          }

          for (size_t m=1; m<=numreward; m++) {
            digitalWrite(LED_BUILTIN, LOW);
            //digitalWrite(pinout[juiceline], HIGH);
            digitalWrite(pin , HIGH);
            delay(duration);
            digitalWrite(pin, LOW);
            digitalWrite(LED_BUILTIN, HIGH);
            if (m<numreward) delay(pausetime);
          }*/
          result = "OK";
          break;
        }
    }
    
    Serial.write((result + '\n').c_str());
    }
  
    readXY(&xydat[0]);
    readXY2(&xy2dat[0]);
    dP = px1*xydat[0] + py1*xydat[1] + px2*xy2dat[0] + py2*xy2dat[1];
    dR = rx1*xydat[0] + ry1*xydat[1] + rx2*xy2dat[0] + ry2*xy2dat[1];
    dY = yx1*xydat[0] + yy1*xydat[1] + yx2*xy2dat[0] + yy2*xy2dat[1];
    pCum = pCum*0.9 + dP;
    rCum = rCum*0.9 + dR;
    yCum = yCum*0.9 + dY;
  //    analogWrite(pVelPin,dP+2048);
  //    analogWrite(rVelPin,dR+2048);
  //    analogWrite(yVelPin,dY+2048);

  Serial.println(String(pCum) + "_" + String(rCum) + "_" + String(yCum));
    
    delay(10);
  }