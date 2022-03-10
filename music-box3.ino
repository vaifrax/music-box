/*
music box
by Marcel Lancelle, 2019
use Arduino Nano, ATmega328P (Old Bootloader) on COM3
*/

/*

// sd:/mp3/0001*.mp3
// sd:/mp3/0002*.mp3
// sd:/mp3/0003*.mp3

*/

/*
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(20);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(400);                       // wait for a second
}
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include "DFMiniMp3.h"
#include "LowPower.h"

//#define DEBUG_OUTPUT

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float ac_x, ac_y, ac_z;

// to calibrate: measure x/y/z min/max due to gravity, manually correct values to be spaced out by 32800 (guess by Marcel)
const int16_t ACC_X_MIN = -15550;
const int16_t ACC_X_MAX =  17250;
const int16_t ACC_Y_MIN = -16400;
const int16_t ACC_Y_MAX =  16400;
const int16_t ACC_Z_MIN = -17300;
const int16_t ACC_Z_MAX =  15500;
const int16_t ACC_X_BIAS = (ACC_X_MIN + ACC_X_MAX) / 2;
const int16_t ACC_Y_BIAS = (ACC_Y_MIN + ACC_Y_MAX) / 2;
const int16_t ACC_Z_BIAS = (ACC_Z_MIN + ACC_Z_MAX) / 2;
const float ACC_NORMALIZE_FACT = 1.0/16400.0;

const int8_t MY_VOLUME = 11;

int8_t state = 0;
bool battery_voltage_low = false;
int16_t timeout_counter = 0;

// implement a notification class,
// its member methods will get called 
//
class Mp3Notify
{
public:
  static void OnError(uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);

    state = 99; // restart playing a song
  }

  static void OnPlayFinished(uint16_t globalTrack)
  {
    Serial.println();
    Serial.print("Play finished for #");
    Serial.println(globalTrack);   

    state = 99; // restart playing a song
  }

  static void OnCardOnline(uint16_t code)
  {
    Serial.println();
    Serial.print("Card online ");
    Serial.println(code);     

    state = 99; // restart playing a song
  }

  static void OnCardInserted(uint16_t code)
  {
    Serial.println();
    Serial.print("Card inserted ");
    Serial.println(code); 

    state = 99; // restart playing a song
  }

  static void OnCardRemoved(uint16_t code)
  {
    Serial.println();
    Serial.print("Card removed ");
    Serial.println(code);  
  }
};

// instance a DFMiniMp3 object, 
// defined with the above notification class and the hardware serial class
//
//DFMiniMp3<HardwareSerial, Mp3Notify> mp3(Serial1);

// Some arduino boards only have one hardware serial port, so a software serial port is needed instead.
// comment out the above definition and uncomment these lines
SoftwareSerial secondarySerial(10, 11); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(secondarySerial);

uint8_t prev_track_num = 99;
void playSide(uint8_t side) { // side 0..5; 6 reserved for low voltage
  timeout_counter = 0;
  
  //mp3.playMp3FolderTrack(1); // sd:/mp3/0001*.mp3
  uint8_t folder = side+1;
  uint16_t track_num = mp3.getFolderTrackCount(folder); // folder on sd card is 01, 02, .., 06 on older chips = my chip (or 001, 002, ..., 006 on newer chips)
  uint8_t track = random(1, track_num+1); // for 3 tracks (track_num == 3), this generates 1, 2 or 3

  if (track == prev_track_num) {
    // if box is still on the same side, this would mean the next song is the same as the previous one
    // make it less likely to repeat songs with one more random number
    track = random(1, track_num+1);
  }
  prev_track_num = track;

  Serial.print("play song in folder ");
  Serial.print(folder);
  Serial.print(" and track number ");
  Serial.print(track);
  Serial.print("/");
  Serial.println(track_num);
  //mp3.playFolderTrack(folder, track); // sd:/001/0001.*.mp3  this is for newer chips
  mp3.playFolderTrack16(folder, track); // sd:/01/0001.*.mp3  this is for older chips = my chip
}

int getAccurateVoltage() {
  getVoltage(); // first read is reported to be too low, second one is accurate, apparently
  return getVoltage();
}

// Read the voltage of the battery the Arduino is currently running on (in millivolts)
int getVoltage(void) {
  const long InternalReferenceVoltage = 1091L; // Adjust this value to your boards specific internal BG voltage x1000
  #if defined(AVR_ATmega1280) || defined(AVR_ATmega2560) // For mega boards
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #else // For 168/328 boards
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #endif
  ADCSRA |= _BV( ADSC ); // Start a conversion 
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) ); // Wait for it to complete
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // Scale the value; calculates for straight line value
  return results*10; // convert from centivolts to millivolts
}

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)

//Marcel: tried this, doesn't work (some acceleration data not updated?), no idea why
//  Wire.write(32+8);     // set to zero (wakes up the MPU-6050)
//  Wire.write(0x6C);  // PWR_MGMT_2 register
//  Wire.write(128+7);  // 64 means LP_WAKE_CTRL=1 which means wake-up frequency is 5 Hz, 7 means disabling all gyros
  
  Wire.endTransmission(true);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  //Serial.begin(9600);
  Serial.begin(115200);

  randomSeed(analogRead(0)); // seed with an unconnected (?) pin for different random sequences at each start

  mp3.begin();

  uint16_t volume = mp3.getVolume();
  Serial.print("volume ");
  Serial.println(volume);
  mp3.setVolume(MY_VOLUME); // 0-30

  uint16_t count = mp3.getTotalTrackCount();
  Serial.print("files ");
  Serial.println(count);

  playSide(0);
}

void loop() {
  timeout_counter++;
  if (timeout_counter > 10*400) { // count 400 is roughly one minute
    Serial.println("timeout, start playing new song");
    state = 99; // restart playing a song
  }
  // also check for mp3 player state and battery voltage every about 3 seconds
  if (timeout_counter % 20 == 0) {
    if (mp3.getStatus() != 513) { // 513 means playing, 512 means finished/not playing
      Serial.println("not playing (error?), start playing new song");
      state = 99;
    }

    //Serial.println(getAccurateVoltage());
    if (!battery_voltage_low && (getAccurateVoltage() < 3300)) {
      battery_voltage_low = true;
    }
  }

  if (battery_voltage_low) {
    if (state != 6) {
      state = 6;
      playSide(state);
    }
  } else {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
  
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    //Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
    //AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    //AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    //AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
    ac_x = (AcX - ACC_X_BIAS) * ACC_NORMALIZE_FACT;
    ac_y = (AcY - ACC_Y_BIAS) * ACC_NORMALIZE_FACT;
    ac_z = (AcZ - ACC_Z_BIAS) * ACC_NORMALIZE_FACT;
  
    //Serial.print(millis()); Serial.print(",");
    #ifdef DEBUG_OUTPUT
    Serial.print(ac_x); Serial.print(",");
    Serial.print(ac_y); Serial.print(",");
    Serial.println(ac_z);
    #endif
    //Serial.print("AcX="); Serial.print(AcX);
    //Serial.print(" AcY="); Serial.print(AcY);
    //Serial.print(" AcZ="); Serial.print(AcZ);
    //Serial.print(" Tmp="); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    //Serial.print(" GyX="); Serial.print(GyX);
    //Serial.print(" GyY="); Serial.print(GyY);
    //Serial.print(" GyZ="); Serial.println(GyZ);
  
    //if (AcX > 0) digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    //  else digitalWrite(LED_BUILTIN, LOW);
  
    mp3.setVolume(MY_VOLUME); // 0-30
  
    if ((state != 0) && (ac_x > 0.8)) {
      state = 0;
      playSide(state);
    }
    if ((state != 1) && (ac_x < -0.8)) {
      state = 1;
      playSide(state);
    }
  
    if ((state != 2) && (ac_y > 0.8)) {
      state = 2;
      playSide(state);
    }
    if ((state != 3) && (ac_y < -0.8)) {
      state = 3;
      playSide(state);
    }
    if ((state != 4) && (ac_z > 0.8)) {
      state = 4;
      playSide(state);
    }
    if ((state != 5) && (ac_z < -0.8)) {
      state = 5;
      playSide(state);
    }
  }

  mp3.loop(); 

  //delay(205);
  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
}
