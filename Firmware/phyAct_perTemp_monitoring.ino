//Author: Antoni López Giménez
//University: UPC (2021)

//Useful Libraries
#include "ArduinoLowPower.h"
#include <Wire.h> //Library to use the I2C 
#include "RTClib.h" //Library to use DS3231
#include <RTCZero.h> // Library to use internal RTC 
#include <Adafruit_SleepyDog.h>//Library to use intern Watchdog timer 
#include "ClosedCube_TMP116.h"
#include "mma8652.h" //Library to use the accelerometre (MMA8652FCR1)

//States of the program and commands to send
#define STOP 0 //S
#define START 1 //A
#define SEND 2 //E
#define RESET 3 //N
#define READ 4 //R
 

//Microcontroller pin assigments
const int LED = 13; //Help you to know if the Arduino bootloader is correctly programmed

//Useful variables
byte State = 0; //Solve the actual state

//Temperature Sensor definitions
uint8_t ADDR_GND =  0x48;   // Skin sensor adress
uint8_t ADDR_VCC =  0x49;   // Internal sensor adress
//TMP117 SKIN_tmp(ADDR_GND); //Adress assigment
//TMP117 INT_tmp(ADDR_VCC); // Adress assigment
ClosedCube::Sensor::TMP116 SKIN_tmp;
ClosedCube::Sensor::TMP116 INT_tmp;

//Internal RTC definitions
RTCZero intRTC;
boolean TEST = false; //Variable for controling acquisition or send data (in acquisition mode goes to sleep TEST=0)
// Set how often alarm goes off here
const byte alarmSeconds = 10;
const byte alarmMinutes = 0;
const byte alarmHours = 0;
volatile bool alarmFlag = true; // Start awake

//External RTC (DS3231) definitions (0x68 default device address)
RTC_DS3231 extRTC;
const char daysOfTheWeek [7][12]={"Sunday","Monday","Tuesday","Wednesday", "Thursday", "Friday", "Saturday"};

//Accelerometer definitions (0x1D default device address)
#define ACC_INT 11
#define actCount 0x80
#define accTHS 0x40
MMA8652 accel= MMA8652(ACC_INT);

//EEPROM MEMORY 
//EEPROM has I2C addresses 0x50, 0x51, 0x52 and 0x53 (0x50 | 0x00, 0x50 | 0x01,0x50 | 0x02 and 0x50 | 0x03)
#define EEPROM_DATA_ADD 0x50 // Address of the first 1024 page M24M02DRC EEPROM data buffer, 2048 bits (256 8-bit bytes) per page
#define EEPROM_IDPAGE_ADD 0x58 // Address of the single M24M02DRC lockable ID page of the EEPROM
#define length_Fblock 12 //Length of first block of data (16 bytes)[0..15]
#define length_Sblock 8 //Length of second blocks of data (12 bytes) [0..11]
#define L_MSmax 256 //Maximum of pages per block and maximum data per page
#define maxBlocks 31 //Maximum number of blocks can be sended per page
#define Bmax 3
uint8_t Badd=0, MSadd=0, LSadd=0; //Badd=Block address (0-3) ; MSadd=MSB(page 0-255) ; LSadd=LSB(0-255)
bool EEPROM_empty = true;
int currentBlocks, sendedBlocks=31744; //(==31744 if you want read memory)

//Obtained data
struct data{ //Struct with all the variables that we will need
  byte day;
  byte month;
  int year;
  byte hour;
  byte min;
  byte sec;
  float int_temp;
  float pat_temp;
  bool phy_act;
};
typedef struct data Data; 
Data ReadData; //data to print
Data ObtData; //Obtanined data to storage
uint8_t clearData[256]; //Variable to reset the EEPROM memory
uint8_t receiveData[L_MSmax]; //Variable to receive byte to byte from EEPROM memory

//Useful variables
byte prevHour;
byte prevMin;
int tt1=0, tt2=0;
int countdownMS;


void setup() {
 
  Wire.begin();
  Serial.begin(115200); //opens serial port, sets data rate to 115200 bps
  while (!Serial) ;
  Serial.println("Setup");
  pinMode(LED,OUTPUT);
  
  //Temperature sensors initialize (ver funciones)
  SKIN_tmp.address(ADDR_GND);
  INT_tmp.address(ADDR_VCC);
    
  //External RTC initialize
  if (! extRTC.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  if (extRTC.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    extRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }else{
    extRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //Internal RTC initialize if TEST=FALSE
  if(TEST==false){
    intRTC.begin(); // Set up clocks and such
    resetAlarm();  // Set alarm
    intRTC.attachInterrupt(alarmMatch); // Set up a handler for the alarm
    digitalWrite(LED,LOW);
    Serial.println("DORMIREMOS");
    delay(1000);
    //countdownMS=Watchdog.enable(alarmSeconds*1000);
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, onWakeup, CHANGE);
    State=STOP;
  }else{
    digitalWrite(LED,HIGH);
  }
  
  
  //Acelerometre initialize 0x1D device adress
  //cambiar init  a 8g
  accel.init();
  delay(1);
  accel.write(F_SETUP_REG,F_MODE_FILL); //Allow mode 1 of FIFO (Fill) if we need a circular fifo you change the second parameter to F_MODE_CIRCULAR for more details read the information in github repository
  //accel.write(OFF_X_REG, X_OFFSET_X); //Congifuration of X-axis offset 
  //accel.write(OFF_Y_REG, Y_OFFSET_y); //Congifuration of Y-axis offset 
  //accel.write(OFF_Z_REG, Z_OFFSET_Z); //Congifuration of Z-axisoffset 
  // Enable Free-Fall detection interrupt
  accel.enableFreeFallInt(accTHS); //the hexadecimal parameter is the sensibility FF_MT_THS
  delay(1);
  
  
}

void loop() {

 byte command;

 switch (State){
  
  case STOP:{
    
       
    if(Serial.available()>0){
    
      command=Serial.read();
      switch (command){
        case 'S':{
          State= STOP;
          Serial.println("STOPPING");
          delay(100);
          //Watchdog.disable();
          break;
        }
        case 'A':{
          State=START;
          Serial.println("CHANGING TO ACQUISITION MODE");
          delay(100);
          //countdownMS=Watchdog.enable(alarmSeconds*1000);
          break;          
        }
        case 'E':{
          
          State=SEND; 
          break;
        }
        case 'N':{
          Serial.println("RESETTING THE MEMORY");
          State=RESET; 
          break;
        }
        case 'R':{          
          State=READ;
          break;          
        }
      }
      // Plot menu
      Serial.println("  ");
      Serial.println("  ");
      Serial.println("S Stop Acquisition");
      Serial.println("A Start Acquisition, goes to sleep");
      Serial.println("E Send acquired data");
      Serial.println("N Reset memory (acquired data)");
      Serial.println("R Read data acquired");
      Serial.println("  ");
      Serial.println("  ");
    }
    break;
  }//case:STOP

  case START:{
   
  if(Badd>=4){ //Exceed the number of blocks
    EEPROM_empty=false;
    Serial.println("Full memory, STOP acquisition");
    Serial.print("Blocks sended: ");
    Serial.println(sendedBlocks);
    State=STOP;
    Watchdog.disable();
    break;
  }

  if (TEST==false){
    
    //AQUI HAY QUE DORMIRLO
    Serial.println("Zzzzz..."); 
    digitalWrite(LED, LOW);   
    if(alarmFlag == true) alarmFlag = false; //Clear flag
    resetAlarm();
    //intRTC.standbyMode();
    LowPower.sleep(alarmSeconds*1000);
    USBDevice.attach(); 
    //Watchdog.reset();    
   
    Serial.println("Hello World");   
    readSensors();
    State=SEND;
    command=Serial.read();
    if(command=='S'){
      State=STOP;
      break;
    }
    
  }else{

    //AQUI NO DUERME   
    readMemory();
    //readSensors();
    testReadSensors();   
    State=SEND;
    command=Serial.read();
    if(command=='S'){
      State=STOP;
      break;
    }
    
  }
    
  break;
  
  }//case:START
  
  case SEND:{
    
    if(LSadd==0){
      
      M24M02DRCwriteBytes(EEPROM_DATA_ADD | Badd, MSadd, LSadd, true, ObtData); //OWN function to send data
      delay(10);// It takes a maximum of 10 ms for a read or write operation; the EEPROM won't respond until the operation is done
      LSadd=LSadd+length_Fblock; //increasing the number of bytes written to the page
      currentBlocks++;
      sendedBlocks++;
           
    }else{
      
      M24M02DRCwriteBytes(EEPROM_DATA_ADD | Badd, MSadd, LSadd, false, ObtData); 
      delay(10);
      LSadd= LSadd+length_Sblock;
      currentBlocks++;
      sendedBlocks++;
      if(currentBlocks == maxBlocks){
        /*Serial.println(sendedBlocks);
        Serial.println(Badd);
        Serial.println(MSadd);
        Serial.println(LSadd);*/
        LSadd=0;
        MSadd++;
        currentBlocks=0; 
               
        if(MSadd == 0){
          Badd++;
          Serial.println("BLOQUE ACTUALIZADO");
          delay(500);           
        }
      }
      /*Serial.println(sendedBlocks);
      Serial.println(Badd);
      Serial.println(MSadd);
      Serial.println(LSadd);*/
    }

    if(TEST==false){
      State=START;
      break;
    }else{
      State=STOP;
      break;
    }
    
  }//case:SEND
  
  case RESET:{

    //RESET EEPROM MEMORY
    for(uint8_t B=0; B < 4; B++){ //fixing block
      for(uint8_t M=0; M < (L_MSmax-1); M++){ //fixing MSB
        M24M02DRCclear(EEPROM_DATA_ADD | B, M, 0,clearData);
        delay(10);
      }
      M24M02DRCclear(EEPROM_DATA_ADD | B, 255, 0,clearData);
      delay(10);
    }
    EEPROM_empty=true;
    
    //RESET ACCELEROMETRE

    State=STOP;
    break;
  }//case:RESET
  
  case READ:{
  int s=0;
  uint8_t B=0;
  while(B<=Bmax){    
      for(uint8_t M=0; M < (L_MSmax-1) ; M++){    //&& s<=sendedBlocks  
        M24M02DRCreadBytes(EEPROM_DATA_ADD | B, M, 0);
        delay(10);
        s+=31;          
        printData();
       }
       M24M02DRCreadBytes(EEPROM_DATA_ADD | B, 255, 0);
      delay(10);
      s+=31;          
      printData();
      B++;
    }
    State=STOP;
    break;
  }//case:READ
   
 }//switch
}//loop


/////////////////////
//USEFUL FUNCTIONS//
///////////////////

//Reset Internal RTC ALARM
void resetAlarm(void){

  //Obtain current time
  //DateTime now = current_Time();
  
  //Set current Time and Date in the internal RTC
  //intRTC.setTime(now.hour(),now.minute(),now.second());
  //intRTC.setDate(now.year(),now.minute(),now.second());
  intRTC.setTime(00,00,00);
  intRTC.setDate(1,1,1);

  //Set alarm time select previously
  intRTC.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds);
  //intRTC.setAlarmTime(now.hour()+alarmHours, now.minute()+alarmMinutes, now.second()+alarmSeconds);
  intRTC.enableAlarm(intRTC.MATCH_HHMMSS);
  
}

void alarmMatch(){ //REVISAR
  digitalWrite(LED, HIGH);
  alarmFlag = true; // Set flag
}

//Get current Time
DateTime current_Time(){
  return extRTC.now();
}

//Send bytes to EEPROM it's an adaptation of the function in github repository (it's thinking to have two different devices in the same design)
void M24M02DRCwriteBytes(uint8_t device_address, uint8_t page_address, uint8_t data_address, bool first, Data dest){
  
  int int_int =(int)(dest.int_temp*100);
  int int_skin=(int)(dest.pat_temp*100);
  //int int_int =(int)(dest.int_temp);
  //int int_skin=(int)(dest.pat_temp);
  
  if(first == true) {
    
    Wire.beginTransmission(device_address); // Initialize the Tx buffer
    Wire.write(page_address); // Put slave register address in Tx buffer
    Wire.write(data_address); // Put slave register address in Tx buffer
    Wire.write(dest.day);
    Wire.write(dest.month);
    Wire.write(dest.year >> 8);
    Wire.write(dest.year);
    Wire.write(dest.hour);
    Wire.write(dest.min);
    Wire.write(dest.sec);
    Wire.write(int_int >> 8);  //Sensor returns a float which we multiply*100 and we do a cast to int to be able to send via I2C
    Wire.write(int_int);
    Wire.write(int_skin >> 8);
    Wire.write(int_skin);
    Wire.write(dest.phy_act);        
    Wire.endTransmission(); // Send the Tx buffer
    
  } else{
    
    Wire.beginTransmission(device_address); // Initialize the Tx buffer
    Wire.write(page_address); // Put slave register address in Tx buffer
    Wire.write(data_address); // Put slave register address in Tx buffer
    Wire.write(dest.hour);
    Wire.write(dest.min);
    Wire.write(dest.sec);
    Wire.write(int_int >> 8);  
    Wire.write(int_int);
    Wire.write(int_skin >> 8);
    Wire.write(int_skin);
    Wire.write(dest.phy_act);     
    Wire.endTransmission(); // Send the Tx buffer
    //Serial.println(dest.hour);
  }
  
}

//Clear the EEPROM memory fully (own function, it's an adaptation of the writing function
void M24M02DRCclear(uint8_t device_address, uint8_t page_address, uint8_t data_address, uint8_t *dest){
    
    Wire.beginTransmission(device_address); // Initialize the Tx buffer
    Wire.write(page_address); // Put slave register address in Tx buffer
    Wire.write(data_address); // Put slave register address in Tx buffer
    
    for(uint16_t i=0; i < (L_MSmax-1) ; i++) {
      Wire.write(dest[i]); // Put data in Tx buffer
    }      
    Wire.endTransmission(); // Send the Tx buffer
}


//Read bytes from EEPROM it's and adaptation of the function in github repository (it's thinking to have two different devices in the same design)
void M24M02DRCreadBytes(uint8_t device_address, uint8_t page_address, uint8_t data_address){

     
  Wire.beginTransmission(device_address); // Initialize the Tx buffer
  Wire.write(page_address); // Put slave register address in Tx buffer
  Wire.write(data_address); // Put slave register address in Tx buffer
  Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(device_address,(L_MSmax-4)); // Read bytes from slave register address
  uint8_t i = 0;
  while(Wire.available()){
    receiveData[i] = Wire.read();
    //Serial.println(receiveData[i]);
    i++;
  }    
}

//Function to read all the Sensors
void readSensors(){
  
   //Read RTC
    DateTime fecha = current_Time();
    ObtData.day=fecha.day();
    ObtData.month=fecha.month();
    ObtData.year=fecha.year();
    ObtData.hour=fecha.hour();
    ObtData.min=fecha.minute();
    ObtData.sec=fecha.second();

    //Read Temperature
    ObtData.int_temp=INT_tmp.readTemperature();
    ObtData.pat_temp=SKIN_tmp.readTemperature();

    //Read Accelerometre
    //-------FORMA 1------
    if(accel.read(FF_MT_SRC_REG)>= 0x80){
      ObtData.phy_act=true;
      accel.read(FF_MT_SRC_REG);
    }else{
      ObtData.phy_act=false;
    } 
}

//Function to check the sensors
void testReadSensors(){
  
    //LEER TEMPERATURAS 
    Serial.print("Temperatura piel: ");
    Serial.print(ObtData.pat_temp);
    Serial.println("ºC");
    delay(100);
  
    Serial.print("Temperatura interna: ");
    Serial.print(ObtData.int_temp);
    Serial.println("ºC");
    delay(100);
    
    //LEER ACELEROMETRO
     //Read Accelerometre
    //-------FORMA 1------
    if(accel.read(FF_MT_SRC_REG)>= 0x80){
      ObtData.phy_act=true;
      Serial.println("EJERCICIO");      
    }else{
      ObtData.phy_act=false;
      Serial.println("ESPABILA");
    } 
    
    //LEER RTC EXTERNO
    //Leer fecha
    Serial.print("Fecha: ");
    Serial.print(ObtData.day,DEC);
    Serial.print("/");
    Serial.print(ObtData.month,DEC);
    Serial.print("/");
    Serial.print(ObtData.year,DEC);
    Serial.print(" at ");
    Serial.print(ObtData.hour,DEC);
    Serial.print(":");
    Serial.print(ObtData.min,DEC);
    Serial.print(":");
    Serial.println(ObtData.sec,DEC);
    delay(100);
}

//Fucntion to printData in EEPROM memory
void printData(){
  uint8_t c = 0;
  while(c<(L_MSmax-4)){
    Serial.print(receiveData[c],DEC);
    Serial.print(" ");
    c++;
  }
  Serial.println("");
}
    


void readMemory(){
  
   //Read RTC
    DateTime fecha = current_Time();
    ObtData.day=fecha.day();
    ObtData.month=fecha.month();
    ObtData.year=fecha.year();
    ObtData.hour=fecha.hour();
    ObtData.min=fecha.minute();
    ObtData.sec=fecha.second();

    //Read Temperature
    ObtData.int_temp=INT_tmp.readTemperature();
    ObtData.pat_temp=SKIN_tmp.readTemperature();

    //Read Accelerometre
    ObtData.phy_act=0;
    
    
}
void onWakeup() {
  
  digitalWrite(LED, HIGH);
}
