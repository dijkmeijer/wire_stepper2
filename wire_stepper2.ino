#include <EEPROM.h>
#include <Wire.h>
#include <AccelStepper.h>

// rotowand



// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER,11, 10); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

#define WireAdress 2
#define MaxSpeedAdress 0
#define MaxAccelerationAdress 4
#define Positie_1_Adress 8
#define Positie_2_Adress 12
 
#define SensorPin1 8
#define SensorPin2 9

long MaxSpeed = 5000;
long  MaxAcceleration = 8000;
long Position_1;
long Position_2;
byte Position;

char buffer[10];
String value;


void setup()
{  

  // Change these to suit your stepper if you want
  Serial.begin(9600); 
  MaxSpeed = read_long_Eeprom(MaxSpeedAdress);
  if(MaxSpeed < 10) MaxSpeed = 4000;
  MaxAcceleration = read_long_Eeprom(MaxAccelerationAdress);
  if(MaxAcceleration < 10) MaxAcceleration = 8000;
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(MaxAcceleration);
  pinMode(SensorPin1, INPUT_PULLUP);
  pinMode(SensorPin2, INPUT_PULLUP);
  Wire.begin(WireAdress);               
  Wire.onReceive(receiveEvent); // register event
  stepper.setPinsInverted(true, false, false);
  Serial.print("start");

}

void loop()
{
  // If at the end of travel go to the other end
  if((PINB & 0x03) != 0x03) 
 // Serial.println(PINB & 0x03);
 stepper.run();
}



void receiveEvent(int howMany)
{
  int i=0;

  while(1 < Wire.available()) // loop through all but the last
  {
    buffer[i++] = Wire.read(); // receive byte as a character
    //    Serial.print(c);         // print the character
  }
  buffer[i]=0;
  Position = Wire.read();
  delay(300);
  Serial.write(buffer);
  Serial.print("  ");
  Serial.write(Position);
  delay(300);
  if(Position != 0)
    parseCommand();

}


void write_long_Eeprom(int addr, long val){
  EEPROM.write(addr, (byte )((val >> 24) & 0xff));
  EEPROM.write(addr+1, (byte )((val >> 16) & 0xff));
  EEPROM.write(addr+2, (byte )((val >> 8) & 0xff));
  EEPROM.write(addr+3, (byte )((val) & 0xff));
}


long read_long_Eeprom(int addr){
  long val;
  val = EEPROM.read(addr);
  val = (val << 8) | EEPROM.read(addr+1);
  val = (val << 8) | EEPROM.read(addr+2);
  val = (val << 8) | EEPROM.read(addr+3);
  return val;
}

void parseCommand(){ 


  switch(Position) {
  case 0: 
    break;
  case 1: 
    stepper.moveTo(Position_1);
    break;
  case 2: 
    stepper.moveTo(Position_2);
    break;
  case 3: 
    break;
  case 4:
    value=String(buffer);
    MaxSpeed=value.toInt();
    Serial.print("Maxspeed = ");
    Serial.println(MaxSpeed);
    stepper.setMaxSpeed(MaxSpeed);
    write_long_Eeprom(MaxSpeedAdress, MaxSpeed);
    break;
  case 5:
    value=String(buffer);
    MaxAcceleration=value.toInt();
    Serial.print("MaxAcceleration = ");
    Serial.println(MaxAcceleration);
    stepper.setAcceleration(MaxAcceleration);
    write_long_Eeprom(MaxAccelerationAdress, MaxAcceleration);
    break;
  case 6:
    value=String(buffer);
    Serial.print("Position_1 = ");
    Serial.println(Position_1);
    Position_1=value.toInt();
    write_long_Eeprom(Positie_1_Adress, MaxSpeed);
    break;
  case 7:
    value=String(buffer);
    Position_2=value.toInt();
    Serial.print("Position_2 = ");
    Serial.println(Position_2);
    write_long_Eeprom(Positie_2_Adress, MaxSpeed);
    break;
  }
}






