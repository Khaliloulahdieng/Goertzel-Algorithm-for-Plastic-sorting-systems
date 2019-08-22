
// CODE GENERAL POUR LE CONTROL DES LEDS, DU MOTEUR, DU DAC MCP4728 et DE L'ADC 3204


//CONFIGURATION I2C & SCAN 

#include <Wire.h>
#include <SPI.h>

//CALCUL DU TEMPS
/////////////////////////////////////////////////////////////////////
unsigned long t_0 = millis(); // gives time since startup in ms. Overflow after 50 days.
unsigned long loop_cnt = 0; //loop count
const int D = 100; //loop target duration in ms

//Wire and I²C functions
/////////////////////////////////////////////////////////////////////
byte data, data0, data1, data2; // used to buffer I²C data (in and/or out)
void set_I2C_register(byte ADDRESS, byte REGISTER, byte VALUE)
{
  Wire.beginTransmission(ADDRESS);
  delayMicroseconds(1);
  Wire.write(REGISTER);
  Wire.write(VALUE);
  Wire.endTransmission();
}

byte get_I2C_register(byte ADDRESS, byte REGISTER)
{
  Wire.beginTransmission(ADDRESS);
  delayMicroseconds(1);
  Wire.write(REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS, 1); //read 1 byte
  byte x = Wire.read();
  return x;
}

//****FIN CONFIGURATION I2C SCAN

// DEBUT CONFIG POUR LE DAC MCP4728

//## MCP4728 (Quad DAC 12bit - Vref=2.048V)
//Address
const byte MCP4728 = B1100000; //0x60

//mode
const byte MCP4728_MODE = B10000000;  // Vref interne - PD=PD0=G=0
const byte MCP4728_FAST = B00000000; //Write one DAC imputs registers from Channel A to Channel D, see fig 5-7 p38

//Commands (see from p31 of datasheet)
const byte MCP4728_RESET = 0x06;
const byte MCP4728_WAKEUP = 0x09;
const byte MCP4728_UPDATE = 0x08;
const byte MCP4728_ADDRESS = 0x0C; //can be changed in EEPROM!
const byte MCP4728_HARDW = B01010000; //Write all DAC imputs registers & EEPROM from Channel A to Channel D, takes ~25ms, see figure 5-9 p40

//DAC value entre 0 et 4095 (12bit) où 1LSB = 2.048/2**16=0.5mV       2048=1024mV    512= 256mV
//R sense = 22 Ohm, I_max=50mA => Ud_max=1000mV => DAC_max=2000
unsigned int DACS[] = {1000, 1000, 1000, 1000}; // {D1, D3, D3, D4} Udrive

// FIN CONFIG POUR LE DAC MCP4728


// SET CONSTANTS FOR THE ARDUINO ADC
const int adcChipSelectPin = SS;      // set pin 8 as the chip select for the ADC:

// ARDUINO MAPPING FOR THE LEDS

int D0= 13;
int D1= 12;
int D2= 11;
int D3= 10;
int D4= 7;

// ***END OF CONFIGURATION FOR THE LEDS

// START OF CONFIGURATION FOR THE MOTOR

//    Direct drive of stepper motor. Full step sequence. Both ways. fixe delay between substeps.

//Pin definition
int LED = 3;
int LED_minus = 2;
int A_p = A0;
int A_m = A1;
int B_p = A2;
int B_m = A3;

//timing and scan range
unsigned int Delai = 1000; //Delay between sub-steps in µs NB: 1 step = 4 sub-steps
                      //Start missing under steps at 800µs/sub-step
int long loop_count = 0;
int steps_per_cycle=200; //steps of a full forward/backward cycle. Motor swich direction at steps_per_cycle/2
                         //full mechanical range: 10mm ~150steps = 600 sub-steps  = 3s      @ 5000 µs/sub-step
                         //                                                       = 600ms   @ 1000 µs/sub-step

void step_none()
{
  digitalWrite(A_p, LOW);
  digitalWrite(A_m, LOW);
  digitalWrite(B_p, LOW);
  digitalWrite(B_m, LOW);
}

void wait(unsigned int d)
{
  delayMicroseconds(d);
//  delayMicroseconds(d/2);
//  step_none();
//  delayMicroseconds(d/2);
}
  
void step_forward(unsigned int d)
{
  //step 11 cf http://www.edaboard.com/thread217270.html
  digitalWrite(A_p, HIGH);
  digitalWrite(A_m, LOW);
  digitalWrite(B_p, HIGH);
  digitalWrite(B_m, LOW);
  wait(d);
  //step 10
  digitalWrite(A_p, HIGH);
  digitalWrite(A_m, LOW);
  digitalWrite(B_p, LOW);
  digitalWrite(B_m, HIGH);
  wait(d);
  //step 00
  digitalWrite(A_p, LOW);
  digitalWrite(A_m, HIGH);
  digitalWrite(B_p, LOW);
  digitalWrite(B_m, HIGH);
  wait(d);
  //step 01
  digitalWrite(A_p, LOW);
  digitalWrite(A_m, HIGH);
  digitalWrite(B_p, HIGH);
  digitalWrite(B_m, LOW);
  wait(d);
}

void step_backward(unsigned int d)
{
  //step 01
  digitalWrite(A_p, LOW);
  digitalWrite(A_m, HIGH);
  digitalWrite(B_p, HIGH);
  digitalWrite(B_m, LOW);
  wait(d);
  //step 00
  digitalWrite(A_p, LOW);
  digitalWrite(A_m, HIGH);
  digitalWrite(B_p, LOW);
  digitalWrite(B_m, HIGH);
  wait(d);
  //step 10
  digitalWrite(A_p, HIGH);
  digitalWrite(A_m, LOW);
  digitalWrite(B_p, LOW);
  digitalWrite(B_m, HIGH);
  wait(d);
  //step 11
  digitalWrite(A_p, HIGH);
  digitalWrite(A_m, LOW);
  digitalWrite(B_p, HIGH);
  digitalWrite(B_m, LOW);
  wait(d);
}

//******END OF CONFIGURATION FOR THE MOTOR

// LA FONCTION READ_ADC POUR LE MCP3204

//Function to read the ADC, accepts the channel to be read.
float readAdc(int channel)
{
  byte adcPrimaryRegister = 0b00000110;      // Sets default Primary ADC Address register B00000110, This is a default address setting, the third LSB is set to one to start the ADC, the second LSB is to set the ADC to single ended mode, the LSB is for D2 address bit, for this ADC its a "Don't Care" bit.
  byte adcPrimaryRegisterMask = 0b00000111;  // b00000111 Isolates the three LSB.  
  byte adcPrimaryByteMask = 0b00001111;      // b00001111 isolates the 4 LSB for the value returned. 
  byte adcPrimaryConfig = adcPrimaryRegister & adcPrimaryRegisterMask; // ensures the adc register is limited to the mask and assembles the configuration byte to send to ADC.
  byte adcSecondaryConfig = channel << 6;
  
  noInterrupts(); // disable interupts to prepare to send address data to the ADC.  
  digitalWrite(adcChipSelectPin,LOW); // take the Chip Select pin low to select the ADC.
  SPI.transfer(adcPrimaryConfig); //  send in the primary configuration address byte to the ADC.  
  byte adcPrimaryByte = SPI.transfer(adcSecondaryConfig); // read the primary byte, also sending in the secondary address byte.  
  byte adcSecondaryByte = SPI.transfer(0x00); // read the secondary byte, also sending 0 as this doesn't matter. 
  digitalWrite(adcChipSelectPin,HIGH); // take the Chip Select pin high to de-select the ADC.
  
  interrupts(); // Enable interupts.
  adcPrimaryByte &= adcPrimaryByteMask; // Limits the value of the primary byte to the 4 LSB:
  int digitalValue = (adcPrimaryByte << 8) | adcSecondaryByte; // Shifts the 4 LSB of the primary byte to become the 4 MSB of the 12 bit digital value, this is then ORed to the secondary byte value that holds the 8 LSB of the digital value.
  float value = (float(digitalValue) * 5.000) / 4096.000; // The digital value is converted to an analogue voltage using a VREF of 2.048V.
  return digitalValue; // Returns the value from the function
}

// ****END OF THE FUNCTION


void setup() 
{
   // SET UP & SCAN I2C BUS START  // Copy/Pasted/Tweeked from https://playground.arduino.cc/Main/I2cScanner
   
  Wire.begin();
  Wire.setClock(400000); //400kHz I²C 
  Serial.println("\nI2C Scanning...");
  byte error, address;
  int nDevices =0;
  
  for(address = 1; address < 128; address++ )
  {
    delay(10);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address<16){Serial.print("0");}
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4){
      Serial.print("Unknown error at address 0x");
      if (address<16){Serial.print("0");}
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.print("done\n");
  delay(100);
  //**** END OF I2C & SCAN SET UP


  //SET UP OF MCP4728 START
  
  Serial.println(" ");
  Serial.print("MCP4728 is RESET, ");
  Wire.beginTransmission(MCP4728);
  Wire.write(MCP4728_RESET);
  Wire.endTransmission();
  delay(100);
  Serial.print("loaded with [");
  Wire.beginTransmission(MCP4728);
  Wire.write(MCP4728_HARDW);
  for(int i=0; i<4; i++)
  {
    Serial.print(DACS[i]/2/22); //0.5mV per value, 20 Ohm sensing
    Serial.print(" ");
    byte Byte1 = (DACS[i]>>8); //first bits (12-8=4bits)
    Byte1 = MCP4728_MODE|Byte1;
    byte Byte2 = DACS[i]&0xff; //last byte
    Wire.write(Byte1);
    Wire.write(Byte2);
    }
  Wire.endTransmission();
  delay(100);
  
  Serial.println("] mA and is ready to go");

  //******* END OF SET UP OF MCP4728
  
  
  // DEBUT SET UP POUR L'ADC MCP3204

  pinMode (adcChipSelectPin, OUTPUT); 
  // set the ChipSelectPins high initially: 
  digitalWrite(adcChipSelectPin, HIGH);  
  // initialise SPI:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);         // Not strictly needed but just to be sure.
  SPI.setDataMode(SPI_MODE0);        // Not strictly needed but just to be sure.
  // Set SPI clock divider to 16, therfore a 1 MhZ signal due to the maximum frequency of the ADC.
  SPI.setClockDivider(SPI_CLOCK_DIV16);

// ***FIN SET UP POUR L'ADC

// DEBUT SET UP POUR LE SPI
  Serial.begin(9600);
  // ****FIN SET UP SPI

  // DEBUT SET UP POUR LE MOTOR

 // GPIO set up

 pinMode(LED, OUTPUT);
 pinMode(LED_minus, OUTPUT);
 pinMode(A_p, OUTPUT);
 pinMode(A_m, OUTPUT);
 pinMode(B_p, OUTPUT);
 pinMode(B_m, OUTPUT);
 
 digitalWrite(LED, LOW);
 digitalWrite(LED_minus, LOW);
 digitalWrite(A_p, LOW);
 digitalWrite(A_m, LOW);
 digitalWrite(B_p, LOW);
 digitalWrite(B_m, LOW);

// Mechanical set up

 for(int i=0; i<150; i++){
  step_forward(5000);}
step_none();
delay(200);
 for(int i=0; i<75; i++){
  step_backward(5000);}
step_none();
delay(200);
 for(int i=0; i<steps_per_cycle/4; i++){
  step_backward(5000);}
step_none();
delay(200);

// ****FIN SET UP POUR LE MOTOR
  
}

void loop() 

{
  // CONTROL OF THE LED LOOP
   analogWrite(D0, 0);
  analogWrite(D1, 128);
  analogWrite(D2,0);
  analogWrite(D3, 0);
  analogWrite(D4, 0);

  // ***end of loop control

  // LOOP FOR THE MCP3204
   float voltage0 = readAdc(0);
  //float voltage1 = readAdc(1);
  //float voltage2 = readAdc(2);
  Serial.println("Resultat pour la conversion A/N");
  Serial.println(readAdc(0));
  delay(1);

  // ***END OF MCP3204 LOOP

  // LOOP FOR THE MOTOR

 loop_count = loop_count + 1;

  //forward
  digitalWrite(LED, HIGH);
  digitalWrite(LED_minus, LOW);
  for(int i=0;i<steps_per_cycle/2;i++){
    step_forward(D);}

  step_none();
  delay(200);
  
  //reverse
  digitalWrite(LED, LOW);
  digitalWrite(LED_minus, HIGH);
  for(int i=0;i<steps_per_cycle/2;i++){
    step_backward(D);}

  step_none();
  delay(200);
  
// ***END OF THE MOTOR LOOP
}
