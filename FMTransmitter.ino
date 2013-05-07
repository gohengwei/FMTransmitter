#include <Wire.h>
long frequency = 102000000;          // the default initial frequency in Hz
long newFrequency = 0;
bool mute = true;
const uint8_t addr= 0x66;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

void setup() {   
	Serial.begin(9600);
	Serial.println("Initializing radio...");
	Wire.begin();
	cbi(PORTC, 4);
    cbi(PORTC, 5);
	 //digitalWrite(SDA, 0);
	//digitalWrite(SCL, 0);
	initFM();
}

// the loop routine runs over and over again forever:
void loop() {
}

void initFM()
{
	writeToReg(addr, 0x0E, 0x05); // Software reset
    
    writeToReg(addr, 0x01, 0xB4); // Reg 1: forced subccarrier, pilot tone on
    
    writeToReg(addr, 0x02, 0x03); // Reg 2: Unlock detect off, 2mW Tx Power

	setFrequency(frequency);
	//writeToReg(addr, 0x08, 0x1A); //Register 8: set Osc on band 2
    
    writeToReg(addr, 0x00, 0xA1); //Register 0: 200mV audio input,   75us pre-emphasis on, crystal off, power on
 
    writeToReg(addr, 0x0E, 0x05); // Software reset
    
    writeToReg(addr, 0x06, 0x1E); // Reg 6: chare pumps at 320uA and 80 uA
 
}

void writeToReg(uint8_t addr,uint8_t reg, uint8_t data)
{
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}

void setFrequency(long freq)
{
	int new_frequency;
 
      // New Range Checking... Implement the (experimentally determined) VFO bands:
      if (freq < 88500000) {        // Band 3
        writeToReg(addr, 0x08, 0x1B);
        Serial.println("Band 3");
      }  
      else if (freq < 97900000) {   // Band 2
        writeToReg(addr, 0x08, 0x1A);
        Serial.println("Band 2");
      }
      else if (freq < 103000000) {  // Band 1 
        writeToReg(addr, 0x08, 0x19);
        Serial.println("Band 1");
      }
      else {                        // Band 0
        // Must be OVER 103.000.000,
        writeToReg(addr, 0x08, 0x18);
        Serial.println("Band 0");
      }
      
      new_frequency = (freq + 304000) / 8192;
      
      uint8_t reg3 = new_frequency & 255;                  //extract low byte of frequency register
      uint8_t reg4 = new_frequency >> 8;                   //extract high byte of frequency register
      writeToReg(addr, 0x03, reg3);                             //send low byte
      writeToReg(addr, 0x04, reg4);                             //send high byte
      writeToReg(addr, 0x0E, 0x05);         // Software reset
}
