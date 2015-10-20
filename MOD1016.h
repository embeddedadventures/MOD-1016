#ifndef __MOD1016_h
#define __MOD1016_h

#include <Arduino.h>
	
#define uns8 unsigned char
#define uns16 unsigned int
#define sgn32 long int
#define sgn64 long long int

#define MOD1016_ADDR 	0x03
#define INDOORS 		0x24
#define OUTDOORS 		0x1C

//			 Reg address	bitmask
#define DISP_LCO	0x08,	0x80
#define TUNE_CAPS	0x08,	0x0F
#define AFE_GB		0x00,	0x3E
#define NOISE_FLOOR	0x01,	0x70
#define MASK_DST	0x03,	0x20
#define IRQ_TBL		0x03,	0x0F
#define LCO_FDIV	0x03,	0xC0
#define LGHT_DIST	0x07,	0x3F
#define DISP_TRCO	0x08,	0x20
#define PWD			0x00,	0x01

//Set of functions for auto calibration of TUNE_CAPS

/*Automatically sets TUNE_CAPS to the best possible setting, ie the one that producs a frequency closest to 500kHz*/
void autoTuneCaps(int irq);
/*ISR for measuring the frequency output on the IRQ pin*/
void pulseDetected();
/*Generate the frequency output on IRQ pin and measure number of pulses in 200ms. Returns number of pulses * 5*/
sgn32 getFrequency(int irq);
/*Measure frequency output under setting 0-15 on TUNE_CAPS*/
void freqPerTuneCaps(int fdiv, int irq);
/*Search for the tuning whose frequency is closest to 500kHz, and sets it to that*/
void recommendTuning();

void auto_calibrate(int irq);

//End of auto-calibration functions

class MOD1016Class
{
private:
	uns8 readRegisterRaw(uns8 reg);
	
public:
	void init(int IRQ_pin);
	void calibrateRCO();
	void writeRegister(uns8 reg, uns8 mask, uns8 data);
	void setIndoors();
	void setOutdoors();
	void setNoiseFloor(uns8 noise);
	void setTuneCaps(uns8 tune);
	uns8 readRegister(uns8 reg, uns8 mask);
	uns8 getNoiseFloor();
	uns8 getAFE();
	uns8 getTuneCaps();
	uns8 getIRQ();
	uns8 getLightDistance();
	//Values for KM -> -1 = out of range, 0 = overhead, 1 = not in table
	int  calculateDistance();
	int  getDivisionRatio();
};

extern MOD1016Class mod1016;

#endif