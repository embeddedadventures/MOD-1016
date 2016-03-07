
/*
Copyright (c) 2016, Embedded Adventures
All rights reserved.
Contact us at source [at] embeddedadventures.com
www.embeddedadventures.com
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
- Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
*/

//	AS3935 MOD-1016 Lightning Sensor Arduino library
//	Written originally by Embedded Adventures


#ifndef __AS3935_h
#define __AS3935_h

#include <Arduino.h>
	
#define uns8 unsigned char
#define uns16 unsigned int
#define sgn32 long int
#define sgn64 long long int

#define AS3935_ADDR 	0x03
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

class AS3935Class
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
	void enableDisturbers();
	void disableDisturbers();
	uns8 readRegister(uns8 reg, uns8 mask);
	uns8 getNoiseFloor();
	uns8 getAFE();
	uns8 getTuneCaps();
	uns8 getIRQ();
	uns8 getLightDistance();
	//Values for KM -> -1 = out of range, 0 = overhead, 1 = not in table
	int  calculateDistance();
	int  getDivisionRatio();
	unsigned int  getIntensity();
};

extern AS3935Class mod1016;

#endif
