#include <Wire.h>
#include <MOD1016.h>

#define IRQ_pin 2

volatile bool detected = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mod1016.init(IRQ_pin);
 
  //Tune Caps, Set AFE, Set Noise Floor
  autoTuneCaps(IRQ_pin);
  mod1016.setOutdoors();
  mod1016.setNoiseFloor(2);
  
  Serial.println("Welcome to the MOD-1016 (AS3935) Lightning Sensor test sketch!");
  Serial.println("Embedded Adventures (www.embeddedadventures.com)\n");

  Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");

  attachInterrupt(digitalPinToInterrupt(IRQ_pin), alert, RISING);
}

void loop() {
  if (detected) {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }
}

void alert() {
  detected = true;
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE DETECTED");
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        break;
      case 8: 
        Serial.println("LIGHTNING DETECTED");
        printDistance();
        break;
      default:
        Serial.println("IDK");
        break;
    }
}

void printDistance() {
  int distance = mod1016.calculateDistance();
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");  
  }
}

