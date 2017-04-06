# MOD-1016 #

Arduino library and sample sketch for the <a href="http://www.embeddedadventures.com/as3935_lightning_sensor_module_mod-1016.html">AS3935 MOD-1016 Lightning Sensor</a>.

## Using the library ##
The MOD-1016 lightning sensor can be interfaced with using I2C or SPI, using Arduino's Wire or SPI library.

	init(uns8 irqPin)

Use this function to communicate with the sensor using I2C protocol. 

	init(uns8 irqPin, uns8 csPin)
Use this function to use Arduino's SPI library for communicating with the sensor. The CS pin is user-defined.

	autoTuneCaps(uns8 irqPin)
When you purchase the sensor, it comes pre-set to the best value and includes that value on the label. Should you misplace the label or forget the right tuning value, use this function to automatically set it for you.

	getNoiseFloor()	
Use this function to get the current noise floor setting (0-7).

	getAFE()
Use this function to determine if the sensor is set for indoors (18) or outdoors (14).

	getTuneCaps()
Use this function to get the current antenna tuning (0-15) 

	getIRQ()
Use this function to get the event type that asserted the interrupt pin.

	calculateDistance()
Get the estimated distance of the lightning source. Out of range = -1, 0 = source is overhead, and 1 = not in the distance estimation table (Table 17 in the <a href="https://www.embeddedadventures.com/datasheets/AS3935_Datasheet_EN_v2.pdf">AS3935 datasheet</a>). 

	getIntensity()
Get the calculated energy intensity measured by the sensor. It has no physical meaning at all but it's nice to look at :)

## Tested with the following boards ##
- Arduino Uno/compatible (using SPI and I2C)
- Arduino Nano (using SPI and I2C)
- <a href="https://www.embeddedadventures.com/esp8266_wifi_module_wrl-esp7.html">ESP7 breakout</a> and <a href="https://www.embeddedadventures.com/esp8266_wifi_module_wrl-esp12e.html">ESP12E breakout</a> (using I2C -> SDA/SCL = IO5/IO4)

