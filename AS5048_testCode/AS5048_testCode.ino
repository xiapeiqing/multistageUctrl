// The sensor should be connected to the hardware SPI pins (MISO, MOSI, SCK). The CS pin can be connected to any GPIO pin but should be passed to the constructor.
// 3-wire SPI (MOSI not connected) not supported! 
#include <AS5048A.h>


AS5048A angleSensor(53);

void setup()
{
	Serial.begin(115200);
	angleSensor.init();
}

void loop()
{
	delay(1000);

	word val = angleSensor.getRawRotation();
	Serial.print("Got rotation of: 0x");
	Serial.println(val, HEX);
	Serial.print("State: ");
	angleSensor.printState();
	Serial.print("Errors: ");
	Serial.println(angleSensor.getErrors());
}
