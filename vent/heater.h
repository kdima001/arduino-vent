#ifndef HEATER_H
#define HEATER_H
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//Нагреватель
class Heater {
	private:
		uint8_t pin;
		uint8_t lock;
		char name[10];
	public:
		Heater(uint8_t, char*);
		void Off(void);
		uint8_t On(uint8_t);
		uint8_t GetState(void) {return digitalRead(pin);};
		//void PrintToSerial(void);
		void PrintToSerialWEB(void);
};

#endif HEATER_H