#ifndef VENT_H
#define VENT_H
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <I2C_eeprom.h>
#include <SoftwareServo.h>

#define VENT_MIN_SPEED -3
#define VENT_MAX_SPEED 5

class Vent {
	private:
		uint8_t en_pin, s1_pin, s2_pin, s3_pin, s4_pin, s5_pin;
		I2C_eeprom *eeprom;
		uint16_t adr;
		SoftwareServo valve;
		//uint8_t offAngle, onAngle
		void ServoSetPos(int);
	public:
		int Speed;            // Текущая скорость
		uint8_t MaxSpeed;         // Максимальная скорость - обновляется из нижнего массива по часам
		uint8_t MaxSpeedByHour[24];  // Максимальная скорость по часам
		
		Vent(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t, I2C_eeprom*);
		bool ReadEepromParams(void);
		void SetMaxSpeed(String*);
		void UpdateMaxSpeed(uint8_t);
		void SetSpeed(int);
		void SetSpeedSafe(int);
		void TestValve(void);
		void Test(void);
};

#endif