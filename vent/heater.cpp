#include "heater.h"

Heater::Heater(uint8_t p_pin, char* p_name){
	memcpy(name, p_name, 10);
	pin = p_pin;
	lock = 0;
	pinMode(pin, OUTPUT);
	Off();
}

void Heater::Off(){
	//(Если можно трогать или всегда выключено) и включено - гасим
	if ((lock==0 || lock ==1) && digitalRead(pin)) digitalWrite(pin, LOW);
}

uint8_t Heater::On(uint8_t p_temp){
	//Свободный режим. Если выключено и есть тепло - включаем
	if (lock ==0 && p_temp>40){
		if (!digitalRead(pin)) 
			digitalWrite(pin, HIGH);
		return 0;
	}

	//если нет тепла возвращаем ошибку
	else 
		return 1;
		
	//Если заблокировано в "выключено" - ничего не трогаем
	if (lock==1) return 0;
	
	//Если режим "включено всегда"
	if (lock==2 && !digitalRead(pin)){
		digitalWrite(pin, HIGH);
		return 0;
	}
}

void Heater::PrintToSerialWEB(void) {
	Serial.print(F("GET /objects/?object="));	Serial.print(name); Serial.print(F("_TSS&op=m&m=tempChanged&t=")); Serial.println(GetState(), DEC);
}


