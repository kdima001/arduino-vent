#include "vent.h"

#include <Servo.h>
#include <I2C_eeprom.h>
#include <stdio.h>

Vent::Vent(uint8_t p_s1_pin, uint8_t p_s2_pin, uint8_t p_s3_pin, uint8_t p_s4_pin, uint8_t p_s5_pin, uint8_t p_valve_pin, uint8_t p_en_pin,
							uint16_t p_adr, I2C_eeprom *p_eeprom)	{
	en_pin = p_en_pin;
	s1_pin = p_s1_pin;
	s2_pin = p_s2_pin;
	s3_pin = p_s3_pin;
	s4_pin = p_s4_pin;
	s5_pin = p_s5_pin;
	
	pinMode(s5_pin, OUTPUT);
	pinMode(s4_pin, OUTPUT);
	pinMode(s3_pin, OUTPUT);
	pinMode(s2_pin, OUTPUT);
	pinMode(s1_pin, OUTPUT);
	pinMode(en_pin, OUTPUT);
	
	digitalWrite(s5_pin, LOW);
	digitalWrite(s4_pin, LOW);
	digitalWrite(s3_pin, LOW);
	digitalWrite(s2_pin, LOW);
	digitalWrite(s1_pin, LOW);
	digitalWrite(en_pin, LOW);
	
	valve.attach(p_valve_pin);
		
	eeprom = p_eeprom;
	adr = p_adr;
	
	MaxSpeed = 0;
	Speed = 0;
}

bool Vent::ReadEepromParams(void) { 
	eeprom->readBlock(adr, MaxSpeedByHour, 24); 
	for (int i = 0; i<24; i++)
		if (MaxSpeedByHour[i] > VENT_MAX_SPEED) return 0;
		else;
	return 1;
};

void Vent::SetMaxSpeed(String *p_str){
	for (int i = 0; i < 24; i++){ 
		MaxSpeedByHour[i] = (uint8_t)(p_str->substring(i, i+1).toInt()); 
		eeprom->writeByte(adr+i, MaxSpeedByHour[i]);
	}
}		

void Vent::UpdateMaxSpeed(uint8_t hour){	
		MaxSpeed = MaxSpeedByHour[hour];
}

void Vent::SetSpeed(int p_sp){	
		if( p_sp != Speed){
			Speed = p_sp;
			if ( Speed <= 0) {
				//стоп машина
				digitalWrite(s5_pin, LOW);
				digitalWrite(s4_pin, LOW);
				digitalWrite(s3_pin, LOW);
				digitalWrite(s2_pin, LOW);
				digitalWrite(s1_pin, LOW);
			}
			else {
			 ServoSetPos(140); //открыть на всю
			 delay(1000);
			}
			
			switch (Speed){
				case -3:
					ServoSetPos(38);
					delay(50);
				break;
				case -2:
					ServoSetPos(76);
					delay(50);
				break;
				case -1:
					ServoSetPos(106);
					delay(50);
				break;
				case 0:
					digitalWrite(s5_pin, LOW);
					digitalWrite(s4_pin, LOW);
					digitalWrite(s3_pin, LOW);
					digitalWrite(s2_pin, LOW);
					digitalWrite(s1_pin, LOW);
				break;
				case 1:
					digitalWrite(s5_pin, LOW);
					digitalWrite(s4_pin, LOW);
					digitalWrite(s3_pin, LOW);
					digitalWrite(s2_pin, LOW);
					digitalWrite(s1_pin, HIGH);
				break;
				case 2:
					digitalWrite(s5_pin, LOW);
					digitalWrite(s4_pin, LOW);
					digitalWrite(s3_pin, LOW);
					digitalWrite(s2_pin, HIGH);
					digitalWrite(s1_pin, LOW);
				break;
				case 3:
					digitalWrite(s5_pin, LOW);
					digitalWrite(s4_pin, LOW);
					digitalWrite(s3_pin, HIGH);
					digitalWrite(s2_pin, LOW);
					digitalWrite(s1_pin, LOW);
				break;
				case 4:
					digitalWrite(s5_pin, LOW);
					digitalWrite(s4_pin, HIGH);
					digitalWrite(s3_pin, LOW);
					digitalWrite(s2_pin, LOW);
					digitalWrite(s1_pin, LOW);
				break;
				case 5:
					digitalWrite(s5_pin, HIGH);
					digitalWrite(s4_pin, LOW);
					digitalWrite(s3_pin, LOW);
					digitalWrite(s2_pin, LOW);
					digitalWrite(s1_pin, LOW);
				break;
			}
		}
}

void Vent::SetSpeedSafe(int p_sp){
	SetSpeed(constrain(p_sp, VENT_MIN_SPEED, MaxSpeed));		
}

void Vent::ServoSetPos(int p_pos)
{ digitalWrite(en_pin, HIGH);
	delay(50);	
	int pos = valve.read();
  if(pos > p_pos) //decrease
    for(; p_pos < pos; pos --)                                          
      { valve.write(pos); SoftwareServo::refresh(); delay(50); }
  else //increase
    for(; p_pos > pos; pos ++)                                    
      { valve.write(pos); SoftwareServo::refresh(); delay(50); }
	delay(50);		
	digitalWrite(en_pin, LOW);
}

void Vent::TestValve(void)
{ int pos;
	/*for(pos = 38; pos < 140; pos++)  
		ServoSetPos(pos);              
	delay(3000);                       	
  for(pos = 140; pos>=38; pos--)     
		ServoSetPos(pos);            
  delay(3000);*/
	
	ServoSetPos(140);
	delay(1000);
	ServoSetPos(106);
	delay(1000);
	ServoSetPos(76);
	delay(1000);
	ServoSetPos(38);
	delay(1000);
	ServoSetPos(76);
	delay(1000);
	ServoSetPos(106);
	delay(1000);
	ServoSetPos(140);
	delay(1000);
	
	ServoSetPos(38);
	delay(3000);
	ServoSetPos(140);
	delay(3000);
	
	ServoSetPos(38);
	delay(3000);
	ServoSetPos(140);
	delay(3000);
	
	ServoSetPos(38);
	delay(3000);
	ServoSetPos(140);
	delay(3000);
	
	ServoSetPos(38);
	delay(3000);
	ServoSetPos(140);
	delay(3000);
}

void Vent::Test(void)
{	SetSpeed(0); delay(500);
	SetSpeed(1); delay(500);
	SetSpeed(2); delay(500);
	SetSpeed(3); delay(500);
	SetSpeed(4); delay(500);
	SetSpeed(5); delay(500);
	SetSpeed(0); delay(500);
	delay(5000);
	TestValve();
	SetSpeed(-1); delay(500);
	SetSpeed(-2); delay(500);
	SetSpeed(-3); delay(500);
	SetSpeed(-2); delay(500);
	SetSpeed(-1); delay(500);
	SetSpeed(0); 
}

