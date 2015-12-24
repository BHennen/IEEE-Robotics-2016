#include "MotorDriver.h"

MotorDriver* md;
byte left_motor_pin_fwd = 5;
byte left_motor_pin_bwd = 6;
byte left_motor_current_pin = A0;
byte right_motor_pin_fwd = 9;
byte right_motor_pin_bwd = 10;
byte right_motor_current_pin = A1;
byte enable_pin = 8;
byte fault_pin = 11;
void stopIfFault()
{
	if(md->isFault())
	{
		Serial.println("fault");
		while(1);
	}
}

void setup()
{
	md = new MotorDriver(left_motor_pin_fwd,
						 left_motor_pin_bwd,
						 left_motor_current_pin,
						 right_motor_pin_fwd,
						 right_motor_pin_bwd,
						 right_motor_current_pin,
						 enable_pin,
						 fault_pin);
	Serial.begin(9600);
	Serial.println("Dual MC33926 Motor Driver");
}

void loop()
{
	for(int i = 0; i <= 255; i++)
	{
		md->SetLeftSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Left Motor current: ");
			Serial.println(md->GetLeftCurrentMilliamps());
		}
		delay(15);
	}

	for(int i = 255; i >= -255; i--)
	{
		md->SetLeftSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Left Motor current: ");
			Serial.println(md->GetLeftCurrentMilliamps());
		}
		delay(15);
	}

	for(int i = -255; i <= 0; i++)
	{
		md->SetLeftSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Left Motor current: ");
			Serial.println(md->GetLeftCurrentMilliamps());
		}
		delay(15);
	}

	for(int i = 0; i <= 255; i++)
	{
		md->SetRightSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Right Motor current: ");
			Serial.println(md->GetRightCurrentMilliamps());
		}
		delay(15);
	}

	for(int i = 255; i >= -255; i--)
	{
		md->SetRightSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Right Motor current: ");
			Serial.println(md->GetRightCurrentMilliamps());
		}
		delay(15);
	}

	for(int i = -255; i <= 0; i++)
	{
		md->SetRightSpeed(i);
		stopIfFault();
		if(abs(i) % 50 == 0)
		{
			Serial.print("Right Motor current: ");
			Serial.println(md->GetRightCurrentMilliamps());
		}
		delay(15);
	}
}
