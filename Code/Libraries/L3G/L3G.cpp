#include <L3G.h>
#include <Wire.h>
#include <math.h>

// Constructors ////////////////////////////////////////////////////////////////

L3G::L3G()
{
}

// Public Methods //////////////////////////////////////////////////////////////

bool L3G::init(byte threshold_size, byte cs, byte sdo, byte sda, byte scl)
{
	_cs = cs;
	_miso = sdo;
	_mosi = sda;
	_clk = scl;

	pinMode(_cs, OUTPUT);
	pinMode(_clk, OUTPUT);
	pinMode(_mosi, OUTPUT);
	pinMode(_miso, INPUT);

	digitalWrite(_cs, HIGH);

	address = L3GD20_ADDRESS;

	/* Make sure we have the correct chip ID since this checks
	for correct address and that the IC is properly connected */
	uint8_t id = readReg(WHO_AM_I);
	if(id != L3GD20H_ID)
	{
		return false;
	}

	/*
	* Configure the registers so we use a dynamic fifo stream with desired threshold size
	* 250 DPS scale, 200Hz ODR, 50Hz bandwidth, only use Z axis
	*/
		
	// Low_ODR = 0 (low speed ODR disabled)
	writeReg(LOW_ODR, 0x00);

	//config fifo to dynamic stream and desired threshold size
	writeReg(FIFO_CTRL, (B1100 << 4) | threshold_size);

	//Enable gyro to send an interrupt when the number of data in FIFO > threshold
	writeReg(CTRL_REG3, B00000100);

	//enable fifo
	writeReg(CTRL5, B01000000);

	// FS = 00 (+/- 250 dps full scale)
	writeReg(CTRL_REG4, 0x00);

	// 0x6F = 0b01101100
	// DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = 1; Yen = Xen = 0 (X axis enabled)
	writeReg(CTRL_REG1, 0x6C);

	return true;
}

// Writes a gyro register
void L3G::writeReg(byte reg, byte value)
{
	digitalWrite(_clk, HIGH);
	digitalWrite(_cs, LOW);

	SPIxfer(reg);
	SPIxfer(value);

	digitalWrite(_cs, HIGH);
}

// Reads a gyro register
byte L3G::readReg(byte reg)
{
	byte value;

	digitalWrite(_clk, HIGH);
	digitalWrite(_cs, LOW);

	SPIxfer((uint8_t)reg | 0x80); // set READ bit
	value = SPIxfer(0xFF);

	digitalWrite(_cs, HIGH);

	return value;
}

// Reads the z gyro channels and stores it in z. Return true when new data.
bool L3G::read()
{
	byte num_data = readReg(FIFO_SRC) & B00011111;
	if(num_data > 0) //read only if we have data
	{
		long raw_z = 0;
		for(byte datum = 0; datum < num_data; ++datum)
		{
			digitalWrite(_clk, HIGH);
			digitalWrite(_cs, LOW);

			SPIxfer(OUT_Z_L | 0x80 | 0x40); // SPI read, autoincrement

			uint8_t zlg = SPIxfer(0xFF);
			uint8_t zhg = SPIxfer(0xFF);

			digitalWrite(_cs, HIGH);

			raw_z += ((int16_t)(zhg << 8 | zlg));
		}
		//Convert to DPS and average over number of data points we read. Negative since Z axis upside down.
		z = -(raw_z * 0.00875f / num_data);
		fresh_data = false;
		return true;
	}
	return false;
}

// Private Methods //////////////////////////////////////////////////////////////

uint8_t L3G::SPIxfer(uint8_t x)
{
	uint8_t value = 0;

	for(int i = 7; i >= 0; i--)
	{
		digitalWrite(_clk, LOW);
		if(x & (1 << i))
		{
			digitalWrite(_mosi, HIGH);
		}
		else
		{
			digitalWrite(_mosi, LOW);
		}
		digitalWrite(_clk, HIGH);
		if(digitalRead(_miso))
			value |= (1 << i);
	}

	return value;
}
