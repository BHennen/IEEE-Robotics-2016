#ifndef Bitset_H
#define Bitset_H
#include <Arduino.h>

//Number of bits in Data type to initialize must be >= number of bits desired.
template<typename Data_Type>
class Bitset
{
public:
	//Initialize. Must be a number in binary form to make any sense.
	Bitset(byte init_number = 0)
	{
		number_ = init_number;
	};

	//Sets a bit at the desired bit_num with value = 1 or 0
	inline void Set(byte bit_num, byte value)
	{
		if(value == 1) 
		{
			//turn on
			number_ |= (1 << bit_num);
		}
		else if(value == 0)
		{
			//turn off
			number_ &= ~(1 << bit_num);
		}
	};

	//Sets the number of the bitset. Must be number in binary to make any sense
	inline void Set(byte value)
	{
		number_ = value;
	};

	//Test whether a given bit is set and returns true if set, false otherwise
	inline bool Test(byte bit_num) const
	{
		return (number_ & (1 << bit_num)) > 0;
	};

	//Gets the number stored
	inline Data_Type Get() const
	{
		return number_;
	};

	//Sets the bitset equal to another
	inline Bitset& operator= (const Bitset& rhs)
	{
		number_ = rhs.number_;
		return *this;
	};

	//Test if two bitsets are equal
	inline bool operator== (const Bitset& rhs) const
	{
		return number_ == rhs.number_;
	};

private:
	Data_Type number_;
};
	

#endif