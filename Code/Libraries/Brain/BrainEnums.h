#ifndef BrainEnums_H
#define BrainEnums_H

#include <Arduino.h> //for byte
#include "Directions.h"

//Check if there are any non-zero flags in an enum.
//Given an an enum type and the date type used to store the enum,
//Cast the flags to the data type and if it's not 0 there are flags.
template<typename Enum, typename DataType>
inline bool any_flags(Enum flags)
{
	return static_cast<DataType>(flags) != static_cast<DataType>(0);
};

//Board positions
enum Position : byte
{
	START,
	RED,
	YELLOW,
	CROSSROAD,
	CITY_R,
	CITY_L,
	MEXICO,
	USA,
	FRONTIER,
	GRASS_S,
	GRASS_N
};

//Flags for wall-follow stopping conditions
enum class StopConditions : byte
{
	NONE = 0,
	GAP = 1 << 0,
	PIXY = 1 << 1,
	FRONT = 1 << 2
};
inline StopConditions operator|(StopConditions a, StopConditions b)
{
	return static_cast<StopConditions>(static_cast<byte>(a) | static_cast<byte>(b));
};
inline StopConditions operator&(StopConditions a, StopConditions b)
{
	return static_cast<StopConditions>(static_cast<byte>(a) & static_cast<byte>(b));
};

//Enum of what state the GoAtoB is in.
enum GoAToBState
{
	GOING = 0,
	STOP_GAP,
	STOP_PIXY,
	STOP_FRONT,
	ERROR,
	SUCCESS
};

enum ActionResult
{
	ACT_GOING = 0,
	ACT_GAP = 1 << 0,
	ACT_PIXY = 1 << 1,
	ACT_FRONT = 1 << 2,
	ACT_ERROR = 1 << 3,
	ACT_SUCCESS = 1 << 4
};

#endif