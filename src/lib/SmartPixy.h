#pragma once

//Heavily modified based on example code provided below:
//http://cmucam.org/projects/cmucam5/wiki/Porting_Guide

#include "WPILib.h"

using namespace frc;

namespace frc973 {


#define PIXY_I2C_DEFAULT_ADDR           0x54

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L	//x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199

// RC-servo values - not needed unless you want to use servo to face the goal instead of moving the whole robot
#define PIXY_RCS_MIN_POS            0L
#define PIXY_RCS_MAX_POS            1000L
#define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)


class Pixy
{

public:

	Pixy();
	~Pixy();

	struct Block
	{
		void print(); //Prints block structure - prints psixy stat(xy coordinates, height, width, etc.)

		uint16_t signature; //Identification number for your object - you could set it in the pixymon
		uint16_t x; //0 pixel - 320 pixel
		uint16_t y; //0 pixel - 200 pixel
		uint16_t width;
		uint16_t height;
		uint16_t angle; //Only appears when using Color Code
	};

	enum BlockType
	{
		NORMAL_BLOCK, //Regular color recognition
		CC_BLOCK	  //Color-Code Recognition (gives how much object is tilted)
	};

	bool getStart(); //Checking if the the frame is a new frame
	uint16_t getWord(); //Getting two Bytes from Pixy (The full information)
	uint8_t getByte(); //Gets a byte from Pixy
	uint16_t getBlocks(uint16_t maxBlocks); //Gives how many (signature) object is detected

private:

	I2C* i2c; //Declare i2c
	BlockType blockType;// it is the enum on the top
	bool  skipStart;	//skips to check 0xaa55, which is byte that tells pixy it is start of new frame
	uint16_t blockCount; //How many signatured objects are there?
public:
	Block blocks[100]; //array that stores blockCount array

};

}
