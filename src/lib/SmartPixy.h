#pragma once

//Heavily modified based on example code provided below:
//http://cmucam.org/projects/cmucam5/wiki/Porting_Guide

#include "WPILib.h"

using namespace frc;

namespace frc973 {

namespace Pixy {

// Pixy x-y position values
static constexpr unsigned long int PIXY_MIN_X = 0L;
static constexpr unsigned long int PIXY_MAX_X = 319L;
static constexpr unsigned long int PIXY_MIN_Y = 0L;
static constexpr unsigned long int PIXY_MAX_Y = 199L;

class PixyLink {
public:
    PixyLink() {}
    virtual ~PixyLink() {}

    virtual uint16_t getWord() = 0;
    virtual uint8_t getByte(uint8_t out) = 0;
    virtual int8_t send(uint8_t *data, uint8_t len) = 0;
};

static constexpr int PIXY_MAX_SIGNATURE = 7;
static constexpr uint16_t PIXY_DEFAULT_ARGVAL = 0xffff;
static constexpr int PIXY_INITIAL_ARRAYSIZE = 30;
static constexpr int PIXY_MAXIMUM_ARRAYSIZE = 130;
static constexpr uint16_t PIXY_START_WORD = 0xaa55;
static constexpr uint16_t PIXY_START_WORD_CC = 0xaa56;
static constexpr uint16_t PIXY_START_WORDX = 0x55aa;

class PixyLinkI2C : public PixyLink
{
public:
    PixyLinkI2C(uint8_t address=PIXY_I2C_DEFAULT_ADDR,
            I2C::Port port=PIXY_I2C_DEFAULT_PORT);
    ~PixyLinkI2C() {}

    uint16_t getWord() override;
    uint8_t getByte(uint8_t out) override;
    int8_t send(uint8_t *data, uint8_t len) override;

    static constexpr uint8_t PIXY_I2C_DEFAULT_ADDR = 0x54;
    static constexpr I2C::Port PIXY_I2C_DEFAULT_PORT = I2C::Port::kOnboard;
private:
    I2C Wire;
};

class PixyLinkSPI : public PixyLink
{
public:
    PixyLinkSPI(SPI::Port port);
    ~PixyLinkSPI() {}

    uint16_t getWord() override;
    uint8_t getByte(uint8_t out) override;
    int8_t send(uint8_t *data, uint8_t len) override;
private:
    SPI wire;

    static constexpr uint8_t PIXY_SYNC_BYTE = 0x5a;
    static constexpr uint8_t PIXY_SYNC_BYTE_DATA = 0x5b;
    static constexpr size_t PIXY_OUTBUF_SIZE = 64;

    uint8_t outBuf[PIXY_OUTBUF_SIZE];
    uint8_t outLen = 0;
    uint8_t outWriteIndex = 0;
    uint8_t outReadIndex = 0;
};

enum BlockType {
  NORMAL_BLOCK,
  CC_BLOCK
};

struct Block {
    void print();

    uint16_t signature;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t angle;
};

class PixyDriver {
public:
    PixyDriver (PixyLink *_link);
    ~PixyDriver();

    uint16_t GetBlocks(uint16_t maxBlocks=1000);
    int8_t SetServos(uint16_t s0, uint16_t s1);
    int8_t SetBrightness(uint8_t brightness);
    int8_t SetLED(uint8_t r, uint8_t g, uint8_t b);

    Block *blocks;
private:
    bool GetStart();
    void Resize();

    PixyLink *link;
    bool skipStart;
    BlockType blockType;
    uint16_t blockCount;
    uint16_t blockArraySize;
};

}

}
