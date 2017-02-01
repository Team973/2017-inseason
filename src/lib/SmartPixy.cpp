#include "lib/SmartPixy.h"
#include "unistd.h"

namespace frc973 {

namespace Pixy {

void Block::print() {
    int i, j;
    char buf[128], sig[6], d;
    bool flag;  
    if (signature > PIXY_MAX_SIGNATURE) {
        // convert signature number to an octal string
        for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
            d = (signature >> i) &0x07;
            if (d > 0 && !flag)
                flag = true;
            if (flag)
                sig[j++] = d + '0';
        }
        sig[j] = '\0';  
        sprintf(buf,
                "CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n",
                sig, signature, x, y, width, height, angle);
    }
    else // regular block.  Note, angle is always zero, so no need to print
        sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d\n",
                signature, x, y, width, height);   
    printf(buf);
}

PixyLinkI2C::PixyLinkI2C(uint8_t address, I2C::Port port):
        Wire(port, address) {
}

uint16_t PixyLinkI2C::getWord() {
    uint8_t c[2];
    Wire.ReadOnly(2, c);
    uint16_t w = (c[1] << 8) + c[0];
    return w;
}

uint8_t PixyLinkI2C::getByte(uint8_t out) {
    uint8_t c;
    Wire.ReadOnly(1, &c);
    return c;
}

int8_t PixyLinkI2C::send(uint8_t *data, uint8_t len) {
    Wire.WriteBulk(data, len);
    return len;
}

PixyLinkSPI::PixyLinkSPI(SPI::Port port):
        wire(port) {
    wire.SetMSBFirst();
    wire.SetClockActiveHigh();
    wire.SetSampleDataOnRising();
    wire.SetChipSelectActiveLow();
}

uint16_t PixyLinkSPI::getWord() {
    // ordering is big endian because Pixy is sending 16 bits through SPI 
    uint16_t w;
    uint8_t c, cout = 0;

    if (outLen) {
        w = getByte(PIXY_SYNC_BYTE_DATA);
        cout = outBuf[outReadIndex++];
        outLen--;
        if (outReadIndex == PIXY_OUTBUF_SIZE)
            outReadIndex = 0; 
    }
    else {
        w = getByte(PIXY_SYNC_BYTE); // send out sync byte
    }
    w <<= 8;
    c = getByte(cout); // send out data byte
    w |= c;
    printf(" getWord 0x%x ", w);

    return w;
}

uint8_t PixyLinkSPI::getByte(uint8_t send) {
    uint8_t recv;
    wire.Transaction(&send, &recv, 1);

    printf(" transaction {%x -> %x} ", send, recv);
    return recv;
}

int8_t PixyLinkSPI::send(uint8_t *data, uint8_t len) {
    int i;

    // check to see if we have enough space in our circular queue
    if (outLen + len > PIXY_OUTBUF_SIZE)
        return -1;

    outLen += len;
    for (i = 0; i < len; i++) {
        outBuf[outWriteIndex++] = data[i];
        if (outWriteIndex == PIXY_OUTBUF_SIZE)
            outWriteIndex = 0;
    }
    return len;
}

PixyDriver::PixyDriver(PixyLink *_link) {
    skipStart = false;
    blockCount = 0;
    blockArraySize = PIXY_INITIAL_ARRAYSIZE;
    blocks = (Block *)malloc(sizeof(Block)*blockArraySize);
    link = _link;
}

PixyDriver::~PixyDriver() {
    delete link;
    free(blocks);
}

bool PixyDriver::GetStart() {
    uint16_t w, lastw;

    lastw = 0xffff;

    int tries = 0;
    printf("GETTING START:\n");
    while(tries++ < 30) {
        w = link->getWord();
        if (w == 0 && lastw == 0) {
            printf("no data... giving up\n\n");
            return false;
        }   
        else if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
            blockType = NORMAL_BLOCK;
            printf("found start block\n\n");
            return true;
        }
        else if (w == PIXY_START_WORD_CC && lastw == PIXY_START_WORD) {
            blockType = CC_BLOCK;
            printf("found start block\n\n");
            return true;
        }
        else if (w==PIXY_START_WORDX) {
            printf("reorder: %x ", link->getByte(0));
        }
        lastw = w; 
    }
    printf("Tried 30 initializations and it didn't work so giving up\n\n");
    SetLED(30, 30, 30);
    return false;
}

void PixyDriver::Resize() {
    blockArraySize += PIXY_INITIAL_ARRAYSIZE;
    Block *newBlocks = (Block *)realloc(blocks, sizeof(Block)*blockArraySize);
    if (blocks == nullptr) {
        free(blocks);
    }
    else {
        blocks = newBlocks;
    }
}
    
uint16_t PixyDriver::GetBlocks(uint16_t maxBlocks) {
    uint8_t i;
    uint16_t w, checksum, sum;
    Block *block;
  
    if (!skipStart) {
        if (GetStart() == false)
            printf(" fail start ");
            return 0;
    }
    else
        skipStart = false;
  
    for(blockCount = 0; blockCount < maxBlocks && blockCount < PIXY_MAXIMUM_ARRAYSIZE; ) {
        checksum = link->getWord();
        if (checksum == PIXY_START_WORD) {
            skipStart = true;
            blockType = NORMAL_BLOCK;
            //Serial.println("skip");
            return blockCount;
        }
        else if (checksum == PIXY_START_WORD_CC) {
            skipStart = true;
            blockType = CC_BLOCK;
            return blockCount;
        }
        else if (checksum == 0)
            return blockCount;

        if (blockCount > blockArraySize)
            Resize();

        block = blocks + blockCount;

        for (i = 0, sum = 0; i < sizeof(Block) / sizeof(uint16_t); i++) {
            if (blockType == NORMAL_BLOCK && i >= 5) {
                block->angle = 0;
                break;
            }
            w = link->getWord();
            sum += w;
            *((uint16_t *)block + i) = w;
        }

        if (checksum == sum) {
            blockCount++;
        }
        else {
            printf("cs error");
        }

        block->print();
  
        w = link->getWord();
        if (w == PIXY_START_WORD)
            blockType = NORMAL_BLOCK;
        else if (w == PIXY_START_WORD_CC)
            blockType = CC_BLOCK;
        else
            return blockCount;
    }
    return blockCount;
}

int8_t PixyDriver::SetServos(uint16_t s0, uint16_t s1) {
    uint8_t outBuf[6];

    outBuf[0] = 0x00;
    outBuf[1] = 0xff; 
    *(uint16_t *)(outBuf + 2) = s0;
    *(uint16_t *)(outBuf + 4) = s1;

    return link->send(outBuf, 6);
}

int8_t PixyDriver::SetBrightness(uint8_t brightness) {
    uint8_t outBuf[3];

    outBuf[0] = 0x00;
    outBuf[1] = 0xfe; 
    outBuf[2] = brightness;

    return link->send(outBuf, 3);
}

int8_t PixyDriver::SetLED(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t outBuf[5];

    outBuf[0] = 0x00;
    outBuf[1] = 0xfd; 
    outBuf[2] = r;
    outBuf[3] = g;
    outBuf[4] = b;

    return link->send(outBuf, 5);
}

}

}
