#include <Servo.h>
#define TARGET_BOARD_IDENTIFIER SENTIBOTS_IDENTIFIER
#include "serial_msp.h"

//unsigned int cycleDelay = 5;

Servo m1;
Servo m2;
Servo s1;
Servo s2;
Servo s3;
Servo s4;

mspPort_t currentPort;

static void serialize8(uint8_t a) {
  Serial.write(a);
  currentPort.checksum ^= a;
}

static void serialize16(uint16_t a) {
  serialize8((uint8_t)(a >> 0));
  serialize8((uint8_t)(a >> 8));
}

static void serialize32(uint32_t a) {
  serialize16((uint16_t)(a >> 0));
  serialize16((uint16_t)(a >> 16));
}

static uint8_t read8(void) {
  return currentPort.inBuf[currentPort.indRX++] & 0xff;
}

static uint16_t read16(void) {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}

static uint32_t read32(void) {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

static void headSerialResponse(uint8_t err, uint8_t responseBodySize) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  currentPort.checksum = 0;
  serialize8(responseBodySize);
  serialize8(currentPort.cmdMSP);
}

static void headSerialReply(uint8_t responseBodySize) {
  headSerialResponse(0, responseBodySize);
}

static void headSerialError(uint8_t responseBodySize) {
  headSerialResponse(1, responseBodySize);
}

bool processOutCommand(uint8_t cmdMSP) {
  switch (cmdMSP) {
    case 0:
      return false;
      break;
    default:
      return false;
  }
  return true;
}

static bool processInCommand(void) {
  switch (currentPort.cmdMSP) {
    case MSP_SET_RAW_COAX:
      s4.write(45);
      m1.writeMicroseconds(read16());
      m2.writeMicroseconds(read16());
      s1.write(read16());
      s2.write(read16());
      s3.write(read16());
      s4.write(read16());
      break;
    default:
      return false;
  }
  headSerialReply(0);
  return true;
}

void tailSerialReply(void) {
  serialize8(currentPort.checksum);
}

void mspProcessReceivedCommand() {
  if (!(processOutCommand(currentPort.cmdMSP) || processInCommand())) {
    headSerialError(0);
  }
  tailSerialReply();
  currentPort.c_state = IDLE;
}

static bool mspProcessReceivedData(uint8_t c) {
  if (currentPort.c_state == IDLE) {
    if (c == '$') {
      currentPort.c_state = HEADER_START;
    } else {
      return false;
    }
  } else if (currentPort.c_state == HEADER_START) {
    currentPort.c_state = (c == 'M') ? HEADER_M : IDLE;
  } else if (currentPort.c_state == HEADER_M) {
    currentPort.c_state = (c == '<') ? HEADER_ARROW : IDLE;
  } else if (currentPort.c_state == HEADER_ARROW) {
    if (c > MSP_PORT_INBUF_SIZE) {
      currentPort.c_state = IDLE;
    } else {
      currentPort.dataSize = c;
      currentPort.offset = 0;
      currentPort.checksum = 0;
      currentPort.indRX = 0;
      currentPort.checksum ^= c;
      currentPort.c_state = HEADER_SIZE;
    }
  } else if (currentPort.c_state == HEADER_SIZE) {
    currentPort.cmdMSP = c;
    currentPort.checksum ^= c;
    currentPort.c_state = HEADER_CMD;
  } else if (currentPort.c_state == HEADER_CMD && currentPort.offset < currentPort.dataSize) {
    currentPort.checksum ^= c;
    currentPort.inBuf[currentPort.offset++] = c;
  } else if (currentPort.c_state == HEADER_CMD && currentPort.offset >= currentPort.dataSize) {
    //if (currentPort.checksum == c) {
    currentPort.c_state = COMMAND_RECEIVED;
    //} else {
    //  currentPort.c_state = IDLE;
    //}
  }
  return true;
}


void setup() {
  Serial.begin(115200);
  // PD3 PD5 PD6 PB2 PB3
  // 3   5   6   10  11
  m1.attach(3);
  m2.attach(10); // 9
  s1.attach(5);
  s2.attach(6);
  s3.attach(11);
  s4.attach(9); // 10
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  s1.write(90);
  s2.write(90);
  s3.write(90);
  s4.write(90);
  //memset(currentPort, 0, sizeof(mspPort_t));
  currentPort.c_state = IDLE;
}

void loop() {
  while (Serial.available() > 0) {

    uint8_t c = Serial.read();
    //bool consumed = mspProcessReceivedData(c);
    mspProcessReceivedData(c);

    if (currentPort.c_state == COMMAND_RECEIVED) {
      mspProcessReceivedCommand();
      break; // process one command at a time so as not to block.
    }

  }
}
