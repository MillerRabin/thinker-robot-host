#pragma once
#include "config.h"
#include "driver/twai.h"

typedef twai_message_t CanFrame;

class CanRingBuffer {
  private:
    CanFrame buffer[CAN_RX_BUFFER_SIZE];
    volatile uint16_t head = 0;
    volatile uint16_t tail = 0;
  public:
    inline bool push(const CanFrame &frame) {
      uint16_t next = (head + 1) % CAN_RX_BUFFER_SIZE;
      if (next == tail)
        return false;
      buffer[head] = frame;
      head = next;
      return true;
    }
    inline bool pop(CanFrame *frame) {
      if (tail == head)
        return false;

      *frame = buffer[tail];
      tail = (tail + 1) % CAN_RX_BUFFER_SIZE;

      return true;
    }
    inline uint16_t size() const {
      return (head + CAN_RX_BUFFER_SIZE - tail) % CAN_RX_BUFFER_SIZE;
    }
};
