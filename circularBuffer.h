#ifndef __CIRCULAR_BUFFER_H_
#define __CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct CIRCULAR_BUFFER
{
    uint8_t *buffer;
    uint32_t size;
    uint32_t positionToWrite; // 当前写入数据的位置
    uint32_t positionToRead;  // 读取读取数据的位置
    uint32_t positionInUsed;  // 正在使用的位置
} CIRCULAR_BUFFER_s;

int circularBufferInit(CIRCULAR_BUFFER_s *circularBuffer, uint8_t *buffer, uint32_t size);
void circularBufferWrite(CIRCULAR_BUFFER_s *circularBuffer, uint8_t *data, uint32_t length);
uint32_t circularBufferRead(CIRCULAR_BUFFER_s *circularBuffer, uint32_t *dataAdder, uint32_t length);
bool isCircularBufferEmpty(CIRCULAR_BUFFER_s *circularBuffer);

#endif

