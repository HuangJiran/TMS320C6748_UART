

/* Circular buffer 循环缓冲区 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "circularBuffer.h"

/* 循环缓冲区初始化，缓冲区需要用户在外部定义好再代入 */
int circularBufferInit(CIRCULAR_BUFFER_s *circularBuffer, uint8_t *buffer, uint32_t size)
{
    if (circularBuffer == NULL || buffer == NULL || size == 0)
        return -1;

    circularBuffer->buffer = buffer;
    circularBuffer->size = size;
    circularBuffer->positionToWrite = 0;
    circularBuffer->positionToRead = 0;

    return 0;
}

/* 缓冲区释放，将使用中的缓冲区数据释放 */
void circularBufferRelease(CIRCULAR_BUFFER_s *circularBuffer)
{
    circularBuffer->positionInUsed = circularBuffer->positionToRead;
}

/**
 * @brief  循环缓冲区写入，向缓冲区中写入指定字节的数据（单任务调用安全，多任务调用需要互斥）
 * @param  *circularBuffer 缓冲区控制指针
 * @param  *data : 数据地址
 * @param  length: 数据长度
 * @retval none
 */
void circularBufferWrite(CIRCULAR_BUFFER_s *circularBuffer, uint8_t *data, uint32_t length)
{
    uint32_t currentPositionInUes; // 当前读取数据位置
    uint32_t writeLength;

    /* 当前读取数据的位置 */
    currentPositionInUes = circularBuffer->positionInUsed;

    /* 写入的数据位置是否在读取数据位置之前 */
    if (currentPositionInUes <= circularBuffer->positionToWrite)
    {
        /* 数据存储区剩余大小够存放当前数据 */
        if (circularBuffer->size - circularBuffer->positionToWrite >= length)
        {
            /* 将数据存放到缓冲 */
            memcpy(circularBuffer->buffer + circularBuffer->positionToWrite, data, length);

            /* 写入数据位置移动到当前数据末尾 */
            circularBuffer->positionToWrite += length;

            /* 数据刚到最末尾，数据回到起始 */
            if (circularBuffer->positionToWrite >= circularBuffer->size)
            {
                /* 数据写入位置到开始位置 */
                circularBuffer->positionToWrite = 0;
            }
        }
        else
        {
            /* 计算数据存储区后端能够写入的数据长度 */
            writeLength = circularBuffer->size - circularBuffer->positionToWrite;

            /* 将存储区后端填满 */
            memcpy(circularBuffer->buffer + circularBuffer->positionToWrite, data, writeLength);

            /* 剩余没有写入的数据 */
            length -= writeLength;

            /* 读取位置之前能够容纳剩余的数据 */
            if (length <= currentPositionInUes)
            {
                /* 剩余没有写入的数据写入缓冲区前端 */
                memcpy(circularBuffer->buffer, data + writeLength, length);

                /* 更新数据写入位置 */
                circularBuffer->positionToWrite = length;
            }
            else
            {
                /* 缓冲区前端不够完全写入剩余的数据，能写入多少就写多少，写入不了的部分直接丢弃 */
                memcpy(circularBuffer->buffer, data + writeLength, length - currentPositionInUes);

                /* 写入位置等于读取位置 */
                circularBuffer->positionToWrite = currentPositionInUes;
            }
        }
    }
    else
    {
        /* 数据存储区剩余大小够存放当前数据 */
        if (currentPositionInUes - circularBuffer->positionToWrite >= length)
        {
            /* 将数据存放到缓冲区 */
            memcpy(circularBuffer->buffer + circularBuffer->positionToWrite, data, length);

            /* 写入数据位置移动到当前数据末尾 */
            circularBuffer->positionToWrite += length;
        }
        else
        {
            /* 缓冲区前端不够完全写入剩余的数据，能写入多少就写多少，写入不了的部分直接丢弃 */
            memcpy(circularBuffer->buffer + circularBuffer->positionToWrite, data, currentPositionInUes - circularBuffer->positionToWrite);

            /* 写入位置等于读取位置 */
            circularBuffer->positionToWrite = currentPositionInUes;
        }
    }
}

bool isCircularBufferEmpty(CIRCULAR_BUFFER_s *circularBuffer)
{
    /* 判断缓冲区是否为空 */
    if (circularBuffer->positionToRead == circularBuffer->positionToWrite)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief  循环缓冲区读出，从缓冲区中读取指定字节的数据，返回实际读取的数据（单任务调用安全，多任务调用需要互斥）
 * @param  *circularBuffer 缓冲区控制指针
 * @param  *dataAdder : 返回读取数据的地址
 * @param  length: 需要读取最大的数据长度
 * @retval 实际读取的数据长度
 */
uint32_t circularBufferRead(CIRCULAR_BUFFER_s *circularBuffer, uint32_t *dataAdder, uint32_t length)
{
    uint32_t currentPositionToWrite; // 当前写入数据位置
    uint32_t readLength;

    /* 将之前使用中的数据释放 */
    circularBuffer->positionInUsed = circularBuffer->positionToRead;

    currentPositionToWrite = circularBuffer->positionToWrite;

    /* 判断缓冲区是否为空 */
    if (circularBuffer->positionToRead == circularBuffer->positionToWrite)
    {
        return 0;
    }

    /* 写入位置是否在读取位置之前 */
    if (currentPositionToWrite > circularBuffer->positionToRead)
    {
        if (currentPositionToWrite - circularBuffer->positionToRead > length)
        {
            *dataAdder = (uint32_t)(circularBuffer->buffer) + circularBuffer->positionToRead;
            circularBuffer->positionToRead += length;
            return length;
        }

        readLength = currentPositionToWrite - circularBuffer->positionToRead;
        *dataAdder = (uint32_t)(circularBuffer->buffer) + circularBuffer->positionToRead;
        circularBuffer->positionToRead = currentPositionToWrite;
        return readLength;
    }
    else if (currentPositionToWrite < circularBuffer->positionToRead)
    {
        if (circularBuffer->size - circularBuffer->positionToRead > length)
        {
            *dataAdder = (uint32_t)(circularBuffer->buffer) + circularBuffer->positionToRead;
            circularBuffer->positionToRead += length;
            return length;
        }

        /* 缓冲区后部的数据不够读取长度时，先将缓冲区后部的数据先读出，下次读取时再从开始位置读取 */
        readLength = circularBuffer->size - circularBuffer->positionToRead;
        *dataAdder = (uint32_t)(circularBuffer->buffer) + circularBuffer->positionToRead;
        circularBuffer->positionToRead = 0;
        return readLength;
    }

    return 0;
}

