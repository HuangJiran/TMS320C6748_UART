#include "soc_C6748.h"
#include "TL6748.h"

#include "psc.h"
#include "uart.h"
#include "hal_uart.h"
#include "interrupt.h"
#include "hw_types.h"

#include "edma3_drv.h"
#include <ti/sysbios/family/c64p/Cache.h>
#include "edma_event.h"
#include "hw_edma3cc.h"

#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <uart/include/Uart.h>

#define UART0_HWI_NUM (11)
#define UART1_HWI_NUM (13)
#define UART2_HWI_NUM (12)

EDMA3_DRV_Handle hEdma = NULL;
UART_HANDLE_s uartHandle[HAL_UART_MAX];

void UARTIsr(UArg baseAdd);

/**
 * @brief  串口硬件初始化，创建串口HWI中断，硬件相关参数波特率、数据长度、校验等在此函数中设置
 * @param  uartPort 串口号
 * @retval NULL
 */
int uartInit(HAL_UART_PORT_e uartPort)
{
    unsigned int baseAdd;    // 串口基地址
    unsigned int moduleId;   // 模块ID
    unsigned int baudRate;   // 波特率
    Uart_CharLen charLen;     // 字符长度
    Uart_NumStopBits stopBits; // 停止位
    Uart_Parity parity;      // 校验

    unsigned int uartHwiNum; // 中断号
    unsigned int eventId;    // CPU事件号

    /* 各个串口初始化参数 */
    switch (uartPort)
    {
    case HAL_UART0:
        baseAdd = SOC_UART_0_REGS;
        moduleId = HW_PSC_UART0;
        baudRate = Uart_BaudRate_115_2K;
        charLen = Uart_CharLen_8;
        stopBits = Uart_NumStopBits_1;
        parity = Uart_Parity_NONE;
        uartHwiNum = UART0_HWI_NUM;
        eventId = SYS_INT_UART0_INT;
        break;

    case HAL_UART1:
        baseAdd = SOC_UART_1_REGS;
        moduleId = HW_PSC_UART1;
        baudRate = Uart_BaudRate_460_8K;
        charLen = Uart_CharLen_8;
        stopBits = Uart_NumStopBits_1;
        parity = Uart_Parity_NONE;
        uartHwiNum = UART1_HWI_NUM;
        eventId = SYS_INT_UART1_INT;
        break;

    case HAL_UART2:
        baseAdd = SOC_UART_2_REGS;
        moduleId = HW_PSC_UART2;
        baudRate = Uart_BaudRate_115_2K;
        charLen = Uart_CharLen_8;
        stopBits = Uart_NumStopBits_1;
        parity = Uart_Parity_ODD;
        uartHwiNum = UART2_HWI_NUM;
        eventId = SYS_INT_UART2_INT;
#ifdef USE_LOG
        baudRate = 460800;
        parity = Uart_Parity_NONE;
#endif
        break;

    default:
        return -1;
    }

    /* Enabling the PSC for UART0. */
    PSCModuleControl(SOC_PSC_1_REGS, moduleId, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);

    /* Setup PINMUX */
    UARTPinMuxSetup(uartPort, FALSE);

    /* Enabling the transmitter and receiver*/
    UARTEnable(baseAdd);

    /* Configuring the UART parameters*/
    UARTConfigSetExpClk(baseAdd, SOC_SYSCLK_2_FREQ, baudRate, (((UInt32)charLen) | ((UInt32)stopBits) | ((UInt32)parity)), UART_OVER_SAMP_RATE_16);

    /* Enabling the FIFO and flushing the Tx and Rx FIFOs.*/
    UARTFIFOEnable(baseAdd);

    /* Setting the UART Receiver Trigger Level*/
    UARTDMAEnable(baseAdd, UART_RX_TRIG_LEVEL_14 | UART_DMAMODE | UART_FIFO_MODE);

    /* Enable the Interrupts in UART.*/
    UARTIntEnable(baseAdd, UART_INT_LINE_STAT | UART_INT_RXDATA_CTI); // UART_INT_TX_EMPTY

    /* 调用中断服务函数，清除已有中断 */
    UARTIsr(baseAdd);

    /* 创建串口中断 */
    Error_Block eb;
    Error_init(&eb);

    Hwi_Handle uartHwi;
    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);
    hwiParams.eventId = eventId;
    hwiParams.arg = baseAdd;    // 串口中断代入参数，串口基地址
    hwiParams.enableInt = TRUE;
    uartHwi = Hwi_create(uartHwiNum, &UARTIsr, &hwiParams, &eb);
    if (uartHwi == NULL)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  读取串口数据，中断中调用，读取串口接收FIFO中数据写入缓冲区，然后通过信号量通知任务数据已接收
 * @param  baseAdd 串口基地址
 * @retval NULL
 */
void uartReadData(unsigned int baseAdd)
{
#define UART_FIFO_DEPTH (16)    // 串口FIFO深度16，能存储16字节数据
    unsigned char buffer[UART_FIFO_DEPTH];
    uint32_t count = 0;
    UART_HANDLE_s *instHandle;

    switch (baseAdd)
    {
    case SOC_UART_0_REGS:
        instHandle = &uartHandle[0];
        break;

    case SOC_UART_1_REGS:
        instHandle = &uartHandle[1];
        break;

    case SOC_UART_2_REGS:
        instHandle = &uartHandle[2];
        break;

    default:
        instHandle = &uartHandle[0];
        break;
    }

    for (count = 0; count < UART_FIFO_DEPTH; count++)
    {
        /* 判断FIFO中是否有数据 */
        if (UARTCharsAvail(baseAdd))
        {
            /* 将数据全部读取数据出来 */
            buffer[count] = UARTCharGetNonBlocking(baseAdd);
        }
        else
        {
            break;
        }
    }

    /* 读取的数据不为0时，将数据写入缓冲区再通知应用程序 */
    if (count != 0)
    {
        /* 将数据写入缓冲区 */
        circularBufferWrite(&instHandle->uartRxBufferCtrl, buffer, count);

        /* 通知应用程序，数据已更新 */
        Semaphore_post(instHandle->uartRxSem);
    }
}

/**
 * @brief  串口中断服务函数
 * @param  baseAdd 串口基地址，创建HWI时设置
 * @retval NULL
 */
void UARTIsr(UArg baseAdd)
{
    unsigned int int_id = 0;
    UInt32 intrCnt = 0;

#define Uart_MAX_ISR_LOOP (5)
    /* 循环处理，直到将所有中断处理完 */
    for (intrCnt = 0; intrCnt < Uart_MAX_ISR_LOOP; intrCnt++)
    {
        /* 检查是否有中断触发 */
        if (!(HWREG(baseAdd + UART_IIR) & UART_IIR_IPEND))
        {
            /* This determines the cause of UART interrupt. */
            int_id = UARTIntStatus(baseAdd);

            switch (int_id)
            {
            /* Checked if the cause is transmitter empty condition. */
            case UART_INTID_TX_EMPTY:
                /* Disable the Transmitter interrupt in UART. */
                UARTIntDisable(baseAdd, UART_INT_TX_EMPTY);
                break;

                /* Check if the cause is receiver data condition. */
            case UART_INTID_RX_LINE_STAT:
                while (UARTRxErrorGet(baseAdd))
                {
                    /* Read a byte from the RBR if RBR has data. */
                    UARTCharGetNonBlocking(baseAdd);
                }
                break;

                /* 串口接收数据达到FIFO阈值  Receiver Data Available */
            case UART_INTID_RX_DATA:
                /* 串口接收超时 Character Timeout interrupt*/
            case UART_INTID_CTI:
                uartReadData(baseAdd);
                break;

            default:
                break;
            }
        }
        else
        {
            break;
        }
    }
}

/**
 * @brief  串口DMA中断服务函数
 * @param  tcc DMA通道
 * @param  status 传输状态
 * @param  appData 应用数据，此处为串口参数指针
 * @retval NULL
 */
static void uart_edma_callback(unsigned int tcc, EDMA3_RM_TccStatus status, void *appData)
{
    (void)tcc;
    (void)appData;

    UInt32 value = 0;
    UInt32 key = 0;
    UART_HANDLE_s *instHandle = (UART_HANDLE_s *)appData;

    /* enter critical section */
    key = (UInt32)_disable_interrupts();

    if (EDMA3_RM_XFER_COMPLETE == status)
    {
        /* 通知数据发送任务，数据已更新 */
        Semaphore_post(instHandle->uartTxEdmaSem);
    }
    else
    {
        /* TODO：传输错误处理，暂时按照正常传输处理 */
        Semaphore_post(instHandle->uartTxEdmaSem);
    }

    EDMA3_DRV_disableLogicalChannel((EDMA3_DRV_Handle)hEdma, tcc, EDMA3_DRV_TRIG_MODE_EVENT);

    /* Write into FCR (Disable only DMAMODE but do not clear the FIFO in UART */
    value &= ~(UART_FCR_DMAMODE1 | UART_FCR_TXCLR | UART_FCR_RXCLR);

    /* Do not Disable FIFO	*/
    value |= UART_FCR_FIFOEN;

    *(unsigned int *)(instHandle->uartBaseAdd + UART_FCR) = value;

    value |= UART_FCR_RXFIFTL_CHAR14 << UART_FCR_RXFIFTL_SHIFT;

    /* Re Enable DMA */
    value |= (UART_FCR_DMAMODE1 | UART_FCR_FIFOEN);

    *(unsigned int *)(instHandle->uartBaseAdd + UART_FCR) = value;

    _restore_interrupts(key);
}

/* External Declaration */
EDMA3_DRV_Handle edma3init (unsigned int edma3Id, EDMA3_DRV_Result *errorCode);

/**
 * @brief  串口硬件初始化
 * @param  uartPort 串口端口号
 * @retval DMA初始化状态，成功返回EDMA3_DRV_SOK
 */
EDMA3_DRV_Result uart_edma_init(HAL_UART_PORT_e uartPort)
{
    EDMA3_DRV_Result result = EDMA3_DRV_SOK;
    EDMA3_DRV_PaRAMRegs *paramSet = &uartHandle[uartPort].edma3Param;

    unsigned int tccNum = 0;
    unsigned int chNum = 0;
    unsigned char temp = 0;
    unsigned int destAddr = uartHandle[uartPort].uartBaseAdd;

    switch (uartPort)
    {
    case HAL_UART0:
        tccNum = EDMA3_CHA_UART0_TX;
        chNum = EDMA3_CHA_UART0_TX;
        break;

    case HAL_UART1:
        tccNum = EDMA3_CHA_UART1_TX;
        chNum = EDMA3_CHA_UART1_TX;
        break;

    case HAL_UART2:
        tccNum = EDMA3_CHA_UART2_TX;
        chNum = EDMA3_CHA_UART2_TX;
        break;

    default:
        return -1;
    }

    if (hEdma == NULL)
    {
        hEdma = edma3init(0, &result);
    }

    if (result == EDMA3_DRV_SOK)
    {
        /* 请求DMA通道，在此处代入DMA回调函数地址和回调函数参数 */
        result = EDMA3_DRV_requestChannel(hEdma, &chNum, &tccNum,
                                          (EDMA3_RM_EventQueue)0, &uart_edma_callback,
                                          (void *)&uartHandle[uartPort]);
    }

    if (result == EDMA3_DRV_SOK)
    {
        /* Fill the PaRAM Set with transfer specific information */
        paramSet->srcAddr = (unsigned int)(&temp);
        paramSet->destAddr = (unsigned int)(destAddr); // UART_RBR_THR_REG

        paramSet->srcBIdx = (short)1u;
        paramSet->destBIdx = (short)0u;
        paramSet->srcCIdx = (short)0u;
        paramSet->destCIdx = (short)0u;

        paramSet->aCnt = (short)1u;
        paramSet->bCnt = (short)1u;
        paramSet->cCnt = (short)1u;

        paramSet->bCntReload = (unsigned short)0u;
        paramSet->linkAddr = 0xFFFFu;

        paramSet->opt = 0x00000000u;
        paramSet->opt |= (EDMA3CC_OPT_DAM);
        paramSet->opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);
        paramSet->opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

        /* Now, write the PaRAM Set. */
        result = EDMA3_DRV_setPaRAM(hEdma, chNum, paramSet);
    }

    if (result == EDMA3_DRV_SOK)
    {
        /* 初始化完成后手动传输一个字节数据，即temp中的数据 */
        result = EDMA3_DRV_enableTransfer(hEdma, chNum, EDMA3_DRV_TRIG_MODE_MANUAL);
    }

    return result;
}

/**
 * @brief  串口DMA启动输出数据
 * @param  uartPort 串口端口号
 * @param  buffer 需要传输的数据地址
 * @param  buffLength 需要传输的数据长度
 * @retval 成功开启传输返回EDMA3_DRV_SOK，否则返回错误码
 */
EDMA3_DRV_Result uart_edma_write(HAL_UART_PORT_e uartPort, volatile char *buffer, unsigned int buffLength)
{
    EDMA3_DRV_Result result = EDMA3_DRV_SOK;
    EDMA3_DRV_PaRAMRegs *paramSet = &uartHandle[uartPort].edma3Param;
    unsigned int chNum = 0;

    switch (uartPort)
    {
    case HAL_UART0:
        chNum = EDMA3_CHA_UART0_TX;
        break;

    case HAL_UART1:
        chNum = EDMA3_CHA_UART1_TX;
        break;

    case HAL_UART2:
        chNum = EDMA3_CHA_UART2_TX;
        break;

    default:
        break;
    }

    if (result == EDMA3_DRV_SOK)
    {
        /* Fill the PaRAM Set with transfer specific information */
        paramSet->srcAddr = (unsigned int)buffer;
        paramSet->bCnt = (unsigned short)buffLength;

        /* Now, write the PaRAM Set. */
        result = EDMA3_DRV_setPaRAM(hEdma, chNum, paramSet);
    }

    if (result == EDMA3_DRV_SOK)
    {
        /* 传输之前刷新缓存 */
        Cache_wbInv((void *)buffer, buffLength, Cache_Type_ALLD, TRUE);

        /* 开始传输数据 */
        result = EDMA3_DRV_enableTransfer(hEdma, chNum, EDMA3_DRV_TRIG_MODE_EVENT);
    }

    return result;
}

/**
 * @brief  串口初始化，初始化串口相关的信号量和缓冲区
 * @param  uartPort 串口号
 * @retval 初始化成功返回0，否则返回-1
 */
int8_t hal_uart_init(HAL_UART_PORT_e uartPort)
{
    switch (uartPort)
    {
    case HAL_UART0:
        uartHandle[uartPort].uartBaseAdd = SOC_UART_0_REGS;
        break;

    case HAL_UART1:
        uartHandle[uartPort].uartBaseAdd = SOC_UART_1_REGS;
        break;

    case HAL_UART2:
        uartHandle[uartPort].uartBaseAdd = SOC_UART_2_REGS;
        break;

    default:
        return -1;
    }

    /* 创建信号量 */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    uartHandle[uartPort].uartRxSem = Semaphore_create(1, &semParams, NULL);
    uartHandle[uartPort].uartTxEdmaSem = Semaphore_create(1, &semParams, NULL);
    if ((uartHandle[uartPort].uartRxSem == NULL)
     || (uartHandle[uartPort].uartTxEdmaSem == NULL))
        return -1;

    /* 数据收发缓冲区初始化 */
    circularBufferInit(&uartHandle[uartPort].uartTxBufferCtrl, uartHandle[uartPort].uartTxBuff, UART_TX_BUFF_SIZE);
    circularBufferInit(&uartHandle[uartPort].uartRxBufferCtrl, uartHandle[uartPort].uartRxBuff, UART_RX_BUFF_SIZE);

    /* 串口硬件初始化 */
    if (uartInit(uartPort) != 0)
        return -1;

    /* 串口DMA初始化 */
    if (uart_edma_init(uartPort) != EDMA3_DRV_SOK)
        return -1;

    return 0;
}

/**
 * @brief  串口消息发送，如果上次发送完成将直接把数据放入DMA缓冲区开始发送，如果上此发送未完成将阻塞等待
 *         上一次DMA发送完成
 * @param  uartPort 串口号
 * @param  data 需要传输数据存放的地址
 * @param  len 需要读取的数据长度，读取后返回已读取的数据长度
 * @retval NULL
 */
void hal_uart_write(HAL_UART_PORT_e uartPort, unsigned char *data, unsigned int len)
{
    if (len == 0 || data == NULL || uartPort >= HAL_UART_MAX)
    {
        return;
    }

    UART_HANDLE_s *instHandle = &uartHandle[uartPort];

    /* 等待之前DMA数据发送完成 */
    Semaphore_pend(instHandle->uartTxEdmaSem, BIOS_WAIT_FOREVER);

    /* 开始DMA数据发送 */
    uart_edma_write(uartPort, (volatile char *)data, len);
}

/**
 * @brief  串口消息读取，接收缓冲区中有数据时将直接返回缓冲区中数据，缓冲区中无数据时，等待数据接收，接
 *         收完成将数据返回
 * @param  uartPort 串口号
 * @param  dataAdder 返回数据存放缓冲区中的地址
 * @param  len 需要读取的数据长度，读取后返回已读取的数据长度
 * @param  timeout 超时时间，在等待时间内未接收到数据直接返回
 * @retval 读取成功将返回数据0，否则返回-1
 */
int hal_uart_read(HAL_UART_PORT_e uartPort, unsigned int *dataAdder, unsigned int *len, uint32_t timeout)
{
    UART_HANDLE_s *instHandle;
    uint32_t length;
    instHandle = &uartHandle[uartPort];

    if (!isCircularBufferEmpty(&instHandle->uartRxBufferCtrl))
    {
        length = circularBufferRead(&instHandle->uartRxBufferCtrl, dataAdder, *len);
        if (length > 0)
        {
            *len = length;
            return 0;
        }
    }
    else
    {
        if (Semaphore_pend(instHandle->uartRxSem, timeout))
        {
            length = circularBufferRead(&instHandle->uartRxBufferCtrl, dataAdder, *len);
            if (length > 0)
            {
                *len = length;
                return 0;
            }
        }
    }

    return -1;
}
