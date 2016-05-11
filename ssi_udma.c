/*
 * heavily based on njneerW/WS2812_drv project
 */

#include "periph/spi.h"

#include "ssi_udma.h"
#include "thread.h"
#include "msg.h"
#include "ws2812.h"

#include <inttypes.h>

static ssi_udma_t *g_dev = NULL;

/* static uint8_t *g_pui8DoneVar = NULL; */
/* static uint8_t *g_pui8SPIArray; */
/* static uint16_t g_ui16SPIArraySize; */
static unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));

void ssi_udma_restart_tx(ssi_udma_t *dev){
            ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                                          UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                                          UDMA_DST_INC_NONE | UDMA_ARB_8);
            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, dev->data,
                                   (void *)(SSI1_BASE + SSI_O_DR),
                                   dev->data_len);
            if(dev->done != NULL)
            {
                *dev->done = 1;
            }
	    ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
}

void isr_ssi1(void){
    unsigned long ulStatus;
    static unsigned char ucPing = 0;
    static unsigned char ucZero = 0;

    //
    // Read the interrupt status of the UART.
    //
    ulStatus = ROM_SSIIntStatus(SSI1_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    ROM_SSIIntClear(SSI1_BASE, ulStatus);

    //
    // If the UART0 DMA TX channel is disabled, that means the TX DMA transfer
    // is done.
    //
    if(!ROM_uDMAChannelIsEnabled(UDMA_CHANNEL_SSI1TX))
    {

        //
        // Start another DMA transfer to UART0 TX.
        //
        if(ucPing)
        {
            ucPing = 0;
	    if (g_dev->listener != KERNEL_PID_UNDEF){
	      msg_t m;
	      m.type = SSI_UDMA_FINISHED;
	      msg_send_int(&m, g_dev->listener);
	    }
	    return;
        }
        else
        {
            //
            // Send out enough zero's to inform the LEDs that the previous
            // message is complete.  uDMA is a bit overkill for this... meh.
            //
            ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
                                          UDMA_DST_INC_NONE | UDMA_ARB_8);
            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                                               UDMA_MODE_BASIC, &ucZero,
                                               (void *)(SSI1_BASE + SSI_O_DR),
                                               2);
            ucPing = 1;
        }
        //
        // The uDMA TX channel must be re-enabled.
        //
        ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
    }
}

void isr_udma_error(void)
{
    unsigned long ulStatus;

    //
    // Check for uDMA error bit
    //
    ulStatus = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and continue.  If we're
    // still debugging our project, we want to infinte loop here so we can
    // investiage the failure cause.
    //
    if(ulStatus)
    {
        ROM_uDMAErrorStatusClear();
        //while(1);
    }
}

void
ssi_udma_register(ssi_udma_t *dev, unsigned int pid){
  dev->listener = pid;
}

void
ssi_udma_init(ssi_udma_t *dev, uint8_t *pui8SPIData, uint16_t ui16DataSize,
	      uint8_t *pui8DoneVar, unsigned int pid)
{
    int i;
    dev->done = pui8DoneVar;
    dev->data = pui8SPIData;
    dev->data_len = ui16DataSize;
    if (pid > 0)
      dev->listener = pid;
    
    g_dev = dev;
    //
    // zero out SPI data array
    //
    for(i=0;i<ui16DataSize;i++)
    {
      // FIXME
      //WSArrayInit(pui8SPIData, ui16DataSize);
    }

    //
    // Enable the uDMA controller at the system level.  Enable it to continue
    // to run while the processor is in sleep.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    ROM_IntEnable(INT_UDMAERR);

    //
    // Enable the uDMA controller.
    //
    ROM_uDMAEnable();

    //
    // Point at the control table to use for channel control structures.
    //
    ROM_uDMAControlBaseSet(ucControlTable);

    //
    // Enable the SPI peripheral, and configure it to operate even if the CPU
    // is in sleep.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI1);

    //
    // Set PF1 to SSI1TX
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
    ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Configure the SPI communication parameters.  We could use 4 instead of
    // 8 here and be more memory efficient (as each spi bit is encoded into 4
    // bits on the SPI line), but the uDMA only allows for 8 bit increment of
    // source data
    //
    ROM_SSIConfigSetExpClk(SSI1_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                            SSI_MODE_MASTER, 2500000, 8);

    //
    // Enable the SSI for operation, and enable the uDMA interface for both the
    // TX channel
    //
    ROM_SSIEnable(SSI1_BASE);
    ROM_SSIDMAEnable(SSI1_BASE, SSI_DMA_TX);

    //
    // Enable the SSI peripheral interrupts.  Note that no SSI interrupts
    // were enabled, but the uDMA controller will cause an interrupt on the
    // SSI interrupt signal when a uDMA transfer is complete.  uDMA interrupt
    // handler is really only used if something goes horribly wrong.
    //
    ROM_IntEnable(INT_SSI1);

    //
    // Put the attributes in a known state for the uDMA SSI1TX channel.  These
    // should already be disabled by default.
    //
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1TX,
                                    UDMA_ATTR_ALTSELECT |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the SSI TX.  The uDMA SSI TX
    // channel is used to transfer a block of data from a buffer to the periph.
    // The data size is 8 bits.  The source address increment is 8-bit bytes
    // since the data is coming from a uint8_t buffer.  The destination
    // increment is none since the data is to be written to the SSI data
    // register.  The arbitration size is set to 4, which doesn't matter much
    // since we're only running one uDMA transfer.  If we were running multiple
    // transfers through the uDMA engine, we would want to increase this enough
    // to insure that the full LED strip worth of data goes out before the uDMA
    // engine services another channel.
    // TODO: see last sentence.
    //
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                              UDMA_ARB_8);

    //
    // Set up the transfer parameters for the uDMA SSI TX channel.  This will
    // configure the transfer source and destination and the transfer size.
    // Basic mode is used because the peripheral is making the uDMA transfer
    // request.  The source is the TX buffer and the destination is the SSI
    // data register.
    //
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, pui8SPIData,
                               (void *)(SSI1_BASE + SSI_O_DR),
                               ui16DataSize);

    *pui8DoneVar = 0;

    //
    // Now both the uDMA SSI TX and channel is primed to start a transfer.  As
    // soon as the channel is enabled, the peripheral will issue a transfer
    // request and the data transfers will begin.
    //
    ROM_IntMasterEnable();
    ROM_uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
}


