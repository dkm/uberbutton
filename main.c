/*
 * Copyright (C) 2014 Hamburg University of Applied Sciences
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for nrf24l01p lowlevel functions
 *
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 *
 * @}
 */

#ifndef SPI_PORT
#error "SPI_PORT not defined"
#endif
#ifndef CE_PIN
#error "CE_PIN not defined"
#endif
#ifndef CS_PIN
#error "CS_PIN not defined"
#endif
#ifndef IRQ_PIN
#error "IRQ_PIN not defined"
#endif

#include "periph/spi.h"

#ifdef ENABLE_NRF_COMM
#include "nrf24l01p_settings.h"
#include "nrf24l01p.h"
#include "nrf24l01p_settings.h"
#endif

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <board.h>
#include <time.h>
#include <string.h>
#include "periph/gpio.h"
#include "xtimer.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "msg.h"


#include "cpu.h"
#include <periph/adc.h>
#include "periph_conf.h"

#include "lcd1602d.h"
#include "uberframe.h"
//#include "uberWrap.h"

#define TEST_RX_MSG                1

#ifdef ENABLE_NRF_COMM
static int cmd_uber_setup(int argc, char **argv);
static int cmd_send(int argc, char **argv);
static int cmd_get_status(int argc, char **argv);
static int cmd_get_config(int argc, char **argv);
static int cmd_print_regs(int argc, char **argv);
static int cmd_its(int argc, char **argv);
static int cmd_set_tx_addr(int argc, char **argv);
static int cmd_set_channel(int argc, char **argv);
static int cmd_set_aa(int argc, char **argv);
static int cmd_get_rf_setup(int argc, char **argv);
static int cmd_set_dpl(int argc, char **argv);
#endif

void printbin(unsigned byte);
void print_register(char reg, int num_bytes);

static gpio_t led = GPIO_PIN(PORT_F, 1);


static unsigned int display_pid = KERNEL_PID_UNDEF;

#ifdef ENABLE_NRF_COMM
static nrf24l01p_t nrf24l01p_0;
static unsigned int sender_pid = KERNEL_PID_UNDEF;
#endif

struct lcd_ctx lcd = {
  .rs_pin = GPIO_PIN(PORT_E, 5),
  .enable_pin = GPIO_PIN(PORT_E,4),
  .data_pins = {GPIO_PIN(PORT_D,0),GPIO_PIN(PORT_D,1),GPIO_PIN(PORT_D,2),GPIO_PIN(PORT_D,3)},
  .displayfunctions = (LCD_4BITMODE | LCD_1LINE | LCD_2LINE | LCD_5x8DOTS),
  .numlines = 2,
};

/**
 * define some additional shell commands
 */
static const shell_command_t shell_commands[] = {
#ifdef ENABLE_NRF_COMM
    {"setaa", "set auto ack", cmd_set_aa },
    {"setchannel", "set channel", cmd_set_channel },
    {"status", "get status value", cmd_get_status },
    {"setdpl", "set dyn payload", cmd_set_dpl },
    {"rfsetup", "get rf setup", cmd_get_rf_setup },
    {"config", "get config value", cmd_get_config },
    { "settxaddr", "set tx addr", cmd_set_tx_addr },
    { "prgs", "print registers", cmd_print_regs },
    { "it", "init transceiver", cmd_its },
    { "send", "send 32 bytes data", cmd_send },
    { "ubersetup", "uber setup", cmd_uber_setup},
#endif
    { NULL, NULL, NULL }
};

// ROTARY
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#define R_START 0x0

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1    
#define R_CW_BEGIN 0x2     
#define R_START_M 0x3      
#define R_CW_BEGIN_M 0x4    
#define R_CCW_BEGIN_M 0x5
const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

/* #define R_START 0x0 */
/* #define R_CCW_BEGIN 0x1 */
/* #define R_CW_BEGIN 0x2 */
/* #define R_START_M 0x3 */
/* #define R_CW_BEGIN_M 0x4 */
/* #define R_CCW_BEGIN_M 0x5 */
/* const unsigned char ttable[6][4] = { */
/*   // R_START (00) */
/*   {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START}, */
/*   // R_CCW_BEGIN */
/*   {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START}, */
/*   // R_CW_BEGIN */
/*   {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START}, */
/*   // R_START_M (11) */
/*   {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START}, */
/*   // R_CW_BEGIN_M */
/*   {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW}, */
/*   // R_CCW_BEGIN_M */
/*   {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW}, */
/* }; */

unsigned int state = R_START;

/* gpio_t b1 = GPIO_PIN(PORT_C, 7); */
/* gpio_t b2 = GPIO_PIN(PORT_D, 6); */

/* void rotary_cb(void *unused __attribute__((unused))) { */
/*   unsigned int b1_v = gpio_read(b1) ? 1 : 0; */
/*   unsigned int b2_v = gpio_read(b2) ? 1 : 0; */
  
/*   unsigned char pinstate = ( b1_v? 2 : 0) | (b2_v ? 1 : 0); */
/*   /\* history[history_cnt] = pinstate; *\/ */
/*   /\* history_cnt++; *\/ */
  
/*   /\* printf("state %d pinstate : %x,  b1 %d b2 %d\n", state, pinstate, b1_v, b2_v); *\/ */
/*   state = ttable[state & 0xf][pinstate]; */

/*   switch(state & 0x30){ */
/*   case DIR_CCW: */
/*     printf("CCW\n"); */
/*     break; */
/*   case DIR_CW: */
/*     printf("CW\n"); */
/*     break; */
/*   case DIR_NONE: */
/*     //    printf("noevent found\n"); */
/*     break; */
/*   default: */
/*     // printf("none of the above ?! %d\n", state); */
/*     break; */
/*   } */
/*   printf("boom\n"); */
/* } */

//END ROTARY

void prtbin(unsigned byte)
{
    for (char i = 0; i < 8; i++) {
        printf("%u", (byte >> (7 - i)) & 0x0001);
    }

    puts("\n");
}


#ifdef ENABLE_NRF_COMM
/**
 * @print register
 */
void print_register(char reg, int num_bytes)
{

    char buf_return[num_bytes];
    int ret;


    gpio_clear(CS_PIN);
    xtimer_usleep(1);
    ret = spi_transfer_regs(SPI_PORT, (CMD_R_REGISTER | (REGISTER_MASK & reg)), 0, buf_return, num_bytes);
    gpio_set(CS_PIN);

    if (ret < 0) {
        printf("Error in read access\n");
    }
    else {
        if (num_bytes < 2) {
            printf("0x%x returned: ", reg);

            for (int i = 0; i < num_bytes; i++) {
	      printf(" (%x) ", buf_return[i]);
                prtbin(buf_return[i]);
            }
        }
        else {
            printf("0x%x returned: ", reg);

            for (int i = 0; i < num_bytes; i++) {
                printf("%x ", buf_return[i]);
            }

            printf("\n\n");
        }
    }
}
#endif

char display_handler_stack[THREAD_STACKSIZE_MAIN];

static char dest_str[256] = {0};
static int dest = 0;

#ifdef ENABLE_NRF_COMM
char tx_handler_stack[THREAD_STACKSIZE_MAIN];

/* RX handler that waits for a message from the ISR */
void *nrf24l01p_tx_thread(void *arg){
    msg_t msg_q[1];
    msg_init_queue(msg_q, 1);
    sender_pid = thread_getpid();

    msg_t m;

    while (msg_receive(&m)) {
      printf("nrf24l01p_tx got a message\n");

      //      lcd1602d_printstr(&lcd, 0, 1, dest_str);
      lcd1602d_printstr(&lcd, 10, 1, "SEND...");
      cmd_send(4, (char**)m.content.ptr);
      lcd1602d_printstr(&lcd, 10, 1, "IDLE   ");
    }
    return NULL;
}
#endif

void *display_thread(void *arg){
    msg_t msg_q[1];
    msg_init_queue(msg_q, 1);

    display_pid = thread_getpid();

    msg_t m;

    while (msg_receive(&m)) {
      printf("display_thread got a message\n");
      const char *name = uber_get_name(dest);
      lcd1602d_printstr(&lcd, 0, 1, name);
      int i;
      for (i=strlen(name); i<10; i++){
	lcd1602d_printstr(&lcd, i, 1, " ");
      }
    }

    return NULL;
}


#ifdef ENABLE_NRF_COMM
char rx_handler_stack[THREAD_STACKSIZE_MAIN];

/* RX handler that waits for a message from the ISR */
void *nrf24l01p_rx_handler(void *arg)
{
    msg_t msg_q[1];
    msg_init_queue(msg_q, 1);
    unsigned int pid = thread_getpid();
    //    char rx_buf[NRF24L01P_MAX_DATA_LENGTH];
    radiohead_frame_t frame;

    char *rx_buf = (char*)&frame;

    puts("Registering nrf24l01p_rx_handler thread...");
    nrf24l01p_register(&nrf24l01p_0, &pid);

    msg_t m;

    while (msg_receive(&m)) {
        printf("nrf24l01p_rx_handler got a message: ");

        switch (m.type) {
            case RCV_PKT_NRF24L01P:
                puts("Received packet.");

                /* CE low */
                nrf24l01p_stop((nrf24l01p_t *)m.content.ptr);

                /* read payload */
                nrf24l01p_read_payload((nrf24l01p_t *)m.content.ptr, rx_buf, NRF24L01P_MAX_DATA_LENGTH);

                /* flush rx fifo */
                nrf24l01p_flush_rx_fifo((nrf24l01p_t *)m.content.ptr);

                /* CE high */
                nrf24l01p_start((nrf24l01p_t *)m.content.ptr);

                /* print rx buffer */
                /* for (int i = 0; i < NRF24L01P_MAX_DATA_LENGTH; i++) { */
                /*     printf("%i ", rx_buf[i]); */
                /* } */

                /* puts(""); */
		uber_dump_frame(&frame);
		char buf[256] = {0};
		uber_get_frame(&frame, buf);
		printf("lcd rx:%p\n", &lcd);
		lcd1602d_printstr(&lcd, 0, 0, buf);
                break;

            default:
                puts("stray message.");
                break;
        }
    }

    puts("nrf24l01p_rx_handler: this should not have happened!");

    return NULL;
}


/**
 * @init transceiver
 */
int cmd_its(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    puts("Init Transceiver\n");

    /* initialize transceiver device */
    if (nrf24l01p_init(&nrf24l01p_0, SPI_PORT, CE_PIN, CS_PIN, IRQ_PIN) < 0) {
        puts("Error in nrf24l01p_init");
        return 1;
    }

    if (nrf24l01p_set_datarate(&nrf24l01p_0, NRF24L01P_DR_2MBS) < 0){
        puts("Error in nrf24l01p_set_datarate");
        return 1;
    }
    

    /* setup device as receiver */
    if (nrf24l01p_set_rxmode(&nrf24l01p_0) < 0) {
        puts("Error in nrf24l01p_set_rxmode");
        return 1;
    }

    return cmd_print_regs(0, 0);
}


int cmd_get_config(int argc, char **argv)
{
    unsigned char buf_return;
    int ret;

    gpio_clear(CS_PIN);
    xtimer_usleep(1);
    ret = spi_transfer_regs(SPI_PORT, (CMD_R_REGISTER | (REGISTER_MASK & REG_CONFIG)), 0, (char*)&buf_return, 1);
    gpio_set(CS_PIN);
    if (ret < 0) {
        printf("Error in read access\n");
	return -1;
    }

    printf("Config : %x\n", buf_return);
    printf(" prim_rx : %d (%s)\n", buf_return & PRIM_RX, (buf_return & PRIM_RX)? "PRX" : "PTX" );
    printf(" pwr_up : %d (%s)\n", buf_return & PWR_UP, (buf_return & PWR_UP)? "UP" : "DOWN");
    printf(" crco : %d (%s)\n", buf_return & CRCO, (buf_return & CRCO)? "2 bytes" : "1 byte");
    printf(" en_crc : %d (%s)\n", buf_return & EN_CRC, (buf_return & EN_CRC)? "enabled" : "disabled");
    printf(" mask_max_rt : %d (%s)\n", buf_return & MASK_MAX_RT, (buf_return & MASK_MAX_RT)? "not IRQ" : "on IRQ");
    printf(" mask_tx_ds : %d (%s)\n", buf_return & MASK_TX_DS, (buf_return & MASK_TX_DS)? "not IRQ" : "on IRQ");
    printf(" mask_rx_dr : %d (%s)\n", buf_return & MASK_RX_DR, (buf_return & MASK_RX_DR)? "not IRQ" : "on IRQ");
    return 0;
}

int cmd_get_status(int argc, char **argv)
{
  int status = nrf24l01p_get_status(&nrf24l01p_0);
    /* get status of the transceiver */

    printf("Status : %x\n", status);
    printf(" rx_dr : %d\n", status & RX_DR);
    printf(" tx_ds : %d\n", status & TX_DS);
    printf(" max_rt : %d\n", status & MAX_RT);
    printf(" rx_p_no : %x (%s)\n", (status & RX_P_NO)>>1,
	   (status & RX_P_NO) == 0x7<<1 ? "EMPTY" :
	   (status & RX_P_NO) == 0x6<<1 ? "INVALID" :
	   "PIPE NUM");
    printf(" tx_full : %d\n", status & TX_FULL);
    return 0;
}


int cmd_uber_setup(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    int pipe;
    puts("Uber Setup !\n");
    printf("CE : pin %d: port %d\n", (unsigned int)(CE_PIN&0x0f), (unsigned int)(CE_PIN>>4));
    printf("CS : pin %d: port %d\n", (unsigned int)(CS_PIN&0x0f), (unsigned int)(CS_PIN>>4));
    printf("IRQ : pin %d: port %d\n", (unsigned int)(IRQ_PIN&0x0f), (unsigned int)(IRQ_PIN>>4));

    /* initialize transceiver device */
    if (nrf24l01p_init(&nrf24l01p_0, SPI_PORT, CE_PIN, CS_PIN, IRQ_PIN) < 0) {
        puts("Error in nrf24l01p_init");
        return 1;
    }
    
    if (nrf24l01p_set_datarate(&nrf24l01p_0, NRF24L01P_DR_2MBS) < 0){
        puts("Error in nrf24l01p_set_datarate");
        return 1;
    }

    if (nrf24l01p_enable_crc(&nrf24l01p_0, NRF24L01P_CRC_2BYTE) < 0){
        puts("Error in nrf24l01p_enable_crc");
        return 1;
    }

    if (nrf24l01p_set_channel(&nrf24l01p_0, 1) < 0){
      puts("Error setting channel\n");
      return 1;
    }
    for (pipe=0; pipe<6; pipe++){
      if(nrf24l01p_setup_auto_ack(&nrf24l01p_0, pipe, NRF24L01P_RETR_250US, 15) < 0){
	printf("Error in nrf24l01p_setup_auto_ack on pipe %d\n", pipe);
	return 1;
      }
      if(nrf24l01p_enable_dynamic_payload(&nrf24l01p_0, pipe)<0){
	printf("Error in nrf24l01p_enable_dynamic_payload on pipe %d\n", pipe);
	return 1;
      }
    }
    if(nrf24l01p_enable_dynamic_ack(&nrf24l01p_0)<0){
      printf("Error in nrf24l01p_enable_dynamic_ack\n");
      return 1;
    }

    /* create thread that gets msg when data arrives */
    if (thread_create(
        rx_handler_stack, sizeof(rx_handler_stack), THREAD_PRIORITY_MAIN - 1, 0,
        nrf24l01p_rx_handler, 0, "nrf24l01p_rx_handler") < 0) {
        puts("Error in thread_create");
        return 1;
    }

    /* create thread that send msg */
    if (thread_create(
        tx_handler_stack, sizeof(tx_handler_stack), THREAD_PRIORITY_MAIN - 1, 0,
        nrf24l01p_tx_thread, 0, "nrf24l01p_tx_thread") < 0) {
        puts("Error in thread_create");
        return 1;
    }

    /* create thread that display msg */
    if (thread_create(
        display_handler_stack, sizeof(display_handler_stack), THREAD_PRIORITY_MAIN - 1, 0,
        display_thread, 0, "display_thread") < 0) {
        puts("Error in thread_create");
        return 1;
    }




    /* setup device as receiver */
    if (nrf24l01p_set_rxmode(&nrf24l01p_0) < 0) {
        puts("Error in nrf24l01p_set_rxmode");
        return 1;
    }

    return 0; //cmd_print_regs(0, 0);
}



/**
 * @set TX mode
 */
int cmd_send(int argc, char **argv)
{
    gpio_set(led);
    puts("Send");

    int status = 0;

    radiohead_frame_t frame = {0};
    //    unsigned int flen = sizeof(frame);
    frame.radiohead_src = 0xff;
    frame.radiohead_dst = 0xff;

    frame.frame.header.src = atoi(argv[1]);
    frame.frame.header.dst = atoi(argv[2]);
    frame.frame.header.type = atoi(argv[3]);
    frame.frame.header.payload_sz = 0;
    uber_dump_frame(&frame);
    
    //    char tx_buf[NRF24L01P_MAX_DATA_LENGTH];
    char *tx_buf = (char*) &frame;
    unsigned int i, flen = uber_get_frame_size(&frame);
    printf ("Sending: ");
    for (i=0; i<flen; i++){
      printf("%x.",tx_buf[i]);
    }
    printf("\n");
    printf(" -> %d bytes\n", flen);
    /* char *tx_buf = (char*) f; */
    /* fill TX buffer with numbers 32..1 */
    /* for (int i = 0; i < sizeof(tx_buf); i++) { */
    /*     tx_buf[i] = NRF24L01P_MAX_DATA_LENGTH - i; */
    /* } */
    /* power on the device */
    if (nrf24l01p_on(&nrf24l01p_0) < 0) {
        puts("Error in nrf24l01p_on");
        return 1;
    }
    /* setup device as transmitter */
    if (nrf24l01p_set_txmode(&nrf24l01p_0) < 0) {
        puts("Error in nrf24l01p_set_txmode");
        return 1;
    }
    /* load data to transmit into device */
    if (nrf24l01p_preload(&nrf24l01p_0, tx_buf, flen /* NRF24L01P_MAX_DATA_LENGTH */) < 0) {
        puts("Error in nrf24l01p_preload");
        return 1;
    }

    /* trigger transmitting */
    nrf24l01p_transmit(&nrf24l01p_0);
    /* wait while data is pysically transmitted  */
    xtimer_usleep(DELAY_DATA_ON_AIR*10);
    /* get status of the transceiver */
    status = nrf24l01p_get_status(&nrf24l01p_0);
    if (status < 0) {
        puts("Error in nrf24l01p_get_status");
    }
    if (status & TX_DS) {
        status = nrf24l01p_reset_interrupts(&nrf24l01p_0, MASK_TX_DS);
        printf("Sent Packet %x\n", MASK_TX_DS);
    } else {
      printf ("Status is %x\n", status);
    }
    /* setup device as receiver */
    if (nrf24l01p_set_rxmode(&nrf24l01p_0) < 0) {
        puts("Error in nrf24l01p_set_rxmode");
        return 1;
    }
    gpio_clear(led);
    return 0;
}

int cmd_set_channel(int argc, char **argv){
  int channel = atoi(argv[1]);
  int ret = nrf24l01p_set_channel(&nrf24l01p_0, channel);
  if (ret < 0){
    printf("Error setting channel\n");
  }
  
  return 0;
}

int cmd_set_aa(int argc, char **argv){
  int pipe = atoi(argv[1]);
  int ret = nrf24l01p_setup_auto_ack(&nrf24l01p_0, pipe, NRF24L01P_RETR_250US, 15);
  if (ret < 0){
    printf("Error setting auto ack on pipe %d\n", pipe);
    return -1;
  }
  return 0;
}

int cmd_set_dpl(int argc, char **argv){
  int pipe = atoi(argv[1]);
  int ret = nrf24l01p_enable_dynamic_payload(&nrf24l01p_0, pipe);
  if (ret < 0){
    printf("Error enabling DPL on pipe %d\n", pipe);
    return -1;
  }
  return 0;
}

int cmd_get_rf_setup(int argc, char **argv){
  unsigned char reg_val;
  unsigned char reg_rf_ch;
  int ret = nrf24l01p_read_reg(&nrf24l01p_0, REG_RF_SETUP, (char*)&reg_val);
  if (ret < 0){
    printf("Error getting RF_SETUP\n");
  }
  ret = nrf24l01p_read_reg(&nrf24l01p_0, REG_RF_CH, (char*)&reg_rf_ch);
  if (ret < 0){
    printf("Error getting RF_CH\n");
  }
  printf("Rf channel : %x\n", reg_rf_ch);
  printf("rf_setup : %x\n", reg_val);
  printf(" rf_pwr : %x ", reg_val & RF_SETUP_RF_PWR);
  switch((reg_val & RF_SETUP_RF_PWR)>>1){
  case 0:
    printf("(-18dBm)\n");
    break;
  case 1:
    printf("(-12dBm)\n");
    break;
  case 2:
    printf("(-6dBm)\n");
    break;
  case 3:
    printf("(0dBm)\n");
    break;
  default:
    printf("invalid\n");
  }

  printf(" rf_dr_high : %x\n", reg_val & RF_SETUP_RF_DR_HIGH);
  printf(" pll_lock : %x\n", reg_val & RF_SETUP_PLL_LOCK);
  printf(" rf_dr_low : %x\n", reg_val & RF_SETUP_RF_DR_LOW);

  printf("dr_low/dr_high : ");
  switch(reg_val & (RF_SETUP_RF_DR_LOW | RF_SETUP_RF_DR_HIGH)){
  case 0x0:
    printf("1mbps\n");
    break;
  case 0x1<<3:
    printf("2mbps\n");
    break;
  case 0x100<<3:
    printf("250kbps\n");
    break;
  case 0x101<<3:
    printf("RESERVED\n");
    break;
  default:
    printf("invalid\n");
  }

  printf(" cont_wave : %x\n", reg_val & RF_SETUP_CONT_WAVE);

  return 0;
}

int cmd_set_tx_addr(int argc, char **argv){
    char new_tx_addr[] =  {0xde, 0xad, 0xbe, 0xef, 0x00,};

    if (argc == 6) {
      int i;
      printf ("Got:");
      for (i=1; i < 6; i++){
	new_tx_addr[i-1] = strtol(argv[i], NULL, 0);
	printf("%x ", new_tx_addr[i-1]);
      }
      puts("\n");
    }
    nrf24l01p_set_tx_address(&nrf24l01p_0, new_tx_addr, INITIAL_ADDRESS_WIDTH);
    return 0;
}

/**
 * @print registers
 */
int cmd_print_regs(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    printf("################## Print Registers ###################\n");


    puts("REG_CONFIG: ");
    print_register(REG_CONFIG, 1);

    puts("REG_EN_AA: ");
    print_register(REG_EN_AA, 1);

    puts("REG_EN_RXADDR: ");
    print_register(REG_EN_RXADDR, 1);

    puts("REG_SETUP_AW: ");
    print_register(REG_SETUP_AW, 1);

    puts("REG_SETUP_RETR: ");
    print_register(REG_SETUP_RETR, 1);

    puts("REG_RF_CH: ");
    print_register(REG_RF_CH, 1);

    puts("REG_RF_SETUP: ");
    print_register(REG_RF_SETUP, 1);

    puts("REG_STATUS: ");
    print_register(REG_STATUS, 1);

    puts("REG_OBSERVE_TX: ");
    print_register(REG_OBSERVE_TX, 1);

    puts("REG_RPD: ");
    print_register(REG_RPD, 1);

    puts("REG_RX_ADDR_P0: ");
    print_register(REG_RX_ADDR_P0, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_ADDR_P1: ");
    print_register(REG_RX_ADDR_P1, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_ADDR_P2: ");
    print_register(REG_RX_ADDR_P2, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_ADDR_P3: ");
    print_register(REG_RX_ADDR_P3, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_ADDR_P4: ");
    print_register(REG_RX_ADDR_P4, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_ADDR_P5: ");
    print_register(REG_RX_ADDR_P5, INITIAL_ADDRESS_WIDTH);

    puts("REG_TX_ADDR: ");
    print_register(REG_TX_ADDR, INITIAL_ADDRESS_WIDTH);

    puts("REG_RX_PW_P0: ");
    print_register(REG_RX_PW_P0, 1);

    puts("REG_FIFO_STATUS: ");
    print_register(REG_FIFO_STATUS, 1);

    puts("REG_DYNPD: ");
    print_register(REG_DYNPD, 1);

    puts("REG_FEATURE: ");
    print_register(REG_FEATURE, 1);

    return 0;
}
#endif


void test_cb(void * bid){
  int id = *(int*)bid;
  printf ("in cb %d\n", id);
  //  static char dest_str[10] = {0};
  static char *argv[] = {"", "4", dest_str , "1"};
  (void)argv;
  switch (id){
  case 1:
    dest = (dest +1 ) % 9;
    if (!dest) {
      // skip 0, it is invalid
      dest = 1;
    }
    printf("dest %d\n", dest != 8 ? dest : 0xFF);
    printf("lcd cb:%p\n", &lcd);
    //    lcd1602d_printstr(&lcd, 0, 1, dest_str);
    if (display_pid != KERNEL_PID_UNDEF) {
      msg_t m;
      m.type = 0;
      m.content.ptr = NULL;
      msg_send_int(&m, display_pid);
    }

    return;
    
    break;
  case 2:
    break;
  default:
    return;
  }

  sprintf(dest_str, "%d", dest != 8 ? dest : 0xFF);
  dest_str[9] = 0;

#ifdef SPI_NUMOF
  if (sender_pid != KERNEL_PID_UNDEF) {
    msg_t m;
    m.type = RCV_PKT_NRF24L01P;
    m.content.ptr = (char *)argv;
    /* transmit more things here ? */
    msg_send_int(&m, sender_pid);
  }
#endif
  
  printf("exit cb\n");
  //cmd_send(4, argv);
}

#ifdef ADC_NUMOF
static int res_ladder_val(adc_t adc, int channel){
  int sample = adc_sample(adc, channel);
  const int max_v = 4095;
  int j;
  int but_state = 0;

  //  printf("%d\n", sample);
  
  for (j=1; j<=4; j++){
    //      printf("[%d-%d > %d?", sample, max_v/(1<<j), -(max_v/(1<<(j+2))));
    if (sample - max_v/(1<<j) > -max_v/(32) /* -(max_v/(1<<(j+2))) */ /* -4095/(j*(j+1)) */){
	//	printf("%d Test ok /", j);
    	sample -= max_v/(1<<j);
    	but_state |= 1<<(j-1);
      }
  }
  return but_state;
}
#endif

int main(void)
{
  puts("Uber\n");

#ifdef ADC_NUMOF
  adc_init(ADC_0, 10);
#endif
  
  lcd1602d_init_lcd(&lcd);
  //  lcd1602d_setCursor(&lcd, 0,0);
  printf("lcd:%p\n", &lcd);
  /* lcd1602d_printstr(&lcd, 0,0,"Bojr"); */
  /* lcd1602d_printstr(&lcd, 3,1,"Bloop"); */

  gpio_t b1 = GPIO_PIN(PORT_F, 4);
  int b1_v = 1, b2_v = 2;
  gpio_init_int(b1, GPIO_PULLUP, GPIO_FALLING, test_cb, &b1_v);
  gpio_t b2 = GPIO_PIN(PORT_F, 0);
  gpio_init_int(b2, GPIO_PULLUP, GPIO_FALLING, test_cb, &b2_v);

  gpio_init(led, GPIO_DIR_OUT, GPIO_NOPULL);
  gpio_clear(led);

  //char line_buf[SHELL_DEFAULT_BUFSIZE];
#ifdef SPI_NUMOF
  cmd_uber_setup(0, NULL);
#endif
    
  //shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

  /* gpio_init_int(b1, GPIO_PULLUP, GPIO_BOTH, rotary_cb, NULL); */
  /* gpio_init(b2, GPIO_DIR_IN, GPIO_PULLUP); */

#ifdef ADC_NUMOF
  int button_state = res_ladder_val(ADC_0, 2);
#endif
    
  while(1){
#ifdef ADC_NUMOF
    int new_button_state = res_ladder_val(ADC_0, 2);

    if (new_button_state != button_state){
      button_state = new_button_state;
      printf("%d-%d-%d-%d\n", new_button_state & 0x8 ? 1 : 0,
	     new_button_state & 0x4 ? 1 : 0,
	     new_button_state & 0x2 ? 1 : 0,
	     new_button_state & 0x1 ? 1 : 0);

      dest = new_button_state;
      if (display_pid != KERNEL_PID_UNDEF) {
	msg_t m;
	m.type = 0;
	m.content.ptr = NULL;
	msg_send_int(&m, display_pid);
      }
    }
#endif
    xtimer_usleep(10000);
    thread_yield(); 
  }
  return 0;
}
