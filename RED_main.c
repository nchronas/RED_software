/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>
#define __MSP432P401R__
#include <ti/devices/msp432p4xx/driverlib/uart.h>
#include <ti/drivers/uart/UARTMSP432.h>


#include "PQ9_bus_engine.h"
#include "queue.h"
#include "subsystem.h"

#include "osal.h"

#include "RED_Board.h"

extern UART_Handle uart_pq9_bus;
extern UART_Handle uart_dbg_bus;

bool start_flag = false;


#define HLDLC_START_FLAG        0x7E
#define HLDLC_CONTROL_FLAG      0x7D
#define HLDLC_STOP_FLAG         0x7C

uint8_t tx_count, tx_size, tx_buf[255];
uint8_t rx_count, rx_size, rx_buf[255];
bool ctrl_flag, strt_flag = false;
uint8_t resp54[3];
uint8_t res;

uint8_t pq_rx_buf[300];
uint16_t pq_rx_count, pq_size;
bool pq_rx_flag;
//uint16_t pq_rx_addr_cnt = 0;
//uint16_t pq_rx_byte_cnt = 0;

void HLDLC_frame(uint8_t *buf_in, uint8_t *buf_out, uint16_t size_in, uint16_t *size_out) {

    uint16_t cnt = 2;

    for(uint16_t i = 0; i < size_in; i++) {
        if(i == 0) {
            buf_out[0] = HLDLC_START_FLAG;
            buf_out[1] = buf_in[0];
        } else if(i == (size_in) - 1) {
            if(buf_in[i] == HLDLC_START_FLAG) {
                buf_out[cnt++] = HLDLC_CONTROL_FLAG;
                buf_out[cnt++] = 0x5E;
            } else if(buf_in[i] == HLDLC_CONTROL_FLAG) {
                buf_out[cnt++] = HLDLC_CONTROL_FLAG;
                buf_out[cnt++] = 0x5D;
            } else { buf_out[cnt++] = buf_in[i]; }
            //buf_out[cnt++] = HLDLC_START_FLAG;
            *size_out = cnt;
            return ;
        } else if(buf_in[i] == HLDLC_START_FLAG) {
            buf_out[cnt++] = HLDLC_CONTROL_FLAG;
            buf_out[cnt++] = 0x5E;
        } else if(buf_in[i] == HLDLC_CONTROL_FLAG) {
            buf_out[cnt++] = HLDLC_CONTROL_FLAG;
            buf_out[cnt++] = 0x5D;
        } else {
            buf_out[cnt++] = buf_in[i];
        }

    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    Timer_init();
   // Watchdog_init();

    /*PQ9 services start*/
    pkt_pool_INIT();
    device_init();

    init_parameters();

    start_flag = true;

    /* Loop forever echoing */
    while (1) {

        update_device(OBC_MON_DEV_ID);
        usleep(1);

        update_device(OBC_TEMP_DEV_ID);
        usleep(1);

        usleep(100);


    }
}


uint8_t test_loop = 0;

/*  ======== ecssThread ========
 *  This thread runs on a higher priority, since wdg pin
 *  has to be ready for master.
 */
void *pqReceiveThread(void *arg0)
{

    while(!start_flag) {
        usleep(1000);
    }

    uint8_t *ts = &test_loop;

    /* Loop forever */
    while (1) {

          while(test_loop) {

          }

          bool res_uart = false;

          pq9_pkt *pkt_rx;
          uint16_t uart_size = 0;
          uint16_t pq_size = 0;
          char res_rx = 0;

          //res_uart = HAL_uart_rx(0, &ud.uart_rx_buf, &uart_size);

          do {
                res_rx = UART_read(uart_dbg_bus, resp54, 1);
                if(res_rx > 0) {
                  if(resp54[0] == HLDLC_START_FLAG) {
                    strt_flag = true;
                  } else if(resp54[0] == HLDLC_CONTROL_FLAG) {
                    ctrl_flag = true;
                  } else if(strt_flag) {
                     strt_flag = false;
                     tx_buf[0] = resp54[0];
                     tx_count = 1;
                  } else if(ctrl_flag) {
                     ctrl_flag = false;
                     if(resp54[0] == 0x5D) {
                       tx_buf[tx_count] = 0x7D;
                       tx_count++;
                     } else if(resp54[0] == 0x5E) {
                       tx_buf[tx_count] = 0x7E;
                       tx_count++;
                     }
                  } else if(tx_count == 1) {
                    tx_buf[tx_count] = resp54[0];
                    tx_size = resp54[0] + 5;
                    tx_count++;
                  } else if(tx_count > 0 && tx_count < tx_size - 1) {
                    tx_buf[tx_count] = resp54[0];
                    tx_count++;
                  } else if(tx_count > 0 && tx_count == tx_size - 1) {
                    tx_buf[tx_count] = resp54[0];
                    tx_count++;
                    if(tx_buf[0] < 127) {
                      res_uart = true;
                    }
                  }
                }
              } while(res_rx > 0);

          if(res_uart == true) {
            res_uart == false;
            //tx_buf, tx_count

            pkt_rx = get_pkt(pq_size);
            if(!C_ASSERT(pkt_rx != NULL) == true) {
                continue ;
            }

            bool res_unpack_PQ = unpack_PQ9_BUS(tx_buf,
                                                tx_count,
                                                pkt_rx);
            if(pkt_rx->dest_id != SYSTEM_APP_ID) {
              free_pkt(pkt_rx);
              continue ;
            }

            if(res_unpack_PQ == true) {

              route_pkt(pkt_rx);
            } else {
              free_pkt(pkt_rx);
              continue ;
            }

            free_pkt(pkt_rx);

            //transmit();
            {

              pq9_pkt *pkt_tx = 0;
              uint16_t size = 0;

              if((pkt_tx = queuePop(RS_POOL_ID)) ==  NULL) {
                  continue ;
              }


              bool res = pack_PQ9_BUS(pkt_tx, rx_buf, &size);
              if(res == false) {
                free_pkt(pkt_tx);
                continue ;
              }

              if(!C_ASSERT(size > 0) == true) {
                free_pkt(pkt_tx);
                continue ;
              }

              //update_pstats_tx_counter(pkt_tx->dest_id);
              //HAL_uart_tx(0, &ud.uart_tx_buf, size);
              //Add hldlc
              //add tx
              uint8_t buf_rs[255];
              uint16_t buf_cnt;

              HLDLC_frame(rx_buf, buf_rs, size, &buf_cnt);

              UART_write(uart_dbg_bus, buf_rs, buf_cnt);
              buf_cnt = 0;

              free_pkt(pkt_tx);
            }

          }
         usleep(1);
    }

    return (NULL);
}

void *pqTransmitThread(void *arg0)
{

    while(!start_flag) {
        usleep(1000);
    }

    /* Loop forever */
    while (1) {
         //export_pkt();
         sleep(100);
    }

    return (NULL);
}

char buf_rs[100];
uint16_t buf_cnt = 0;

void *dbgThread(void *arg0)
{

    while(!start_flag) {
        usleep(1000);
    }

    /* Loop forever */
    while (1) {


        //UARTMSP432_Object *object = uart_pq9_bus->object;

        //char temp[10];

          //while(RingBuf_get(&object->ringBuffer, &buf_rs[buf_cnt]) != -1) {
          //  buf_cnt++;
          //}
          //while(buf_cnt > 0) {
          //  sprintf(temp,"%02x ", buf_rs[buf_cnt]);
          // UART_write(uart_dbg_bus, temp, strlen(temp));
          //  buf_cnt--;
          //}
         usleep(10000);
    }

    return (NULL);
}
