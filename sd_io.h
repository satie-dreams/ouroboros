/*
 * sd_io.h
 *
 *  Created on: Dec 16, 2019
 *      Author: denysmaletsden
 */

#include "cyhal.h"

#ifndef SD_IO_H_
#define SD_IO_H_

/* Assign pins for SD Host on SDHC1 */
#define SDHC1_PORT1   (P12_0_PORT)
#define SDHC1_PORT2   (P13_0_PORT)


#define SDHC1_IO_VOLT_SEL_NUM           (P12_7_NUM)
#define SDHC1_CARD_DETECT_N_NUM         (P12_1_NUM)
#define SDHC1_CARD_MECH_WRITE_PROT_NUM  (P12_2_NUM)
#define SDHC1_LED_CTRL_NUM              (P12_3_NUM)
#define SDHC1_CARD_IF_PWR_EN_NUM        (P12_6_NUM)
#define SDHC1_CARD_EMMC_RESET_N_NUM     (P12_0_NUM)
#define SDHC1_CARD_CMD_NUM              (P12_4_NUM)
#define SDHC1_CLK_CARD_NUM              (P12_5_NUM)
#define SDHC1_CARD_DAT_3TO00_NUM        (P13_0_NUM)
#define SDHC1_CARD_DAT_3TO01_NUM        (P13_1_NUM)
#define SDHC1_CARD_DAT_3TO02_NUM        (P13_2_NUM)
#define SDHC1_CARD_DAT_3TO03_NUM        (P13_3_NUM)

/* Used when an array of data is printed on the console */
#define NUM_BYTES_PER_LINE      (16u)
#define LED_TOGGLE_DELAY_MSEC   (1000u)   /* LED blink delay */
#define MEM_SLOT_NUM            (0u)      /* Slot number of the memory to use */
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)
#define FLASH_DATA_AFTER_ERASE  (0xFFu)   /* Flash data after erase */

void prepareSDCardPins();
void SD_Host_User_Isr(void);
void initSDCardInterrupt();
void initCard();
void checkReadWriteStatus(cy_en_sd_host_status_t result);
void writeDataToSD(cy_stc_sd_host_write_read_config_t data);
void readDataFromSD(cy_stc_sd_host_write_read_config_t data);
void eraseSDBuffer(uint32_t start, uint32_t end);
void get_sectors_count(uint32_t* buff);


#endif /* SD_IO_H_ */
