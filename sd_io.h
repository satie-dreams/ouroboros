#include "cyhal.h"

#ifndef SD_IO_H_
#define SD_IO_H_

void prepareSDCardPins();
void SD_Host_User_Isr(void);
void initSDCardInterrupt();
void initCard();
void checkReadWriteStatus(cy_en_sd_host_status_t result);
void writeDataToSD(cy_stc_sd_host_write_read_config_t data);
void readDataFromSD(cy_stc_sd_host_write_read_config_t data);
void eraseSDBuffer(uint32_t start, uint32_t end);
void get_sectors_count(uint32_t* buff);
void sdcard_test();

#endif /* SD_IO_H_ */
