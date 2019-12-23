#include "stdlib.h"

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"

#define PDM_PCM_FIFO_TRG_LVL        128u
#define BUFFER_SIZE                 256u
#define BUFFERS_NUM	                2u
#define THRESHOLD_HYSTERESIS        4u
#define VOLUME_RATIO                (1024u)

volatile uint16_t buffers[BUFFERS_NUM][BUFFER_SIZE];

volatile bool pdm_pcm_flag = false;
volatile bool writing_mode = false;

uint32_t volume = 0;
uint32_t num_samples = 0;
uint32_t noise_threshold = 2u;

volatile uint8_t sineidx = 0;

const uint16_t sinetable[256] = {31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 46, 47, 48, 48, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 53, 54, 54, 54, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 46, 46, 45, 45, 44, 43, 43, 42, 41, 41, 40, 39, 39, 38, 37, 36, 36, 35, 34, 33, 33, 32, 31, 30, 29, 29, 28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 21, 20, 19, 19, 18, 17, 17, 16, 16, 15, 14, 14, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 26, 26, 27, 28, 29, 29, 30};

uint8_t read_buffer_idx = 0;
uint8_t write_buffer_idx = 0;
uint16_t write_idx = 127;
uint16_t read_idx = 0;

const cy_stc_sysint_t pdm_pcm_isr_cfg = {
#if CY_IP_MXAUDIOSS_INSTANCES == 1
    .intrSrc = (IRQn_Type) audioss_interrupt_pdm_IRQn,
#else
    .intrSrc = (IRQn_Type) audioss_0_interrupt_pdm_IRQn,
#endif
    .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT
};

const cy_stc_sysint_t sine_isr_cfg = {
    .intrSrc = (IRQn_Type) tcpwm_1_interrupts_21_IRQn,
    .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT
};

const cy_stc_sysint_t fetcher_isr_cfg = {
    .intrSrc = (IRQn_Type) FETCHER_IRQ,
    .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT
};

bool led_state = false;

uint32_t chan = 0UL;

void SAR_Interrupt(void) {
    uint32_t intr_status = 0u;

    /* Read interrupt status register. */
    intr_status = Cy_SAR_GetInterruptStatus(SAR);

    /* Check what triggered the interrupt. */
    if ((intr_status & (uint32_t) CY_SAR_INTR_EOS_MASK) == (uint32_t) CY_SAR_INTR_EOS_MASK) {

        /* if (writing_mode) */
        buffers[write_buffer_idx][write_idx] = Cy_SAR_GetResult16(SAR, chan);


    	/* printf("%d\r\n", Cy_SAR_GetResult16(SAR, chan)); */

    	if (write_idx == BUFFER_SIZE) {
    		write_idx = 0;
    		write_buffer_idx += 1;

    		if (write_buffer_idx == BUFFERS_NUM) {
				write_buffer_idx = 0;
			}
    	}
    }

    /* Check for the saturation detection status, if enabled. */
    /* Check for the range detection status, if enabled. */
    /* Clear the handled interrupt. */
    Cy_SAR_ClearInterrupt(SAR, intr_status);
}

void example_of_work_with_sd() {
	// initialize card
	initCard();

	// erase all information on given addresses
	eraseSDBuffer(0, CY_SD_HOST_BLOCK_SIZE);

	uint8_t rxBuff[CY_SD_HOST_BLOCK_SIZE];   /* Receiver buffer. */
	uint8_t txBuff[CY_SD_HOST_BLOCK_SIZE];   /* Transmitter buffer. */

	/* Prepare the TX buffer */
	for(uint32_t index = 0; index < CY_SD_HOST_BLOCK_SIZE; index++) {
		txBuff[index] = (uint8_t) index;
		txBuff[index] =  10;
	}

	cy_stc_sd_host_write_read_config_t data = {
		.numberOfBlocks = 1UL, 						/* The number of blocks to write/read (Single block write/read). */
		.autoCommand = CY_SD_HOST_AUTO_CMD_NONE,	/* Selects which auto commands are used if any. */
		.dataTimeout = 12UL,						/* The timeout value for the transfer. */
		.enReliableWrite = false,					/* For EMMC cards enable reliable write. */
		.enableDma = true,							/* Enable DMA mode. */
		.data = (uint32_t*)txBuff,					/* The pointer to data to write. */
		.address = 0u
	};

	print_array("Written Data:", txBuff, CY_SD_HOST_BLOCK_SIZE);

	writeDataToSD(data);

	// change transmitter buffer to receiver one
	data.data = (uint32_t*)rxBuff;

	readDataFromSD(data);

	print_array("Read Data:", rxBuff, CY_SD_HOST_BLOCK_SIZE);
}

void button_isr_handler(void *callback_arg, cyhal_gpio_event_t event) {
    int status = !Cy_GPIO_Read(CYBSP_PIN12_PORT, CYBSP_PIN12_NUM);
    cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, status);
    Cy_GPIO_Write(CYBSP_PIN12_PORT, CYBSP_PIN12_NUM, status);

    writing_mode = !writing_mode;

    for (uint8_t idx = 0; idx < BUFFERS_NUM; idx++) {
        printf("buffer %d: \r\n", idx);

        for (uint16_t idxx = 0; idxx < BUFFER_SIZE; idxx++)
            printf("%d, ", buffers[idx][idxx]);

        printf("\r\n");
    }

    printf("writing_mode: %d\r\n", writing_mode);
}

void pdm_pcm_isr_handler() {
    pdm_pcm_flag = true;

    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);
}

void sine_isr_handler() {
    Cy_TCPWM_PWM_SetCompare0(ANOTHER_HW, ANOTHER_NUM, sinetable[sineidx++]);
    Cy_TCPWM_ClearInterrupt(SINE_GIVER_HW, SINE_GIVER_NUM, CY_TCPWM_INT_ON_TC);
}

void fetcher_isr_handler() {
    Cy_TCPWM_PWM_SetCompare0(PLAYER_HW, PLAYER_NUM, buffers[read_buffer_idx][read_idx++]);
    Cy_TCPWM_ClearInterrupt(FETCHER_HW, FETCHER_NUM, CY_TCPWM_INT_ON_TC);

    if (read_idx >= BUFFER_SIZE) {
        read_buffer_idx += 1;

        if (read_buffer_idx >= BUFFERS_NUM)
            read_buffer_idx = 0;

        read_idx = 0;
    }
}

void handle_error () {
    printf("sin²(θ) is odious to me\r\n");

    int status = 1;
    for (int idx = 0; idx < 16; idx++) {
        CyDelay(100);
        status = !status;
        cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, status);
    }

    CY_ASSERT(0);
}

int main() {
    if (cybsp_init() != CY_RSLT_SUCCESS)
        handle_error();

    __enable_irq();

    cy_rslt_t result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	// After all printing through UART REQUIRED "\r\n" in the end of message
	if(result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    // Initialize ADC
    ADC_init();

    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback((cyhal_gpio_t) CYBSP_USER_BTN, button_isr_handler, NULL);

    Cy_GPIO_Pin_FastInit(CYBSP_PIN12_PORT, CYBSP_PIN12_NUM, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);

    Cy_SysInt_Init(&pdm_pcm_isr_cfg, pdm_pcm_isr_handler);
    /* NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc); */
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_ClearFifo(CYBSP_PDM_PCM_HW);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);

    // 6-bit left pwm
    Cy_TCPWM_PWM_Init(PLAYER_HW, PLAYER_NUM, &PLAYER_config);
    Cy_TCPWM_PWM_Enable(PLAYER_HW, PLAYER_NUM);
    Cy_TCPWM_TriggerStart(PLAYER_HW, PLAYER_MASK);

    /* Cy_TCPWM_PWM_Init(ANOTHER_HW, ANOTHER_NUM, &ANOTHER_config); */
    /* Cy_TCPWM_PWM_Enable(ANOTHER_HW, ANOTHER_NUM); */
    /* Cy_TCPWM_TriggerStart(ANOTHER_HW, ANOTHER_MASK); */

    // Fetching 440Hz test sine
    /* Cy_SysInt_Init(&sine_isr_cfg, sine_isr_handler); */
    /* NVIC_EnableIRQ(sine_isr_cfg.intrSrc); */
    /* Cy_TCPWM_PWM_Init(SINE_GIVER_HW, SINE_GIVER_NUM, &SINE_GIVER_config); */
    /* Cy_TCPWM_TriggerStart(SINE_GIVER_HW, SINE_GIVER_MASK); */
    /* Cy_TCPWM_PWM_Enable(SINE_GIVER_HW, SINE_GIVER_NUM); */

    // fetching real data
    Cy_SysInt_Init(&fetcher_isr_cfg, fetcher_isr_handler);
    NVIC_EnableIRQ(fetcher_isr_cfg.intrSrc);
    Cy_TCPWM_PWM_Init(FETCHER_HW, FETCHER_NUM, &FETCHER_config);
    Cy_TCPWM_TriggerStart(FETCHER_HW, FETCHER_MASK);
    Cy_TCPWM_PWM_Enable(FETCHER_HW, FETCHER_NUM);

    printf(". . .\r\n");

    /* for (uint8_t idx = 0; idx < BUFFERS_NUM; idx++) { */
    /*     for (uint16_t idxx = 0; idxx < BUFFER_SIZE; idxx++) */
    /*         buffers[idx][idxx] = sinetable[idxx]; */
    /* } */

    uint64_t total = 0;
    uint64_t index = 0;

    for(;;) {
//    	if (pdm_pcm_flag) {
//            pdm_pcm_flag = 0;
//
//            if (volume_index >= 4) {
//                total /= 1024;
//                buffers[write_buffer_idx][write_idx] = total >> 10;
//
//                write_idx += 1;
//                volume_index = 0;
//                total = 0;
//            }
//
//            volume_index += 1;
//
//            for (uint8_t index = 0; index < PDM_PCM_FIFO_TRG_LVL; index++) {
//                buffers[write_buffer_idx][write_idx + index] = (uint32_t) Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW);
//            }
//
//            write_idx += PDM_PCM_FIFO_TRG_LVL;
//
//            Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, CY_PDM_PCM_INTR_RX_TRIGGER);
//            NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);
//
//            if (write_idx >= BUFFER_SIZE) {
//                // Moving onto the next buffer
//                printf("write_buffer_idx ~ %d\r\n", write_buffer_idx);
//                write_buffer_idx = (write_buffer_idx + 1) % BUFFERS_NUM;
//
//                if (write_buffer_idx == 0) {
//                    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);
//                    printf("done\r\n");
//                }
//
//                // Reset writing index
//                write_idx = 0;
//            }
//        }
    }
}
