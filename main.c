#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_device_headers.h"

#include "stdlib.h"

#define PDM_PCM_FIFO_TRG_LVL        128u
#define FRAME_SIZE                  (4*PDM_PCM_FIFO_TRG_LVL)
#define THRESHOLD_HYSTERESIS        4u
#define VOLUME_RATIO                (1024u)

volatile bool button_flag = false;
volatile bool pdm_pcm_flag = false;
volatile bool sine_change_flag = false;

uint32_t volume = 0;
uint32_t num_samples = 0;
uint32_t noise_threshold = 2u;

volatile uint8_t sineidx = 0;

const uint8_t sinetable[256] = {31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 46, 47, 48, 48, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 53, 54, 54, 54, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 46, 46, 45, 45, 44, 43, 43, 42, 41, 41, 40, 39, 39, 38, 37, 36, 36, 35, 34, 33, 33, 32, 31, 30, 29, 29, 28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 21, 20, 19, 19, 18, 17, 17, 16, 16, 15, 14, 14, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 26, 26, 27, 28, 29, 29, 30};


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

bool led_state = false;


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

/* Allocate context for SD Host operation */
cy_stc_sd_host_context_t sdHostContext;
cy_en_sd_host_card_type_t cardType;
uint32_t rca;
cy_en_sd_host_card_capacity_t cardCapacity;

void prepareSDCardPins() {
	Cy_SD_Host_Enable(SDHC1);

	/* Connect SD Host SDHC function to pins */
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_IO_VOLT_SEL_NUM, P12_7_SDHC1_IO_VOLT_SEL);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CARD_DETECT_N_NUM, P12_1_SDHC1_CARD_DETECT_N);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CARD_MECH_WRITE_PROT_NUM, P12_2_SDHC1_CARD_MECH_WRITE_PROT);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_LED_CTRL_NUM, P12_3_SDHC1_LED_CTRL);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CARD_IF_PWR_EN_NUM, P12_6_SDHC1_CARD_IF_PWR_EN);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CARD_EMMC_RESET_N_NUM, P12_0_SDHC1_CARD_EMMC_RESET_N);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CARD_CMD_NUM, P12_4_SDHC1_CARD_CMD);
	Cy_GPIO_SetHSIOM(SDHC1_PORT1, SDHC1_CLK_CARD_NUM, P12_5_SDHC1_CLK_CARD);
	Cy_GPIO_SetHSIOM(SDHC1_PORT2, SDHC1_CARD_DAT_3TO00_NUM, P13_0_SDHC1_CARD_DAT_3TO00);
	Cy_GPIO_SetHSIOM(SDHC1_PORT2, SDHC1_CARD_DAT_3TO01_NUM, P13_1_SDHC1_CARD_DAT_3TO01);
	Cy_GPIO_SetHSIOM(SDHC1_PORT2, SDHC1_CARD_DAT_3TO02_NUM, P13_2_SDHC1_CARD_DAT_3TO02);
	Cy_GPIO_SetHSIOM(SDHC1_PORT2, SDHC1_CARD_DAT_3TO03_NUM, P13_3_SDHC1_CARD_DAT_3TO03);
	/* Configure pins for SDHC operation */
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_IO_VOLT_SEL_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CARD_DETECT_N_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CARD_MECH_WRITE_PROT_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_LED_CTRL_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CARD_IF_PWR_EN_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CARD_EMMC_RESET_N_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CARD_CMD_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT1, SDHC1_CLK_CARD_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT2, SDHC1_CARD_DAT_3TO00_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT2, SDHC1_CARD_DAT_3TO01_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT2, SDHC1_CARD_DAT_3TO02_NUM, CY_GPIO_DM_STRONG);
	Cy_GPIO_SetDrivemode(SDHC1_PORT2, SDHC1_CARD_DAT_3TO03_NUM, CY_GPIO_DM_STRONG);

	/* Note: The CLK_HF2 input clock must be configured and enabled. */
	/* Apply the CLK_HF2 divider to have CLK_HF2 = 100 MHz. */
	Cy_SysClk_ClkHfSetSource(2u, CY_SYSCLK_CLKHF_IN_CLKPATH0);
	Cy_SysClk_ClkHfSetDivider(2u, CY_SYSCLK_CLKHF_NO_DIVIDE);
	Cy_SysClk_ClkHfEnable(2u);

}

void SD_Host_User_Isr(void) {
    uint32_t normalStatus;
    uint32_t errorStatus;

    normalStatus = Cy_SD_Host_GetNormalInterruptStatus(SDHC1);
    /* Check the Error event */
    if (0u < normalStatus) {
        /* Clear the normalStatus event */
        Cy_SD_Host_ClearNormalInterruptStatus(SDHC1, normalStatus);
    }

    errorStatus = Cy_SD_Host_GetErrorInterruptStatus(SDHC1);
    /* Check the Error event */
    if (0u < errorStatus) {
        /* Clear the Error event */
        Cy_SD_Host_ClearErrorInterruptStatus(SDHC1, errorStatus);
    }

    /* Add the use code here. */
}


void print_array(char *message, uint8_t *buf, uint32_t size) {
    printf("\r\n%s (%lu bytes):\r\n", message, size);
    printf("-------------------------\r\n");

    for(uint32_t index = 0; index < size; index++) {
    	if (buf[index] < 10) {
    		printf(" ");
    	}

    	if (buf[index] < 100) {
			printf(" ");
		}
        printf("%d ", buf[index]);

        if(0u == ((index + 1) % 40)) {
            printf("\r\n");
        }
    }

    // REQUIRED for proper working of UART
    printf("\r\n");
}

void initSDCardInterrupt() {
	/* Assign SDHC1 interrupt number and priority */
	#define SD_Host_INTR_NUM        sdhc_1_interrupt_general_IRQn
	#define SD_Host_INTR_PRIORITY   (3UL)
	/* Populate configuration structure (code specific for CM4) */
	const cy_stc_sysint_t sdHostIntrConfig = {
		#if (CY_CPU_CORTEX_M0P)
			 /* .intrSrc */ NvicMux4_IRQn,
			/* .cm0pSrc */ SD_Host_INTR_NUM,
		#else
			 /* .intrSrc */ SD_Host_INTR_NUM, /* SD Host interrupt number (non M0 core)*/
		#endif
		/* .intrPriority */ SD_Host_INTR_PRIORITY
	};


	/* Hook interrupt service routine and enable interrupt */
	(void) Cy_SysInt_Init(&sdHostIntrConfig, &SD_Host_User_Isr);
	#if (CY_CPU_CORTEX_M0P)
		NVIC_EnableIRQ(NvicMux4_IRQn);
	#else
		NVIC_EnableIRQ(SD_Host_INTR_NUM);
	#endif
}

void initCard() {
	prepareSDCardPins();

	initSDCardInterrupt();

	cy_en_sd_host_status_t initResult;

	/* Populate configuration structure */
	const cy_stc_sd_host_init_config_t sdHostConfig = {
		.dmaType = CY_SD_HOST_DMA_ADMA2,
		.enableLedControl = false,
		.emmc = false,
	};

	/* Configure SD Host to operate */
	initResult = Cy_SD_Host_Init(SDHC1, &sdHostConfig, &sdHostContext);
	if (initResult != CY_SD_HOST_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Populate configuration structure */
	cy_stc_sd_host_sd_card_config_t sdCardConfig = {
		.lowVoltageSignaling = false,
		.busWidth = CY_SD_HOST_BUS_WIDTH_4_BIT,
		.cardType = &cardType,
		.rca = &rca,
		.cardCapacity = &cardCapacity,
	};

	/* Initialize the card */
	initResult = Cy_SD_Host_InitCard(SDHC1, &sdCardConfig, &sdHostContext);
	if(initResult != CY_SD_HOST_SUCCESS ) {
		CY_ASSERT(0);
	}
}


void checkReadWriteStatus(cy_en_sd_host_status_t result) {
	if(result != CY_SD_HOST_SUCCESS ) {
		CY_ASSERT(0);
		return;
	}

	while (CY_SD_HOST_XFER_COMPLETE != (Cy_SD_Host_GetNormalInterruptStatus(SDHC1) & CY_SD_HOST_XFER_COMPLETE)) {
		/* Wait for the data-transaction complete event. */
	}
	/* Clear the data-transaction complete event. */
	Cy_SD_Host_ClearNormalInterruptStatus(SDHC1, CY_SD_HOST_XFER_COMPLETE);
}

void writeDataToSD(cy_stc_sd_host_write_read_config_t data) {
	checkReadWriteStatus(
		Cy_SD_Host_Write(SDHC1, &data, &sdHostContext)
	);
}

void readDataFromSD(cy_stc_sd_host_write_read_config_t data) {
	checkReadWriteStatus(
		Cy_SD_Host_Read(SDHC1, &data, &sdHostContext)
	);
}


void eraseSDBuffer(uint32_t start, uint32_t end) {
	cy_en_sd_host_status_t sd_result = Cy_SD_Host_Erase(SDHC1, start, end, CY_SD_HOST_ERASE_FULE , &sdHostContext);
	if(sd_result != CY_SD_HOST_SUCCESS ) {
		CY_ASSERT(0);
	}
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

    button_flag = true;
}

void pdm_pcm_isr_handler() {
    pdm_pcm_flag = true;

    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);
}

void sine_isr_handler() {
    Cy_TCPWM_PWM_SetCompare0(PLAYER_HW, PLAYER_NUM, sinetable[sineidx++]);
    Cy_TCPWM_ClearInterrupt(SINE_GIVER_HW, SINE_GIVER_NUM, CY_TCPWM_INT_ON_TC);
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

    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback((cyhal_gpio_t) CYBSP_USER_BTN, button_isr_handler, NULL);

    Cy_GPIO_Pin_FastInit(CYBSP_PIN12_PORT, CYBSP_PIN12_NUM, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);

    Cy_SysInt_Init(&pdm_pcm_isr_cfg, pdm_pcm_isr_handler);
    NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_ClearFifo(CYBSP_PDM_PCM_HW);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);

    // 6-bit left pwm
    Cy_TCPWM_PWM_Init(PLAYER_HW, PLAYER_NUM, &PLAYER_config);
    Cy_TCPWM_PWM_Enable(PLAYER_HW, PLAYER_NUM);
    Cy_TCPWM_TriggerStart(PLAYER_HW, PLAYER_MASK);

    // fecthing 440Hz test sine
    Cy_SysInt_Init(&sine_isr_cfg, sine_isr_handler);
    NVIC_EnableIRQ(sine_isr_cfg.intrSrc);
    Cy_TCPWM_PWM_Init(SINE_GIVER_HW, SINE_GIVER_NUM, &SINE_GIVER_config);
    Cy_TCPWM_PWM_Enable(SINE_GIVER_HW, SINE_GIVER_NUM);
    Cy_TCPWM_TriggerStart(SINE_GIVER_HW, SINE_GIVER_MASK);

    printf(". . .\r\n");

    for(;;) {
    	if (pdm_pcm_flag) {
            pdm_pcm_flag = 0;

            for (uint32_t index = 0; index < PDM_PCM_FIFO_TRG_LVL; index++) {
                volume += abs((int32_t) Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW));
            }

            num_samples += PDM_PCM_FIFO_TRG_LVL;

            Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, CY_PDM_PCM_INTR_RX_TRIGGER);

            NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);

            if (num_samples >= FRAME_SIZE) {
                printf("\r\n");

                for (uint32_t index = 0; index < (volume/VOLUME_RATIO); index++)
                    printf("-");

                if ((volume/VOLUME_RATIO) > noise_threshold)
                    cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                else
                    cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

                num_samples = 0;
                volume = 0;
            }
        }

        if (sine_change_flag) {
            sine_change_flag = false;
        }

        if (button_flag) {
            button_flag = false;
            noise_threshold += 4u;
            printf("Noise threshold: %lu\r\n", (uint32_t) noise_threshold);
        }
    }
}
