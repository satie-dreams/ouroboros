#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "sd_io.h"

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
