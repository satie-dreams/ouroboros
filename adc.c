#include "adc.h"

const cy_stc_sysint_t SAR_IRQ_cfg = {
    .intrSrc        	= pass_interrupt_sar_IRQn,
    .intrPriority   	= 0
};

/* ISR function to handle all SAR interrupts.
 * This same routine gets called when any of the enabled SAR interrupt sources
 * are enabled (EOS, overflow, FW collision, saturation detection, or range detection). */
void SAR_Interrupt(void)
{
    uint32_t intr_status = 0u;
    /* Read interrupt status register. */
    intr_status = Cy_SAR_GetInterruptStatus(SAR);
    /* Check what triggered the interrupt. */
    if ((intr_status & (uint32_t) CY_SAR_INTR_EOS_MASK) == (uint32_t) CY_SAR_INTR_EOS_MASK)
    {
    	uint32_t chan = 0UL;

    	printf("%d\r\n", Cy_SAR_GetResult16(SAR, chan));

//    	Cy_SysLib_Delay(500);
        /* An end of scan occurred, retrieve the ADC result and do something with it here. */
    }
    /* Check for the saturation detection status, if enabled. */
    /* Check for the range detection status, if enabled. */
    /* Clear the handled interrupt. */
    Cy_SAR_ClearInterrupt(SAR, intr_status);
}

void ADC_init() {
	cy_en_sysanalog_status_t status_aref;
	status_aref = Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);

	/* Turn on the hardware block. */
	Cy_SysAnalog_Enable();

	/* After the AREF is enabled, enable the consumer blocks (SAR, CTDAC, CTB, and CSDv2). */
	cy_en_sar_status_t status;
	status = Cy_SAR_Init(SAR, &pass_0_sar_0_config);

	cy_en_tcpwm_status_t status_timer;
	status_timer = Cy_TCPWM_PWM_Init(TCPWM0, 0, &tcpwm_0_cnt_0_config);

	Cy_TCPWM_PWM_Enable(TCPWM0, 0);
	Cy_TCPWM_TriggerStart(TCPWM0, 1);

	(void)Cy_SysInt_Init(&SAR_IRQ_cfg, SAR_Interrupt);
	NVIC_EnableIRQ(SAR_IRQ_cfg.intrSrc);

	if (CY_SAR_SUCCESS == status) {
		/* Turn on the SAR hardware. */
		Cy_SAR_Enable(SAR);
		/* Begin continuous conversions. */
		Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
	}
}
