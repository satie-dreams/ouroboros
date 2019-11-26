#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "stdlib.h"

/* Trigger level configured in the PDM/PCM */
#define PDM_PCM_FIFO_TRG_LVL        128u
/* Define how many samples in a frame */
#define FRAME_SIZE                  (4*PDM_PCM_FIFO_TRG_LVL)
/* Noise threshold hysteresis */
#define THRESHOLD_HYSTERESIS        3u
/* Volume ratio for noise and print purposes */
#define VOLUME_RATIO                (1024u)

volatile bool button_flag = false;
volatile bool pdm_pcm_flag = false;

uint32_t volume = 0;
uint32_t num_samples = 0;
uint32_t noise_threshold = THRESHOLD_HYSTERESIS;

const cy_stc_sysint_t pdm_pcm_isr_cfg = {
#if CY_IP_MXAUDIOSS_INSTANCES == 1
    .intrSrc = (IRQn_Type) audioss_interrupt_pdm_IRQn,
#else
    .intrSrc = (IRQn_Type) audioss_0_interrupt_pdm_IRQn,
#endif
    .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT
};

void button_isr_handler(void *callback_arg, cyhal_gpio_event_t event) {
    button_flag = true;
}

void pdm_pcm_isr_handler() {
    pdm_pcm_flag = true;

    /* Disable PDM/PCM ISR to avoid multiple calls to this ISR */
    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);
}

int main() {
    cy_rslt_t result;

    if (cybsp_init() != CY_RSLT_SUCCESS)
        CY_ASSERT(0);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the User Button */
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback((cyhal_gpio_t) CYBSP_USER_BTN, button_isr_handler, NULL);

    /* Initialize the PDM/PCM interrupt (PDL) */
    Cy_SysInt_Init(&pdm_pcm_isr_cfg, pdm_pcm_isr_handler);
    NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);

    /* Initialize the PDM/PCM block (PDL) */
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_ClearFifo(CYBSP_PDM_PCM_HW);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);

    printf("\x1b[2J\x1b[;H");

    for(;;) {
        if (pdm_pcm_flag) {
            pdm_pcm_flag = 0;

            /* Calculate the volume by summing the absolute value of all the
             * data stored in the PDM/PCM FIFO */
            for (uint32_t index = 0; index < PDM_PCM_FIFO_TRG_LVL; index++) {
                volume += abs((int32_t) Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW));
            }

            /* Add to the number of samples */
            num_samples += PDM_PCM_FIFO_TRG_LVL;

            /* Clear the PDM/PCM interrupt */
            Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, CY_PDM_PCM_INTR_RX_TRIGGER);

            /* Re-enable PDM/PCM ISR */
            NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);

            /* Check if ready to process an entire frame */
            if (num_samples >= FRAME_SIZE) {
                /* Prepare line to report the volume */
                printf("\n\r");

                /* Report the volume */
                for (uint32_t index = 0; index < (volume/VOLUME_RATIO); index++)
                    printf("-");

                /* Turn ON the LED when the volume is higher than the threshold */
                if ((volume/VOLUME_RATIO) > noise_threshold)
                    cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                else
                    cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

                /* Reset the number of samples and volume */
                num_samples = 0;
                volume = 0;
            }
        }

        /* Reset the noise threshold if User Button is pressed */
        if (button_flag) {
            /* Reset button flag */
            button_flag = false;

            /* Get the current volume and add a hysteresis as the new threshold */
            noise_threshold = (volume/VOLUME_RATIO) + THRESHOLD_HYSTERESIS;

            /* Report the new noise threshold over UART */
            printf("Noise threshold: %lu\n\r", (uint32_t) noise_threshold);
        }

    }
}
