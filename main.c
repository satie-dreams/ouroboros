#include "stdlib.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"

#include "adc.h"

#define DEBUG 1
#define DEBUG_ADC 0
#define CY_USING_HAL 1

#define PDM_PCM_FIFO_TRG_LVL        128u
// How many blocks to fetch in one go
#define BLOCK_BATCH_NUM             2
// How many of 16bit to store in one ping pong buffer
#define BUFFER_SIZE                 512*512
// ping pong buffer count
#define BUFFERS_NUM	                2u
#define THRESHOLD_HYSTERESIS        4u
#define VOLUME_RATIO                (1024u)

cyhal_gpio_t led = CYBSP_USER_LED;
cyhal_gpio_t little_button = CYBSP_USER_BTN;
cyhal_gpio_t hero_pin = HERO_HAL_PORT_PIN;

uint16_t buffer[BUFFER_SIZE];

volatile bool pdm_pcm_flag = false;
volatile bool record_mode = false;

cy_stc_sd_host_context_t sdHostContext;
uint32_t volume = 0;
uint32_t num_samples = 0;
uint32_t noise_threshold = 2u;

volatile uint8_t sineidx = 0;

static const uint16_t sinetable[256] = {31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 46, 47, 48, 48, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 53, 54, 54, 54, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 48, 48, 47, 46, 46, 45, 45, 44, 43, 43, 42, 41, 41, 40, 39, 39, 38, 37, 36, 36, 35, 34, 33, 33, 32, 31, 30, 29, 29, 28, 27, 26, 26, 25, 24, 23, 23, 22, 21, 21, 20, 19, 19, 18, 17, 17, 16, 16, 15, 14, 14, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 26, 26, 27, 28, 29, 29, 30};

uint16_t write_idx = 0;
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

uint32_t chan = 0UL;
uint32_t intr_status = 0u;
uint16_t res = 0;
uint16_t adc_code = 0;

void SAR_Interrupt() {
    intr_status = Cy_SAR_GetInterruptStatus(SAR);

    if ((intr_status & (uint32_t) CY_SAR_INTR_EOS_MASK) == (uint32_t) CY_SAR_INTR_EOS_MASK) {
        adc_code = Cy_SAR_GetResult16(SAR, chan) >> 6;

        Cy_TCPWM_PWM_SetCompare0(PLAYER_HW, PLAYER_NUM, adc_code);

        if (record_mode) {
            buffer[write_idx++] = adc_code;
        }

        if (DEBUG_ADC)
            printf("(adc): %d\r\n", adc_code);
    }

    Cy_SAR_ClearInterrupt(SAR, intr_status);
}

void little_button_isr_handler(void *callback_arg, cyhal_gpio_event_t event) {
    cyhal_gpio_toggle(led);
    cyhal_gpio_toggle(hero_pin);

    record_mode = !record_mode;

    if (DEBUG)
        printf("record_mode: %d\r\n", record_mode);
}

void pdm_pcm_isr_handler() {
    pdm_pcm_flag = true;

    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);
}

void sine_isr_handler() {
    Cy_TCPWM_PWM_SetCompare0(SINE_PLAYER_HW, SINE_PLAYER_NUM, sinetable[sineidx++]);
    Cy_TCPWM_ClearInterrupt(SINE_FETCHER_HW, SINE_FETCHER_NUM, CY_TCPWM_INT_ON_TC);
}

void fetcher_isr_handler() {
    Cy_TCPWM_ClearInterrupt(FETCHER_HW, FETCHER_NUM, CY_TCPWM_INT_ON_TC);

    if (!record_mode) {
        Cy_TCPWM_PWM_SetCompare0(PLAYER_HW, PLAYER_NUM, buffer[read_idx++]);

        if (read_idx >= BUFFER_SIZE)
            read_idx = 0;
    }
}

void handle_error(char* msg) {
    printf(msg);

    uint8_t times = 16;

    while (times --> 0) {
        CyDelay(150);
        cyhal_gpio_toggle(led);
    }

    CY_ASSERT(0);
}

cy_rslt_t status;

int main() {
    if (cybsp_init() != CY_RSLT_SUCCESS)
        handle_error("");

    __enable_irq();

    if (cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE) != CY_RSLT_SUCCESS)
        handle_error("");

	// \x1b[2J\x1b[;H - ANSI ESC sequence for clearing screen
    printf("\x1b[2J\x1b[;H");

    status = cyhal_gpio_init(led, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (status != CY_RSLT_SUCCESS)
        handle_error("(init): failed led");

    cyhal_gpio_init(little_button, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(little_button, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(little_button, little_button_isr_handler, NULL);

    // most of those pins are dedicated, can't repurpose them unless init'ing directly
    Cy_GPIO_Pin_FastInit(HERO_PORT, HERO_NUM, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);

    // 6-bit left pwm
    Cy_TCPWM_PWM_Init(PLAYER_HW, PLAYER_NUM, &PLAYER_config);
    Cy_TCPWM_PWM_Enable(PLAYER_HW, PLAYER_NUM);
    Cy_TCPWM_TriggerStart(PLAYER_HW, PLAYER_MASK);

    // fetching real data
    Cy_SysInt_Init(&fetcher_isr_cfg, fetcher_isr_handler);
    NVIC_EnableIRQ(fetcher_isr_cfg.intrSrc);
    Cy_TCPWM_PWM_Init(FETCHER_HW, FETCHER_NUM, &FETCHER_config);
    Cy_TCPWM_TriggerStart(FETCHER_HW, FETCHER_MASK);
    Cy_TCPWM_PWM_Enable(FETCHER_HW, FETCHER_NUM);

    // playing testing wave
    Cy_TCPWM_PWM_Init(SINE_PLAYER_HW, SINE_PLAYER_NUM, &SINE_PLAYER_config);
    Cy_TCPWM_PWM_Enable(SINE_PLAYER_HW, SINE_PLAYER_NUM);
    Cy_TCPWM_TriggerStart(SINE_PLAYER_HW, SINE_PLAYER_MASK);

    // fetching testing wave
    Cy_SysInt_Init(&sine_isr_cfg, sine_isr_handler);
    NVIC_EnableIRQ(sine_isr_cfg.intrSrc);
    Cy_TCPWM_PWM_Init(SINE_FETCHER_HW, SINE_FETCHER_NUM, &SINE_FETCHER_config);
    Cy_TCPWM_TriggerStart(SINE_FETCHER_HW, SINE_FETCHER_MASK);
    Cy_TCPWM_PWM_Enable(SINE_FETCHER_HW, SINE_FETCHER_NUM);

    adc_init();

    printf(". . .\r\n");

    while (true) {
        CyDelay(100);
        cyhal_gpio_toggle(led);
    }
}
