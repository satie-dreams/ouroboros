#include "stdlib.h"
#define CY_USING_HAL 1

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_device_headers.h"
#include "cycfg.h"
#include "cy_sysint.h"

#include "adc.h"
#include "sd_io.h"

#define DEBUG 1
#define DEBUG_ADC 0
#define DEBUG_PWM 0

#define BUFFER_SIZE                 512*512
#define SAR_CHANNEL                 0u

cyhal_gpio_t led = CYBSP_USER_LED;
cyhal_gpio_t little_button = CYBSP_USER_BTN;
cyhal_gpio_t hero_pin = HERO_HAL_PORT_PIN;

uint16_t buffer[BUFFER_SIZE];
uint8_t buf_idx = 0;

volatile bool pdm_pcm_flag = false;
volatile bool record_mode = false;
volatile bool fetch_flag = false;
volatile bool hero_flag = false;
volatile bool adc_flag = false;

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

uint32_t intr_status = 0u;
uint16_t adc_code = 0;

void SAR_Interrupt() {
    intr_status = Cy_SAR_GetInterruptStatus(SAR);

    if ((intr_status & (uint32_t) CY_SAR_INTR_EOS_MASK) == (uint32_t) CY_SAR_INTR_EOS_MASK) {
        adc_flag = true;
    }

    Cy_SAR_ClearInterrupt(SAR, intr_status);
}

void little_button_isr_handler(void *callback_arg, cyhal_gpio_event_t event) {
    cyhal_gpio_toggle(led);

    // cyclic buffer from read_idx to write_idx;
    read_idx = write_idx + 1;

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
    fetch_flag = true;

    Cy_TCPWM_ClearInterrupt(FETCHER_HW, FETCHER_NUM, CY_TCPWM_INT_ON_TC);
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


int main() {
    if (cybsp_init() != CY_RSLT_SUCCESS)
        handle_error("");

    __enable_irq();

    if (cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE) != CY_RSLT_SUCCESS)
        handle_error("");

	// \x1b[2J\x1b[;H - ANSI ESC sequence for clearing screen
    printf("\x1b[2J\x1b[;H");

    cy_rslt_t status = cyhal_gpio_init(
        led, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF
    );

    if (status != CY_RSLT_SUCCESS)
        handle_error("(init): failed led");

    cyhal_gpio_init(little_button, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(little_button, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(little_button, little_button_isr_handler, NULL);

    cyhal_gpio_init(hero_pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(hero_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(hero_pin, little_button_isr_handler, NULL);

    // two pwms for playing
    Cy_TCPWM_PWM_Init(PLAYER_L_HW, PLAYER_L_NUM, &PLAYER_L_config);
    Cy_TCPWM_PWM_Enable(PLAYER_L_HW, PLAYER_L_NUM);
    Cy_TCPWM_TriggerStart(PLAYER_L_HW, PLAYER_L_MASK);

    Cy_TCPWM_PWM_Init(PLAYER_R_HW, PLAYER_R_NUM, &PLAYER_R_config);
    Cy_TCPWM_PWM_Enable(PLAYER_R_HW, PLAYER_R_NUM);
    Cy_TCPWM_TriggerStart(PLAYER_R_HW, PLAYER_R_MASK);

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

    printf("(done initializing)\r\n");

    uint8_t lcode = 0;
    uint8_t rcode = 0;

    while (true) {
        if (hero_flag) {
            record_mode = !record_mode;

            read_idx = write_idx + 1;

            if (DEBUG)
                printf("?record_mode: %d\r\n", record_mode);

            hero_flag = 0;
        }

        if (fetch_flag) {
            if (!record_mode) {
                adc_code += buffer[read_idx++];

                if (read_idx >= BUFFER_SIZE) {
                    read_idx = 0;
                }
            }

            lcode = adc_code >> 6;
            rcode = adc_code & 0x3f;

            if (DEBUG_PWM)
                printf("(pwm): %u -> (%u, %u)\r\n", adc_code, lcode, rcode);

            if (!DEBUG_ADC) {
                Cy_TCPWM_PWM_SetCompare0(PLAYER_L_HW, PLAYER_L_NUM, lcode);
                Cy_TCPWM_PWM_SetCompare0(PLAYER_R_HW, PLAYER_R_NUM, rcode);
            }

            fetch_flag = false;
        }

        if (adc_flag) {
            adc_code = Cy_SAR_GetResult16(SAR, SAR_CHANNEL);

            if (record_mode) {
                buffer[write_idx++] = adc_code;

                if (write_idx >= BUFFER_SIZE) {
                    cyhal_gpio_toggle(led);
                    write_idx = 0;
                }
            }

            if (DEBUG_ADC) {
                printf("(adc): %d\r\n", adc_code);
            }

            adc_flag = false;
        }
    }
}
