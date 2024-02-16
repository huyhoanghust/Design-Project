#include "led.h"

static const led_t leds_rev1_0[] = {
    [ledRanging] = {.pin = LED_ACT_Pin, .port = LED_ACT_GPIO_Port},
    [ledSync] = {.pin = LED_SYS_Pin, .port = LED_SYS_GPIO_Port},
    [ledMode] = {.pin = LED_ERR_Pin, .port = LED_ERR_GPIO_Port}};

static bool isBlinking[N_LEDS];
static uint32_t disableTime[N_LEDS];

void ledInit(void)
{
    /* Do nothing */
    // Init in MX_GPIO_Init()
}
static inline void setLed(led_e led, bool value)
{
    HAL_GPIO_WritePin(leds_rev1_0[led].port, leds_rev1_0[led].pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ledOn(led_e led)
{
    isBlinking[led] = false;
    setLed(led, true);
}

void ledOff(led_e led)
{
    isBlinking[led] = false;
    setLed(led, false);
}

void ledBlink(led_e led, bool oneshot)
{
    isBlinking[led] = true;
    if (oneshot)
    {
        disableTime[led] = HAL_GetTick() + 50;
        setLed(led, true);
    }
}
void ledTick()
{
    static uint32_t last_tick;
    static bool blinkStatus;

    for (int led = 0; led < N_LEDS; led++)
    {
        if (isBlinking[led] && disableTime[led] && disableTime[led] < HAL_GetTick())
        {
            setLed(led, false);
            disableTime[led] = 0;
            isBlinking[led] = false;
        }
    }

    if (HAL_GetTick() > (last_tick + 250))
    {
        blinkStatus = !blinkStatus;
        last_tick = HAL_GetTick();
        for (int led = 0; led < N_LEDS; led++)
        {
            if (isBlinking[led] && !disableTime[led])
            {
                setLed(led, blinkStatus);
            }
        }
    }
}
