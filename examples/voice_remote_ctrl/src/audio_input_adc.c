#include "ingsoc.h"
#include "platform_api.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>

#include "app_cfg.h"

#include "audio.h"

#if (INGCHIPS_FAMILY == INGCHIPS_FAMILY_918)

uint32_t audio_sample_isr(void *user_data)
{    
    pcm_sample_t sample;

    TMR_IntClr(APB_TMR1);

    sample = ADC_ReadChannelData(ADC_CHANNEL_ID) - 512;
    
    audio_rx_sample(sample);

    return 0;
}

void audio_input_setup(void)
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ClkGate_APB_TMR1));

    // timer 0 can be used as watchdog, so we use timer 1.
    // setup timer 1 to sampling rate
    TMR_SetCMP(APB_TMR1, TMR_CLK_FREQ / SAMPLING_RATE);
    TMR_SetOpMode(APB_TMR1, TMR_CTL_OP_MODE_WRAPPING);
    TMR_Reload(APB_TMR1);
    TMR_IntEnable(APB_TMR1);

    ADC_PowerCtrl(1);
    ADC_Reset();
    ADC_SetClkSel(ADC_CLK_EN | ADC_CLK_16);
    ADC_SetMode(ADC_MODE_LOOP);
    ADC_EnableChannel(ADC_CHANNEL_ID, 1);
    ADC_Enable(1);
    
    platform_set_irq_callback(PLATFORM_CB_IRQ_TIMER1, audio_sample_isr, NULL);
}

void audio_input_start(void)
{
    TMR_Enable(APB_TMR1);
}

void audio_input_stop(void)
{
    TMR_Disable(APB_TMR1);
}
#elif (INGCHIPS_FAMILY == INGCHIPS_FAMILY_916)
#define ADC_CHANNEL    ADC_CH_0
#define DMA_CHANNEL    0
static DMA_PingPang_t PingPang;

static uint32_t DMA_cb_isr(void *user_data)
{
    uint32_t state = DMA_GetChannelIntState(0);
    DMA_ClearChannelIntState(0, state);

    uint32_t *buff = DMA_PingPangIntProc(&PingPang, DMA_CHANNEL);
    uint32_t tranSize = DMA_PingPangGetTransSize(&PingPang);
    for (uint32_t i = 0; i < tranSize; ++i) {
		if (ADC_GetDataChannel(buff[i]) != ADC_CHANNEL) continue;
        uint16_t sample = ADC_GetData(buff[i]);
		// do something with 'sample'
    }
    return 0;
}
void audio_input_setup(void)
{
    SYSCTRL_ClearClkGateMulti((1 << SYSCTRL_ITEM_APB_DMA));

    ADC_ClkCfg(SADC_CLK_6M);
    ADC_Calibration(DIFFERENTAIL_MODE);
    ADC_ConvCfg(CONTINUES_MODE, PGA_GAIN_4, 1, ADC_CHANNEL, 0, 8, DIFFERENTAIL_MODE, 6100000/100);
    ADC_SetDmaSel(0);

    DMA_PingPangSetup(&PingPang, SYSCTRL_DMA_ADC, 80, 8);
    platform_set_irq_callback(PLATFORM_CB_IRQ_DMA, DMA_cb_isr, 0);
}
void audio_input_start(void)
{
    DMA_PingPangEnable(&PingPang, DMA_CHANNEL);
    ADC_Start(1);
}

void audio_input_stop(void)
{
    DMA_PingPangDisable(&PingPang, DMA_CHANNEL);
    ADC_Start(0);
}

#endif
