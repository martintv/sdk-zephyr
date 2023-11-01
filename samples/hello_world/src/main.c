#include "nrf.h"
#include <stdint.h>

#define NRF_CLOCK NRF_CLOCK_NS
#define NRF_RTC0 NRF_RTC0_NS
#define NRF_TIMER0 NRF_TIMER0_NS
#define NRF_DPPIC NRF_DPPIC_NS
uint32_t duration = 3;
uint32_t timer0stop = 80;

void RTC0_IRQHandler(void)
{
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NRF_RTC0->CC[0] = duration;
	NRF_RTC0->TASKS_START = 1;
	if (timer0stop == 120)
	{
		timer0stop = 80;
	} else
	{
		timer0stop++;
	}
	NRF_CLOCK->TASKS_HFCLKSTOP=1;
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->CC[0] = timer0stop;
	NRF_TIMER0->EVENTS_COMPARE[0] =0;
	NRF_TIMER0->TASKS_START = 1;
}

static volatile uint32_t dummy_variable = 0xa5a5a5a5;
int main(void)
{
	NVIC_EnableIRQ(RTC0_IRQn);
	NRF_CLOCK->SUBSCRIBE_HFCLKSTART = 10 | (1 << 31);
	NRF_RTC0->SHORTS = RTC_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_RTC0->CC[0] = duration;
	NRF_RTC0->INTENSET =RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->EVTENSET =RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->TASKS_START = 1;

	NRF_DPPIC->CHENSET = (1 << 10);
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Msk| TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER0->CC[0] = timer0stop;
	NRF_TIMER0->PUBLISH_COMPARE[0] = TIMER_PUBLISH_COMPARE_EN_Msk | 10;
	NRF_TIMER0->TASKS_START = 1;

	SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
	while(1)
	{
		uint32_t i = 0;
		__disable_irq();
		__NOP();__NOP();/* wfe instructnion needs to be at addres which is aliigned to 8 for this issue to happen*/
		__WFE();
		i = dummy_variable;
		if (!(i==0xa5a5a5a5))
		{
			while(1)
			{
				__NOP();
			}
		}
		__enable_irq();
	}
	return 0;
}
