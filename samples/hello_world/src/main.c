#include "pin_debug_transport.h"
#include "nrf.h"
uint32_t duration = 3;
uint32_t timer0stop = 80;
void mpsl_RTC0_isr_handler(void)
{
	DBP_ON(1);
	NRF_RTC0->EVENTS_COMPARE[0] = 0;
	NRF_RTC0->CC[0] = duration;
	NRF_RTC0->TASKS_START = 1;
	if (timer0stop == 120)
	{
		DBP_TOGGLE(3);
		timer0stop = 80;
	} else
	{
		DBP_TOGGLE(4);
		timer0stop++;
	}
	NRF_CLOCK->TASKS_HFCLKSTOP=1;
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->TASKS_CLEAR = 1;
	NRF_TIMER0->CC[0] = timer0stop;
	NRF_TIMER0->EVENTS_COMPARE[0] =0;
	NRF_TIMER0->TASKS_START = 1;
	DBP_OFF(1);
}

static volatile uint32_t dummy_variable = 0xa5a5a5a5;
int main(void)
{
	DBP_PORTS_ENABLE
	for (uint32_t i = 0; i < 8; i++)
	{
		DBP_ON(i);
	}
	for (uint32_t i = 0; i < 8; i++)
	{
		DBP_OFF(i);
	}
	NRF_CLOCK->PUBLISH_HFCLKSTARTED =  0 | (1 << 31);
	NRF_CLOCK->SUBSCRIBE_HFCLKSTART =  10 | (1 << 31);
	debug_pin_ch(DBP_PIN(8), 0);

	NRF_POWER->PUBLISH_SLEEPENTER =  2 | (1 << 31);
	NRF_POWER->PUBLISH_SLEEPEXIT =  3 | (1 << 31);
	debug_pin_ch_togle(DBP_PIN(12), 2, 3);
	IRQ_CONNECT(RTC0_IRQn, 0,mpsl_RTC0_isr_handler, 0, 0);
	irq_enable(RTC0_IRQn);
	NRF_RTC0->SHORTS = RTC_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_RTC0->CC[0] = duration;
	NRF_RTC0->INTENSET =RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->EVTENSET =RTC_INTENSET_COMPARE0_Msk;
	NRF_RTC0->TASKS_START = 1;
	NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Msk| TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER0->CC[0] = timer0stop;
	NRF_TIMER0->PUBLISH_COMPARE[0] = PUBLISH_EN_Msk | 10;
	NRF_TIMER0->TASKS_START = 1;
	debug_pin_ch(DBP_PIN(5), 10);
	debug_pin_ch(DBP_PIN(6), 11);
	SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
	while(1)
	{
		uint32_t i = 0;
		DBP_ON(2);
		__disable_irq();
		__NOP();__NOP();/* wfe instructnion needs to be at addres which is aliigned to 8 for this issue to happen*/
		__WFE();
		i = dummy_variable;
		if (!(i==0xa5a5a5a5))
		{
			DBP_TOGGLE(10);
			while(1)
			{
				__NOP();
			}
		}
		__enable_irq();
		DBP_OFF(2);
	}
	return 0;
}