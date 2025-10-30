#include "stm32f446xx.h"
#include "lcd.h"
#include <math.h>
#include "arm_math.h"
#include <stdio.h>
#include "stm32f446xx_gpio_driver.h"
#include <stdint.h>

#define ENABLE 	1
//macrodefinitions
#define ADC_CHANNEL		0//A0 pin
#define BUFFER_SIZE		1024
#define SAMPLING_RATE	8000 //sampling frequency in hertz
#define APB1CLK			48000000
//PC13 is the button to configure interrupt


//function prototypes
void ADC_Init(void);
void TIM2_Init(void);
void DMA_Init(void);
void process_buffer(void);
void EXTI15_10_IRQHandler(void);
void Start_Conversion(void);
void Delay_ms(uint32_t milliseconds);
void Delay_us(uint32_t microseconds);
void Capture_Samples(void);
void Start_Conversion(void);
void Stop_Conversion(void);

//global declarations
volatile uint8_t start_sampling=0;//Flag set by button interrupt
uint16_t adc_buffer[BUFFER_SIZE];
float windowed_buffer[BUFFER_SIZE];
float fft_real[BUFFER_SIZE];
float fft_imag[BUFFER_SIZE];
float magnitude[(BUFFER_SIZE)/2];
uint32_t sample_index=0;
//STANDARD GUITAR NOTES(standard Tuning)
const char* notes[]={"E2","A2","D3","G3","B3","E4"};
const float note_freq[]={82.41,110.0,146.83,196.0,246.94,329.63};
arm_rfft_fast_instance_f32 fft_inst;
int peak_index=0;
float max_val=0.0f;
int closest =0;


//Compute Magnitude
float magnitude1(float real,float imag)
{
	return sqrt(real*real + imag*imag);
}

//Configuring Timer for Triggering ADC

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define BUFFER_SIZE 1024
#define PI 3.14159265359f

uint16_t adc_buffer[BUFFER_SIZE];

void fill_adc_buffer_with_sine(float frequency, float amplitude, float fs)
{
    for(int i = 0; i < BUFFER_SIZE; i++)
    {
        float t = (float)i / fs;
        float value = amplitude * sinf(2.0f * PI * frequency * t);

        //Shift to unsigned 12-bit range (0 to 4095)
        adc_buffer[i] = (uint16_t)(2048.0f+ 2048.0F*value);
    }
}


void TIM2_Init()
{
	//Enablin TIM2 clock
	RCC->APB1ENR|=(1U<<0);

	//2.Configure prescalar and auto-reload for desired Sample Rate

	//Eg: 16KHz : timer freq= 45Mhz /(PSCC+1)/(ARR+1) ~ 8kHz

	TIM2->PSC=2;//PSC+1=3
	TIM2->ARR=1874;//Auto reload =1874 so that ARR+1=1875
	TIM2->CR2|=(1U<<5);//MMS=010 to update event as TRGO (trigger output)
	TIM2->EGR=1;//generate update event immediately
	//Timer Output Trigger (TRGO) is ready to trigger ADC Conversion on every event update
    //TIM2->CR1|=1;//enable the counter ----> this is set later after init
}

void ADC_Init()
{
	// Enable GPIOA clock
	RCC->AHB1ENR |= (1U << 0);//GPIOAEN

	// Set PA0 to analog mode (MODER = 11)
	GPIOA->MODER |= (3U << (0 * 2));//Analog mode for PA0

	//Enable ADC1 clock
	RCC->APB2ENR|=(1U<<8);

	ADC1->SQR3 &=~(1U<<0);//Set first conversion channel to Channel 0
	ADC1->SQR3 &=~(1U<<1);
	ADC1->SQR3 &=~(1U<<2);
	ADC1->SQR3 &=~(1U<<3);
	ADC1->SQR3 &=~(1U<<4);

	ADC1->SMPR2|=(7U<<0);//to set sampling time for channel 0 to 480 cycles (max) represented by 111



	//Timer2 TRGO as external trigger
	ADC1->CR2|=(1U<<26);//0110 for EXTSEL
	ADC1->CR2|=(1U<<25);
	ADC1->CR2&=~(1U<<27);
	ADC1->CR2&=~(1U<<24);

	// Enable external trigger on rising edge (EXTEN = 01)
	ADC1->CR2 &= ~(3U << 28); // clear EXTEN bits
	ADC1->CR2 |=  (1U << 28); // set EXTEN = 01 (rising edge)



}

void Delay_ms(uint32_t milliseconds)
{
	//enabling clock control for Timer3
	RCC->APB1ENR|=(1U<<1);
	TIM3->CR1=0;//stop the timer
	TIM3->CNT=0;//reset the counter
	TIM3->PSC=47999;
	TIM3->ARR=milliseconds-1;
	TIM3->CR1|=1;//enabling the counter
	while(!(TIM3->SR & (1U<<0))); //waits for update flag
	TIM3->CR1&=~1;      //stoppin the timer
	TIM3->SR &=~(1U<<0); //Clearing the flag
}
#define ADC_CR2_SWSTART  (1U << 30)    // SWSTART bit in ADC_CR2 (STM32F4)
#define ADC_SR_EOC       (1U << 1)

void Capture_Samples_swstart(void)
{
    // Make sure ADC is powered on
    ADC1->CR2 |= (1U<<0);//ADCON
    Delay_us(100);

    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        ADC1->SR &= ~ADC_SR_EOC;//clear flags
        ADC1->CR2 |= ADC_CR2_SWSTART;//software start
        //wait for end of conversion
        while (!(ADC1->SR & ADC_SR_EOC)) {}
        adc_buffer[i] = ADC1->DR;
    }

    //turn ADC off
    ADC1->CR2 &= ~(1U<<0);
}

void Delay_us(uint32_t microseconds)
{
	//enabling clock control for Timer3
	RCC->APB1ENR|=(1U<<1);
	TIM3->CR1=0;//stop the timer
	TIM3->CNT=0;//reset the counter
	TIM3->PSC=47;
	TIM3->ARR=microseconds-1;
	TIM3->CR1|=1;//enabling the counter
	while(!(TIM3->SR & (1U<<0))); //waits for update flag
	TIM3->CR1&=~1;      //stoppin the timer
	TIM3->SR &=~(1U<<0); //Clearing the flag
}


void Button_Init(void)
{
	// Enable clocks for GPIO A (PA5 LED) and GPIO C (PC13 button)
		    RCC->AHB1ENR |= (1U << 0);
		    RCC->AHB1ENR |= (1U << 2);

		    // Set PA5 as LED: output mode, push-pull, low speed
		    GPIOA->MODER |= (1U << 10);
		    GPIOA->MODER &= ~(1U << 11);
		    GPIOA->OTYPER &= ~(1U << 5);

		    // Set PC13 as input with pull-up
		    GPIOC->MODER &= ~(1U << 26);
		    GPIOC->MODER &= ~(1U << 27);
		    GPIOC->OSPEEDR |= (1U << 27);//fast speed
		    GPIOC->PUPDR |= (1U << 26);//pull-up
		    GPIOC->PUPDR &= ~(1U << 27);


		    // Enable SYSCFG clock
		    RCC->APB2ENR |= (1U << 14);

		    // Map EXTI13 line to Port C
		    SYSCFG->EXTICR[3] &= ~(0xF << 4);
		    SYSCFG->EXTICR[3] |=  (0x2 << 4);// 0x2 = Port C

		    // Unmask EXTI line 13
		    EXTI->IMR |= (1U << 13);

		    // Configuring falling edge trigger (button press)
		    EXTI->FTSR |= (1U << 13);
		    EXTI->RTSR &= ~(1U << 13);//disabling rising edge


		    //IRQ configurations

		    GPIO_IRQPriorityConfig(40, 15);
		    GPIO_IRQConfig(40,1);

}
int main(void)
{
		SCB->CPACR |= (0xF << 20);
		__DSB();
		__ISB();
		Button_Init();//config button+led+exti
		TIM2_Init();//COnfigure timer(ready to start)
		ADC_Init();//configure ADC(ready but idle)
		lcd_init();



		while(1)
		{
			if(start_sampling)
			{
				//float test_freq = 245.8f;// 50 Hz sine wave
				//float test_amp = 0.5f;// Max amplitude
				//float fs = 8000.0f;// Sampling frequency (8 kHz)

				//fill_adc_buffer_with_sine(test_freq, test_amp, fs);
				Start_Conversion();
				Capture_Samples_swstart();//Fill ADC_BUFFER[]
				Stop_Conversion();
				process_buffer();//Run FFT+ display result
				start_sampling=0;

			}
		}
		return 0;
}
void Capture_Samples(void)
{
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        // Wait for End of Conversion
        while (!(ADC1->SR & (1U << 1)));//EOC bit = 1 when ready
        adc_buffer[i] = ADC1->DR;//Read converted value
    }
}

void process_buffer(void)
{
	//Convert ADC samples (uint16_t) to float and apply Hanning window
	for(int i=0;i<BUFFER_SIZE;i++)
	{
		windowed_buffer[i]=((float)adc_buffer[i])*(0.5f-0.5f*arm_cos_f32(2*PI*i/(BUFFER_SIZE-1)));
	}

	//Perform FFT using CMSIS DSP

	arm_rfft_fast_init_f32(&fft_inst, BUFFER_SIZE);
	arm_rfft_fast_f32(&fft_inst, windowed_buffer,fft_real,0);
	magnitude[0]=fabsf(fft_real[0]);
	magnitude[(BUFFER_SIZE/2)-1]=fabsf(fft_real[1]);

	//Compute magnitudes
	for(int i=0;i<(BUFFER_SIZE/2);i++)
	{
		float re=fft_real[2*i];
		float im=fft_real[2*i+1];
		magnitude[i]=sqrtf((re*re)+(im*im));
	}

	for(int i=5;i<BUFFER_SIZE/2;i++)
	{
		if(magnitude[i]>max_val)
		{
			max_val=magnitude[i];
			peak_index=i;
		}
	}

	//Calculate Frequency
	float detected_freq=(float)peak_index*((float)SAMPLING_RATE/(float)BUFFER_SIZE);

	//Compare with known string frequencies
	float min_diff =16ef;
	int closest =0;
	float closest_harmonic=0;
	float min_diff=fabsf(detected_freq - note_freq[0]);
	for(int j=1;j<6;j++)
	{ for (int h=1; h<8; h++){ //checking upto 8 harmonics of that note
		float harmonic = note_freq[j] *h;
		if (harmonic >1300.00f) break;  //dont check harmonics after 1.3kHz
		float diff=fabsf(detected_freq - harmonic);
		if(diff<min_diff)
		{
			min_diff=diff;
			closest=j;
			closest_harmonic = harmonic; //closest harmonic to the detected frequency
		}
	  }
	}

	float freq_diff = detected_freq - closest_harmonic;

	const char *direction;
	if (fabsf(min_diff) < 0.5f)
	   direction = "In tune";
	else if (freq_diff > 0.5f)
	   direction = "Higher";
	else
	   direction = "Lower";
	char line1[20];
	char line2[20];
	snprintf(line1, sizeof(line1), "Note:%s F:%.0fHz", note_freq[closest] , detected_freq);

	  if (fabsf(min_diff) < 0.5f)
	        snprintf(line2, sizeof(line2), "Diff:+/-%.1fHz", fabsf(freq_diff));
	  else
	        snprintf(line2, sizeof(line2), "Diff:%.1fHz %s", fabsf(freq_diff), (freq_diff > 0) ? "HIGH" : "LOW");

	    //Display on LCD
	    lcd_display_clear();
	    lcd_send_command(0x28);
	    Delay_us(50);
	    lcd_set_cursor(1,1);
	    Delay_us(50);
	    lcd_print_string(line1);

	    lcd_set_cursor(2,1);
	    Delay_us(50);
	    lcd_print_string(line2);

}


void Start_Conversion(void)
{
    sample_index = 0;
    // Enabling ADC and Timer2 in sync
    ADC1->CR2 |= (1U << 0);//ADC ON
    Delay_us(100);
    TIM2->CR1 |= (1U << 0);//Start timer counter

    //Timer2 TRGO pulses start ADC conversions automatically
}
void Stop_Conversion(void)
{
	TIM2->CR1 &=~(1U<<0);//Stop Timer
	ADC1->CR2 &=~(1U<<0);//Disable ADC
}


void EXTI15_10_IRQHandler(void)
{
	 Delay_ms(50);


    if (EXTI->PR & (1 << GPIO_PIN_NO_13))

    {
        EXTI->PR |= (1 << GPIO_PIN_NO_13);//clear pending bit
        GPIOA->ODR^=(1U<<5);//toggle LED (not reqd for project)
        start_sampling=1;//signal the main loop to start ADC
    }

}



