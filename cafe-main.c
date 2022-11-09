#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// AV2
#include "coffee/coffee.h"

#define AFEC AFEC0
#define AFEC_ID ID_AFEC0
#define AFEC_CHANNEL 0 // Canal do pino PD30

#define BUT_PIO_OLED1 PIOD
#define BUT_PIO_OLED1_ID ID_PIOD
#define BUT_PIO_OLED1_IDX 28
#define BUT_PIO_OLED1_IDX_MASK (1 << BUT_PIO_OLED1_IDX)

#define BUT_PIO_OLED2 PIOC
#define BUT_PIO_OLED2_ID ID_PIOC
#define BUT_PIO_OLED2_IDX 31
#define BUT_PIO_OLED2_IDX_MASK (1 << BUT_PIO_OLED2_IDX)

// LED 1 (OLED)
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_IDX_MASK (1 << LED1_PIO_IDX)

// LED 2 (OLED)
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_IDX_MASK (1 << LED2_PIO_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)
TimerHandle_t xTimer;
QueueHandle_t xQueueTemp;
SemaphoreHandle_t xSemaphoreCoffeSimple;
SemaphoreHandle_t xSemaphoreCoffeDouble;
SemaphoreHandle_t xSemaphoreMakeCoffe;

/************************************************************************/
/* glboals                                                               */
/************************************************************************/

/************************************************************************/
/* PROTOtypes                                                           */
/************************************************************************/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void config_AFEC(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
static void AFEC_callback(void) {
	uint32_t temp_lida = afec_channel_get_value(AFEC, AFEC_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueTemp, &temp_lida, &xHigherPriorityTaskWoken);
}

void but_oled1_callback(void){
	xSemaphoreGiveFromISR(xSemaphoreCoffeSimple, 0);
}

void but_oled2_callback(void){
	xSemaphoreGiveFromISR(xSemaphoreCoffeDouble, 0);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		coffe_pump_off();
		pio_set(LED1_PIO, LED1_IDX_MASK);
		pio_set(LED2_PIO, LED2_IDX_MASK);
		xSemaphoreGiveFromISR(xSemaphoreMakeCoffe, 0);
		gfx_mono_draw_filled_rect(0,0, 128, 32, GFX_PIXEL_CLR);
		gfx_mono_draw_string("Pronto", 0, 0, &sysfont);
	}
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void vTimerCallback(TimerHandle_t xTimer) {
	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC, AFEC_CHANNEL);
	afec_start_software_conversion(AFEC);
}

static void task_oled(void *pvParameters) {
	config_AFEC(AFEC, AFEC_ID, AFEC_CHANNEL, AFEC_callback);
	
	
    xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
                          kernel. */
                          "Timer",
                          /* The timer period in ticks, must be
                          greater than 0. */
                          100,
                          /* The timers will auto-reload themselves
                          when they expire. */
                          pdTRUE,
                          /* The ID is used to store a count of the
                          number of times the timer has expired, which
                          is initialised to 0. */
                          (void *)0,
                          /* Timer callback */
                          vTimerCallback);
    xTimerStart(xTimer, 0);
	
	gfx_mono_ssd1306_init();
	
	pio_set(LED1_PIO, LED1_IDX_MASK);
	pio_set(LED2_PIO, LED2_IDX_MASK);
	
	uint32_t temp_lida = 0; 
	uint32_t temp_celsius = 0;
	uint32_t temp_ant = 0;
	char maquina_livre = 1;
	char aquecedor = 1;
	for (;;)  {
		 if (xQueueReceive(xQueueTemp, &(temp_lida), 0)){
			 temp_celsius = temp_lida*100/4095;
		 }
		  if((temp_celsius<80 && aquecedor || temp_ant == 0) && maquina_livre){
			  coffee_heat_on();
			  gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			  gfx_mono_draw_string("Aquecendo", 0, 0, &sysfont);
			  maquina_livre = 0;
			  aquecedor = 0;
			  }else if(temp_celsius>=80 && temp_ant<80){
			  gfx_mono_draw_filled_rect(0,0, 128, 32, GFX_PIXEL_CLR);
			  gfx_mono_draw_string("Pronto", 0, 0, &sysfont);
			  maquina_livre = 1;
		  }
		  if (xSemaphoreTake(xSemaphoreCoffeSimple, 0) && maquina_livre && temp_celsius>=80){
			  gfx_mono_draw_filled_rect(0,0, 128, 32, GFX_PIXEL_CLR);
			  gfx_mono_draw_string("Simples", 0, 0, &sysfont);
			  pio_clear(LED1_PIO, LED1_IDX_MASK);
			  coffe_pump_on();
			  RTT_init(1, 5, RTT_MR_ALMIEN);
			  maquina_livre = 0;
		  }
		  if(xSemaphoreTake(xSemaphoreCoffeDouble, 0) && maquina_livre && temp_celsius>=80){
			  gfx_mono_draw_filled_rect(0,0, 128, 32, GFX_PIXEL_CLR);
			  gfx_mono_draw_string("Duplo", 0, 0, &sysfont);
			  pio_clear(LED2_PIO, LED2_IDX_MASK);
			  coffe_pump_on();
			  RTT_init(1, 10, RTT_MR_ALMIEN);
			  maquina_livre = 0;
		  }
		  temp_ant = temp_celsius;
		  if (xSemaphoreTake(xSemaphoreMakeCoffe, 0)){
			  maquina_livre = 1;
			  aquecedor = 1;
		  }
		 
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void config_AFEC(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
    /*************************************
     * Ativa e configura AFEC
     *************************************/
    /* Ativa AFEC - 0 */
    afec_enable(afec);

    /* struct de configuracao do AFEC */
    struct afec_config afec_cfg;

    /* Carrega parametros padrao */
    afec_get_config_defaults(&afec_cfg);

    /* Configura AFEC */
    afec_init(afec, &afec_cfg);

    /* Configura trigger por software */
    afec_set_trigger(afec, AFEC_TRIG_SW);

    /*** Configuracao específica do canal AFEC ***/
    struct afec_ch_config afec_ch_cfg;
    afec_ch_get_config_defaults(&afec_ch_cfg);
    afec_ch_cfg.gain = AFEC_GAINVALUE_0;
    afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

    /*
    * Calibracao:
    * Because the internal ADC offset is 0x200, it should cancel it and shift
    down to 0.
    */
    afec_channel_set_analog_offset(afec, afec_channel, 0x200);

    /***  Configura sensor de temperatura ***/
    struct afec_temp_sensor_config afec_temp_sensor_cfg;

    afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
    afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

    /* configura IRQ */
    afec_set_callback(afec, afec_channel, callback, 1);
    NVIC_SetPriority(afec_id, 4);
    NVIC_EnableIRQ(afec_id);
}

void io_init(void){
	
	 pmc_enable_periph_clk(LED1_PIO_ID);
	 pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
	 pmc_enable_periph_clk(LED2_PIO_ID);
	 pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
	 
	 pmc_enable_periph_clk(BUT_PIO_OLED1);
	 pmc_enable_periph_clk(BUT_PIO_OLED2);
	 
	 pio_configure(BUT_PIO_OLED1, PIO_INPUT, BUT_PIO_OLED1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	 pio_configure(BUT_PIO_OLED2, PIO_INPUT, BUT_PIO_OLED2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
     
	 pio_set_debounce_filter(BUT_PIO_OLED1, BUT_PIO_OLED1_IDX_MASK, 60);
     pio_set_debounce_filter(BUT_PIO_OLED2, BUT_PIO_OLED2_IDX_MASK, 60);
	 
     pio_handler_set(BUT_PIO_OLED1,
     BUT_PIO_OLED1_ID,
     BUT_PIO_OLED1_IDX_MASK,
     PIO_IT_FALL_EDGE,
     but_oled1_callback);

     pio_handler_set(BUT_PIO_OLED2,
     BUT_PIO_OLED2_ID,
     BUT_PIO_OLED2_IDX_MASK,
     PIO_IT_FALL_EDGE,
     but_oled2_callback);
	 
	 pio_enable_interrupt(BUT_PIO_OLED1, BUT_PIO_OLED1_IDX_MASK);
	 pio_enable_interrupt(BUT_PIO_OLED2, BUT_PIO_OLED2_IDX_MASK);

     pio_get_interrupt_status(BUT_PIO_OLED1);
     pio_get_interrupt_status(BUT_PIO_OLED2);
	 
     NVIC_EnableIRQ(BUT_PIO_OLED1_ID);
     NVIC_EnableIRQ(BUT_PIO_OLED2_ID);
	
     NVIC_SetPriority(BUT_PIO_OLED1_ID, 4);
     NVIC_SetPriority(BUT_PIO_OLED2_ID, 4);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	io_init();
	xSemaphoreCoffeSimple = xSemaphoreCreateBinary();
	xSemaphoreCoffeDouble = xSemaphoreCreateBinary();
	xSemaphoreMakeCoffe = xSemaphoreCreateBinary();
	
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_av2, "av2", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	 xQueueTemp = xQueueCreate(100, sizeof(uint32_t));
	 if (xQueueTemp == NULL)
	 printf("falha em criar a queue xQueueADC \n");

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
