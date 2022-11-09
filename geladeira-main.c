#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "sensor.h"
#include "string.h"

/* Botao da placa */
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

/** RTOS  */
#define TASK_MAIN_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_COMPRESS_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_COMPRESS_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

typedef struct {
	char modo_name[13];
	int tempo_ligado;
  int tempo_desligado;
} modo;

typedef struct {
	modo modos[5];
	int selected;
	int size;
} list_modo;

/** Queue for msg log send data */
QueueHandle_t xQueueLED;
SemaphoreHandle_t xSemaphore;

/** prototypes */
void io_init(void);
void apaga_tela();
void modos();

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
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
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
  // libera semáforo
  xSemaphoreGiveFromISR(xSemaphore, 0);
 }

void but2_callback(void) { }

void but3_callback(void) { }

/************************************************************************/
/* Funções                                                              */
/************************************************************************/

void apaga_tela() {
	gfx_mono_draw_filled_rect(0, 0, 120, 30, GFX_PIXEL_CLR);
}

void mostra_modo(modo m) {
  apaga_tela();
  // int start_digit = (13 - strlen(m.modo_name)) * 11 / 2;
	gfx_mono_draw_string(m.modo_name, 0, 12, &sysfont);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_main(void *pvParameters) {
	gfx_mono_ssd1306_init();
  io_init();
  pio_set(LED_1_PIO, LED_1_IDX_MASK);
  pio_set(LED_2_PIO, LED_2_IDX_MASK);
  pio_set(LED_3_PIO, LED_3_IDX_MASK);

  modo normal, eco, turbo, festa;

  normal.tempo_ligado = 3;
  normal.tempo_desligado = 3;
  strcpy(normal.modo_name, "Normal");

  eco.tempo_ligado = 1;
  eco.tempo_desligado = 5;
  strcpy(eco.modo_name, "Eco");

  turbo.tempo_ligado = 5;
  turbo.tempo_desligado = 3;
  strcpy(turbo.modo_name, "Turbo");

  festa.tempo_ligado = 5;
  festa.tempo_desligado = 1;
  strcpy(festa.modo_name, "Festa");

  list_modo list_modo;

  list_modo.modos[0] = normal;
  list_modo.modos[1] = eco;
  list_modo.modos[2] = turbo;
  list_modo.modos[3] = festa;
  list_modo.size = 4;
  list_modo.selected = 0;

  mostra_modo(list_modo.modos[0]);
  xQueueSend(xQueueLED, &(list_modo.modos[list_modo.selected]), 10);
  uint32_t n;
	for (;;)  {
    if ( xSemaphoreTake(xSemaphore, 0) == pdTRUE ) {
      list_modo.selected++;
      if (list_modo.selected >= list_modo.size) {
        list_modo.selected = 0;
      }
	  modo m = list_modo.modos[list_modo.selected];
    mostra_modo(m);
    xQueueSend(xQueueLED, &(m), 0);
    }
	}
}

static void task_compressor(void *pvParameters) { 
  int ligado= 3, desligado= 3;
  for (;;) {
	modo mi;
    if (xQueueReceive(xQueueLED, &(mi), 10)) {
      ligado = mi.tempo_ligado;
      desligado = mi.tempo_desligado;
    } else {
      pio_clear(LED_1_PIO, LED_1_IDX_MASK);
      vTaskDelay(ligado * 100 / portTICK_PERIOD_MS);
      pio_set(LED_1_PIO, LED_1_IDX_MASK);
      vTaskDelay(desligado * 100 / portTICK_PERIOD_MS);
      printf("erro");
    }
  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {
  pmc_enable_periph_clk(LED_1_PIO_ID);
  pmc_enable_periph_clk(LED_2_PIO_ID);
  pmc_enable_periph_clk(LED_3_PIO_ID);
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);

  pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
  but1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but3_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but2_callback);

  pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
  pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
  pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

  pio_get_interrupt_status(BUT_1_PIO);
  pio_get_interrupt_status(BUT_2_PIO);
  pio_get_interrupt_status(BUT_3_PIO);

  NVIC_EnableIRQ(BUT_1_PIO_ID);
  NVIC_SetPriority(BUT_1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_2_PIO_ID);
  NVIC_SetPriority(BUT_2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_3_PIO_ID);
  NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

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

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
  board_init();
  sysclk_init();
  delay_init();

	/* Initialize the console uart */
	configure_console();

  xQueueLED = xQueueCreate(32, sizeof(modo));
  if (xQueueLED == NULL)
    printf("falha em criar a queue xQueueLED \n");

  // cria semáforo binário
  xSemaphore = xSemaphoreCreateBinary();

  // verifica se semáforo foi criado corretamente
  if (xSemaphore == NULL)
      printf("falha em criar o semaforo \n");
  
	/* Create task to control oled */
	if (xTaskCreate(task_main, "main", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create main task\r\n");
	}

  if (xTaskCreate(task_compressor, "compressor", TASK_COMPRESS_STACK_SIZE, NULL, TASK_COMPRESS_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create compressor task\r\n");
	}
  
	/* Start the scheduler. */
	vTaskStartScheduler();
  
  /* RTOS n�o deve chegar aqui !! */
	while(1){
  }

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
