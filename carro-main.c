#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define IN1_PIO PIOD
#define IN1_PIO_ID ID_PIOD
#define IN1_PIO_IDX 30
#define IN1_IDX_MASK (1 << IN1_PIO_IDX)

#define IN2_PIO PIOA
#define IN2_PIO_ID ID_PIOA
#define IN2_PIO_IDX 6
#define IN2_IDX_MASK (1 << IN2_PIO_IDX)

#define HEADLIGHT_PIO PIOC
#define HEADLIGHT_PIO_ID ID_PIOC
#define HEADLIGHT_PIO_IDX 13
#define HEADLIGHT_IDX_MASK (1 << HEADLIGHT_PIO_IDX)

#define BACKLIGHT_PIO PIOC
#define BACKLIGHT_PIO_ID ID_PIOC
#define BACKLIGHT_PIO_IDX 19
#define BACKLIGHT_IDX_MASK (1 << BACKLIGHT_PIO_IDX)

#define SERVO_PIO PIOD
#define SERVO_PIO_ID ID_PIOD
#define SERVO_PIO_IDX 11
#define SERVO_IDX_MASK (1 << SERVO_PIO_IDX)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

/** PWM frequency in Hz */
#define PWM_FREQUENCY      50
/** Period value of PWM output waveform */
#define PERIOD_VALUE       250
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    1

pwm_channel_t g_pwm_channel_servo;

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/
#define TASK_BLUETOOTH_STACK_SIZE (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY (tskIDLE_PRIORITY)
/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
void PWM0_init(uint channel, uint duty);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(){
	pmc_enable_periph_clk(IN1_PIO_ID);
	pio_set_output(IN1_PIO, IN1_IDX_MASK, 0, 0, 0);
	pmc_enable_periph_clk(IN2_PIO_ID);
	pio_set_output(IN2_PIO, IN2_IDX_MASK, 0, 0, 0);
	pmc_enable_periph_clk(HEADLIGHT_PIO_ID);
	pio_set_output(HEADLIGHT_PIO, HEADLIGHT_IDX_MASK, 0, 0, 0);
	pmc_enable_periph_clk(BACKLIGHT_PIO_ID);
	pio_set_output(BACKLIGHT_PIO, BACKLIGHT_IDX_MASK, 0, 0, 0);
	pmc_enable_periph_clk(SERVO_PIO_ID);
	pio_set_output(SERVO_PIO, SERVO_IDX_MASK, 0, 0, 0);
	pio_set_peripheral(SERVO_PIO, PIO_PERIPH_B, SERVO_IDX_MASK);
	PWM0_init(0, 7);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_servo.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_servo.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_servo.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_servo.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_servo.ul_duty = duty;
	g_pwm_channel_servo.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_servo);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
void task_bluetooth(void) {
	printf("Carro Inicializado\n");
	config_usart0();
	char way = 1;
	char readChar;
	char readBluetooth[10];
	uint8_t i = 0;
	uint16_t clocks = 0;
	char command = '\0';
	char connected = 0;
	for(;;) {
		while(usart_is_rx_ready(USART_COM)) {
			int status = usart_read(USART_COM, &readChar);	
			//printf("lido: %c\n", readChar);		
			if (!connected){
				if (readChar=='L'){
					while(!usart_is_tx_ready(USART_COM)) {
						vTaskDelay(10 / portTICK_PERIOD_MS);
					}
					usart_write(USART_COM, 'C');
					connected = 1;
					printf("Conectado\n");
					pio_set(HEADLIGHT_PIO,HEADLIGHT_IDX_MASK);
					pio_set(BACKLIGHT_PIO,BACKLIGHT_IDX_MASK);
					vTaskDelay(300/ portTICK_PERIOD_MS);
					pio_clear(HEADLIGHT_PIO,HEADLIGHT_IDX_MASK);
					pio_clear(BACKLIGHT_PIO,BACKLIGHT_IDX_MASK);
					vTaskDelay(300/ portTICK_PERIOD_MS);
				}
			}
			else{
				readBluetooth[i]= readChar;
				i++;
			 }
		}					

		if( readBluetooth[i-1]=='/' && connected){
			printf("%c\n", readBluetooth[0]);
			if(readBluetooth[0] == 'H'){
				pio_set(HEADLIGHT_PIO, HEADLIGHT_IDX_MASK);
				printf("Ligou head\n");
			}
			else if(readBluetooth[0] == 'h'){
				pio_clear(HEADLIGHT_PIO, HEADLIGHT_IDX_MASK);
				printf("Desligou head\n");
			}
			else if(readBluetooth[0] == 'B'){
				pio_set(BACKLIGHT_PIO, BACKLIGHT_IDX_MASK);
				printf("Ligou back\n");
			}
			else if(readBluetooth[0] == 'b'){
				pio_clear(BACKLIGHT_PIO, BACKLIGHT_IDX_MASK);
				printf("Desligou back\n");
			}
			else if(readBluetooth[0]=='W'){
				way = 1;
			}
			else if(readBluetooth[0] == 'w'){
				way = 0;				
			}else{
				readBluetooth[i-1] = '\0';
				command = readBluetooth[0];
				for(int j = 1; j<i; j++){
					readBluetooth[j-1] = readBluetooth[j];
				}
				int value = atoi(readBluetooth);
				if(command == 'd'){
				    clocks = 6 + value/10;
					uint16_t duty = clocks*100/200;
					pwm_channel_update_duty(PWM0, &g_pwm_channel_servo,duty);
				}
				if(command == 'v'){
					if (value == 0){
						pio_clear(IN1_PIO, IN1_IDX_MASK);
						pio_clear(IN2_PIO, IN2_IDX_MASK);
					}else if(way){
						pio_set(IN1_PIO, IN1_IDX_MASK);
						pio_clear(IN2_PIO, IN2_IDX_MASK);
					}else{
						pio_clear(IN1_PIO, IN1_IDX_MASK);
						pio_set(IN2_PIO, IN2_IDX_MASK);
					}
				}
			}
			memset( readBluetooth, 0, i);
			i = 0;
		}
	}
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();
	configure_console();
	printf("Sys init ok \n"); 
	 
	 /* Create task to receive and send data to bluetooth */
	 if (xTaskCreate(task_bluetooth, "Bluetooth", TASK_BLUETOOTH_STACK_SIZE, NULL,
	 TASK_BLUETOOTH_STACK_PRIORITY, NULL) != pdPASS) {
		 printf("Failed to create bluetooth task\r\n");
		 } else {
		 printf("task bluetooth created \r\n");
	 }

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
