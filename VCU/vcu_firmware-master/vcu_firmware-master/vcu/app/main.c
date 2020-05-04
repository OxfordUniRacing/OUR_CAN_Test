#include "atmel_start.h"
#include "hpl_pmc.h"

#include "driver/usb/usb.h"
#include "driver/usb/usb_drive.h"
#include "driver/usb/usb_console.h"
#include "driver/log_msg.h"
#include "driver/adc.h"
#include "driver/eeprom_emu.h"
#include "framework/sensor/sensor.h"
#include "framework/sensor/sensor_db.h"
#include "app/motor_controller.h"
#include "app/analog_poll.h"


static void CAN_0_tx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8);
	(void)descr;
}


static void CAN_0_rx_callback(struct can_async_descriptor *const descr) {
	struct can_message msg;
	uint8_t data[8];
	msg.data = data;
	can_async_read(descr, &msg);
	log_info("Here: %d",msg.data);
}

extern uint8_t *can0_rx_fifo;
static void setup_can(void) {

	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, CAN_0_rx_callback);
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, CAN_0_tx_callback);
	int32_t status = can_async_enable(&CAN_0);
	
	// get out of stand by mode
	// IMPORTANT: PIN_PD11 may put can chip in stand by mode
	gpio_set_pin_direction(PIN_PD11, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PD11, 0);

	log_debug("enable status: %d", status);
	struct can_filter filter;
	filter.id   = 0;
	filter.mask = 0;
	status = can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
	log_debug("filter status: %d", status);
}



//vehicle startup test
static void start_up_test(void){
	// call all Units to make sure they are online
	
	
}

static void do_can_tx(void) {
	static uint32_t counter;

	struct can_message msg;
	uint32_t data = __builtin_bswap32(counter);
	//uint8_t data[8];
	//data[0] = counter + 42;

	msg.id   = 0x246;
	msg.type = CAN_TYPE_DATA;
	msg.data = &data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	
	int32_t status = can_async_write(&CAN_0, &msg);
	log_debug("message_status: %d", status);
	
	counter++;
}

static void can_task(void *p) {
	(void)p;
	
	log_debug("start can_test");
	setup_can();
	log_debug("setup finished");

	//hri_mcan_set_CCCR_TEST_bit(MCAN0);
	//hri_mcan_set_TEST_LBCK_bit(MCAN0);
	
	
	while (1) {
		do_can_tx();
		for (int i = 0; i < 1000000; i++);
	}
}

static void test_task(void *p) {
	(void)p;

	while (1) {
		vTaskDelay(789);
		float val = sensor_get_f(SENS_PRES_R_OUT);
		uint32_t ts = sensor_get_ts(SENS_PRES_R_OUT);
		log_info("%d %f", ts, val);
		
	}
}

int main(void) {
	// TODO don't hardcode address
	hri_matrix_write_CCFG_CAN0_CAN0DMABA_bf(MATRIX, (uint32_t)0x2042);

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	NVIC_SetPriority(HSMCI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(USBHS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(MCAN0_INT0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(MCAN0_INT1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(UART1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(UART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(AFEC0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_SetPriority(AFEC1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

	_pmc_enable_periph_clock(ID_PIOB);

	gpio_set_pin_direction(PIN_PC8, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PC8, 0);
	
	log_init();
	log_debug("startup");

	eeprom_emu_init();

	mc_init();
	analog_poll_init();
	
	usb_drive_init();
	usb_console_init();
	usb_start();
	
	//----
	
	//int pin = PIN_PB3;

	//gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
	//gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
	//gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_OFF);

	//while (1) {
	//	int x = gpio_get_pin_level(pin);
	//	gpio_set_pin_level(PIN_PC8, x);
	//}
	
	//----
	
	
	xTaskCreate(can_task, "can_task", 1024, NULL, 1, NULL);

	vTaskStartScheduler();

	while (1);
}
