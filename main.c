#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <audio_processing.h>
#include <detector.h>
#include <sensors/proximity.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    serial_start();
    usb_start();
    motors_init();
    proximity_start();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    calibrate_ir();
    process_audio_start();

    while(1){
    	chThdSleepMilliseconds(ONE_SEC);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
