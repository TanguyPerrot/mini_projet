#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <leds.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <sensors/proximity.h>


//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

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

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{
	int var0 = 0;
	int var1 = 0;
	int var2 = 0;
	int var5 = 0;
	int var6 = 0;
	int var7 = 0;

	volatile uint16_t time = 0;
    halInit();
    chSysInit();
    mpu_init();



    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    proximity_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    calibrate_ir();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];
    static complex_float temp_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){




        	for(uint16_t i=0 ; i<(2*FFT_SIZE) ; i+=2){
        		temp_tab[i/2].real = bufferCmplxInput[i];
        		temp_tab[i/2].imag = bufferCmplxInput[i+1];
        	}
        	//chSysLock();
        	//GPTD12.tim->CNT = 0;

        	//doFFT_c(FFT_SIZE, temp_tab);
        	doFFT_optimized(FFT_SIZE, bufferCmplxInput);

        	//time = GPTD12.tim->CNT;
        	//chSysUnlock();

        	//chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n" , time);


        	for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
        		bufferCmplxInput[i] = temp_tab[i/2].real;
        		bufferCmplxInput[i+1] = temp_tab[i/2].imag;
        	}
            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

        }
#endif  /* SEND_FROM_MIC */



        var0 = get_prox(0);
        if (var0 > 150){
        	set_led(LED1, 1);
        	}
       /* else{
        	set_led(LED5, 0);
        	}*/

        var1 = get_prox(1);
        if (var1 > 150){
        	set_led(LED3, 1);
            }
        /*else{
        	set_body_led(0);
            }*/

        var2 = get_prox(2);
        if (var2 > 150){
        	set_led(LED5, 1);
        	}
       /* else{
        	set_led(LED3, 0);
        	}*/

        var5 = get_prox(5);
        if (var5 > 150){
              	set_led(LED7, 1);
            }
        /*else{
        	set_led(LED7, 0);
            }*/

        var6 = get_prox(6);
        if (var6 > 150){
        	set_body_led(1);
        	}
        /*else{
        	set_led(LED8, 0);
        	}*/

        var7 = get_prox(7);
        if (var7 > 150){
        	clear_leds();
        	set_body_led(0);
            }
        /*else{
        	set_front_led(0);
            }*/
    }

}
//chprintf((BaseSequentialStream *)&SD3, "%d\r\n", var);
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
