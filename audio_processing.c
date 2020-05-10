#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
//#include <arm_math.h>
#include <detector.h>
#include <stdbool.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

//used to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//used to send the FFT to the computer
#define DOUBLE_BUFFERING


//send_tab is used to save the state of the buffer to send (double buffering)
//to avoid modifications of the buffer while sending it
static float send_tab[FFT_SIZE];

//Global variable to check if the robot is receiving audio data
static bool micro = OFF;

void sound_remote(float* data){

	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){

		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//Movement activated
	//When a frequency in the imposed band is detected, the variable micro is ON which means that
	//the detection of sound is done
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		micro = ON;
	}
	//stop moving
	else{
		move_guidance(STOP);
	}
}

/*
*	Callback called when the demodulation of the left microphone is done.
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part

		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else{
		return NULL;
	}
}

static THD_WORKING_AREA(waProcessAudio, MEMORY);


static THD_AUDIO_PROCESSING(ProcessAudio, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

 	 #ifdef SEND_FROM_MIC

     	 //starts the microphone processing thread.
     	 //it calls the callback given in parameter when samples are ready
     	 mic_start(&processAudioData);

	#endif  /* SEND_FROM_MIC */

     //We start by checking if the robot detected the required frequency to move
     //micro == OFF means that it is not detected yet as this variable is not activated yet
     //If this is the case the robot starts the detection of any sound until the required one is detected
     while (micro == OFF) {

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

    	 		 doFFT_optimized(FFT_SIZE, bufferCmplxInput);

    	 		 arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

    	 		 SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
    	 	 }
		#endif  /* SEND_FROM_MIC */
    }

    //Once the required frequency for the robot to move is detected (micro == ON)
    //this loop is generated to handle the movement of the robot
    while(1){
    	obstacle();
    }
}

void process_audio_start(void){
	chThdCreateStatic(waProcessAudio, sizeof(waProcessAudio), NORMALPRIO, ProcessAudio, NULL);
}
