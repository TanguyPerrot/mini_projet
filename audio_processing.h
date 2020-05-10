#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 			1024

#define MIN_VALUE_THRESHOLD	10000 	//No noise is detected over 10'000

#define MIN_FREQ			20		//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD		35		//550Hz
#define MAX_FREQ			50  	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-3)
#define FREQ_FORWARD_H		(FREQ_FORWARD+3) //Frequency around ~[500Hz-600Hz]

#define OFF					0
#define ON 					1
#define MEMORY 				800 	//The needed memory space (in bits) for the audio processing thread

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
} BUFFER_NAME_t;

void processAudioData(int16_t *data, uint16_t num_samples);

//put the invoking thread into sleep until it can process the audio datas
void wait_send_to_computer(void);

//Returns the pointer to the BUFFER_NAME_t buffer asked
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

void process_audio_start(void);

#endif /* AUDIO_PROCESSING_H */
