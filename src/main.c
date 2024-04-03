#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#define LOOP_TIMEOUT_MS 2000 // 10 seconds

// Declaration
void my_expiry_function(struct k_timer *timer_id);
void my_timer_stop_function(struct k_timer *timer_id);

K_TIMER_DEFINE(my_timer, my_expiry_function, my_timer_stop_function);

// Global flag to indicate if the loop should continue or not
bool run_loop = true;



#define WINDOWSIZE 620 // Integrator window size, in samples. The article recommends 150ms. So, FS*0.15.
						// However, you should check empirically if the waveform looks ok.
#define NOSAMPLE -32000 // An indicator that there are no more samples to read. Use an impossible value for a sample.
#define FS 250         // Sampling frequency.
#define BUFFSIZE 22    // The size of the buffers (in samples). Must fit more than 1.66 times an RR interval, which
                        // typically could be around 1 second.

#define DELAY 22		// Delay introduced by the filters. Filter only output samples after this one.
						// Set to 0 if you want to keep the delay. Fixing the delay results in DELAY less samples
						// in the final end result.

#include "panTompkins.h"

void my_expiry_function(struct k_timer *timer_id) {
    printk("Timer expired\n");
    run_loop = false; // Set the flag to stop the loop
}

void my_timer_stop_function(struct k_timer *timer_id) {
    printk("Timer stopped\n");
}





typedef int dataType; // Change this to your desired data type
void delay_ms(int ms) {
    k_msleep(ms);
}
void panTompkins(const dataType *inputSignal, dataType *outputSignal, int inputSize) 
{
    dataType signal[BUFFSIZE], dcblock[BUFFSIZE], lowpass[BUFFSIZE], highpass[BUFFSIZE], derivative[BUFFSIZE], squared[BUFFSIZE], integral[BUFFSIZE] ;
    int rr1[8], rr2[8], rravg1, rravg2, rrlow = 0, rrhigh = 0, rrmiss = 0;
    long unsigned int i, j, sample = 0, lastQRS = 0, lastSlope = 0, currentSlope = 0;
    int current;
    dataType peak_i = 0, peak_f = 0, threshold_i1 = 0, threshold_i2 = 0, threshold_f1 = 0, threshold_f2 = 0, spk_i = 0, spk_f = 0, npk_i = 0, npk_f = 0;
    bool qrs, regular = true, prevRegular;
	
      
    for (i = 0; i < 8; i++)
    {
        rr1[i] = 0;
        rr2[i] = 0;
    }
	
    do
    {
        if (sample >= BUFFSIZE)
		{
			for (i = 0; i < BUFFSIZE - 1; i++)
			{
				signal[i] = signal[i+1];
				dcblock[i] = dcblock[i+1];
				lowpass[i] = lowpass[i+1];
				highpass[i] = highpass[i+1];
				derivative[i] = derivative[i+1];
				squared[i] = squared[i+1];
				integral[i] = integral[i+1];
				outputSignal[i] = outputSignal[i+1];
			}
			current = BUFFSIZE - 1;
		}
		else
		{
			current = sample;
		}
		signal[current] = inputSignal[inputSize];

		
        
		// If no sample was read, stop processing!
		if (sample > inputSize)
			break;

        sample++;
        
        // DC Block filter
		// This was not proposed on the original paper.
		// It is not necessary and can be removed if your sensor or database has no DC noise.
		if (current >= 1)
			dcblock[current] = inputSignal[current] - inputSignal[current-1] + 0.995*dcblock[current-1];
			//dcblock[current] = signal[current] - signal[current-1] + 0.995*dcblock[current-1];
		else
			dcblock[current] = 0;



		// Low Pass filter
		// Implemented as proposed by the original paper.
		// y(nT) = 2y(nT - T) - y(nT - 2T) + x(nT) - 2x(nT - 6T) + x(nT - 12T)
		// Can be removed if your signal was previously filtered, or replaced by a different filter.
		lowpass[current] = dcblock[current];
		if (current >= 1)
			lowpass[current] += 2*lowpass[current-1];
		if (current >= 2)
			lowpass[current] -= lowpass[current-2];
		if (current >= 6)
			lowpass[current] -= 2*dcblock[current-6];
		if (current >= 12)
			lowpass[current] += dcblock[current-12];

		// High Pass filter
		// Implemented as proposed by the original paper.
		// y(nT) = 32x(nT - 16T) - [y(nT - T) + x(nT) - x(nT - 32T)]
		// Can be removed if your signal was previously filtered, or replaced by a different filter.
		highpass[current] = -lowpass[current];
		if (current >= 1)
			highpass[current] -= highpass[current-1];
		if (current >= 16)
			highpass[current] += 32*lowpass[current-16];
		if (current >= 32)
			highpass[current] += lowpass[current-32];

		// Derivative filter
		// This is an alternative implementation, the central difference method.
		// f'(a) = [f(a+h) - f(a-h)]/2h
		// The original formula used by Pan-Tompkins was:
		// y(nT) = (1/8T)[-x(nT - 2T) - 2x(nT - T) + 2x(nT + T) + x(nT + 2T)]
        derivative[current] = highpass[current];
		if (current > 0)
			derivative[current] -= highpass[current-1];

		// This just squares the derivative, to get rid of negative values and emphasize high frequencies.
		// y(nT) = [x(nT)]^2.
		
		squared[current] = derivative[current]*derivative[current];
		
		
		
		// Moving-Window Integration
		// Implemented as proposed by the original paper.
		// y(nT) = (1/N)[x(nT - (N - 1)T) + x(nT - (N - 2)T) + ... x(nT)]
		// WINDOWSIZE, in samples, must be defined so that the window is ~150ms.

		integral[current] = 0;
		for (i = 0; i < WINDOWSIZE; i++)
		{
			if (current >= (dataType)i)
				integral[current] += squared[current - i];
			else
				break;
		}
		integral[current] /= (dataType)i;

		
		
		qrs = false;

		// If the current signal is above one of the thresholds (integral or filtered signal), it's a peak candidate.
        if (integral[current] >= threshold_i1 || highpass[current] >= threshold_f1)
        {
            peak_i = integral[current];
            peak_f = highpass[current];
        }

		// If both the integral and the signal are above their thresholds, they're probably signal peaks.
		if ((integral[current] >= threshold_i1) && (highpass[current] >= threshold_f1))
		{
			// There's a 200ms latency. If the new peak respects this condition, we can keep testing.
			if (sample > lastQRS + FS/5)
			{
			    // If it respects the 200ms latency, but it doesn't respect the 360ms latency, we check the slope.
				if (sample <= lastQRS + (long unsigned int)(0.36*FS))
				{
				    // The squared slope is "M" shaped. So we have to check nearby samples to make sure we're really looking
				    // at its peak value, rather than a low one.
				    currentSlope = 0;
				    for (j = current - 10; j <= current; j++)
                        if (squared[j] > currentSlope)
                            currentSlope = squared[j];

				    if (currentSlope <= (dataType)(lastSlope/2))
                    {
                        qrs = false;
                    }

                    else
                    {
                        spk_i = 0.125*peak_i + 0.875*spk_i;
                        threshold_i1 = npk_i + 0.25*(spk_i - npk_i);
                        threshold_i2 = 0.5*threshold_i1;

                        spk_f = 0.125*peak_f + 0.875*spk_f;
                        threshold_f1 = npk_f + 0.25*(spk_f - npk_f);
                        threshold_f2 = 0.5*threshold_f1;

                        lastSlope = currentSlope;
                        qrs = true;
                    }
				}
				// If it was above both thresholds and respects both latency periods, it certainly is a R peak.
				else
				{
				    currentSlope = 0;
                    for (j = current - 10; j <= current; j++)
                        if (squared[j] > currentSlope)
                            currentSlope = squared[j];

                    spk_i = 0.125*peak_i + 0.875*spk_i;
                    threshold_i1 = npk_i + 0.25*(spk_i - npk_i);
                    threshold_i2 = 0.5*threshold_i1;

                    spk_f = 0.125*peak_f + 0.875*spk_f;
                    threshold_f1 = npk_f + 0.25*(spk_f - npk_f);
                    threshold_f2 = 0.5*threshold_f1;

                    lastSlope = currentSlope;
                    qrs = true;
				}
			}
			// If the new peak doesn't respect the 200ms latency, it's noise. Update thresholds and move on to the next sample.
			else
            {
                peak_i = integral[current];
				npk_i = 0.125*peak_i + 0.875*npk_i;
				threshold_i1 = npk_i + 0.25*(spk_i - npk_i);
				threshold_i2 = 0.5*threshold_i1;
				peak_f = highpass[current];
				npk_f = 0.125*peak_f + 0.875*npk_f;
				threshold_f1 = npk_f + 0.25*(spk_f - npk_f);
                threshold_f2 = 0.5*threshold_f1;
                qrs = false;
				outputSignal[current] = qrs;
				if (sample > DELAY + BUFFSIZE)
                	outputSignal[sample]=outputSignal[current]; // fix this
                continue;
            }

		}
		
		
		// If a R-peak was detected, the RR-averages must be updated.
		if (qrs)
		{
			// Add the newest RR-interval to the buffer and get the new average.
			rravg1 = 0;
			for (i = 0; i < 7; i++)
			{
				rr1[i] = rr1[i+1];
				rravg1 += rr1[i];
			}
			rr1[7] = sample - lastQRS;
			lastQRS = sample;
			rravg1 += rr1[7];
			rravg1 *= 0.125;

			// If the newly-discovered RR-average is normal, add it to the "normal" buffer and get the new "normal" average.
			// Update the "normal" beat parameters.
			if ( (rr1[7] >= rrlow) && (rr1[7] <= rrhigh) )
			{
				rravg2 = 0;
				for (i = 0; i < 7; i++)
				{
					rr2[i] = rr2[i+1];
					rravg2 += rr2[i];
				}
				rr2[7] = rr1[7];
				rravg2 += rr2[7];
				rravg2 *= 0.125;
				rrlow = 0.92*rravg2;
				rrhigh = 1.16*rravg2;
				rrmiss = 1.66*rravg2;
			}

			prevRegular = regular;
			if (rravg1 == rravg2)
			{
				regular = true;
			}
			// If the beat had been normal but turned odd, change the thresholds.
			else
			{
				regular = false;
				if (prevRegular)
				{
					threshold_i1 /= 2;
					threshold_f1 /= 2;
				}
			}
		}
		// If no R-peak was detected, it's important to check how long it's been since the last detection.
		else
		{
		    // If no R-peak was detected for too long, use the lighter thresholds and do a back search.
			// However, the back search must respect the 200ms limit and the 360ms one (check the slope).
			if ((sample - lastQRS > (long unsigned int)rrmiss) && (sample > lastQRS + FS/5))
			{
				for (i = current - (sample - lastQRS) + FS/5; i < (long unsigned int)current; i++)
				{
					if ( (integral[i] > threshold_i2) && (highpass[i] > threshold_f2))
					{
					    currentSlope = 0;
                        for (j = i - 10; j <= i; j++)
                            if (squared[j] > currentSlope)
                                currentSlope = squared[j];

                        if ((currentSlope < (dataType)(lastSlope/2)) && (i + sample) < lastQRS + 0.36*lastQRS)
                        {
                            qrs = false;
                        }
                        else
                        {
                            peak_i = integral[i];
                            peak_f = highpass[i];
                            spk_i = 0.25*peak_i+ 0.75*spk_i;
                            spk_f = 0.25*peak_f + 0.75*spk_f;
                            threshold_i1 = npk_i + 0.25*(spk_i - npk_i);
                            threshold_i2 = 0.5*threshold_i1;
                            lastSlope = currentSlope;
                            threshold_f1 = npk_f + 0.25*(spk_f - npk_f);
                            threshold_f2 = 0.5*threshold_f1;
                            // If a signal peak was detected on the back search, the RR attributes must be updated.
                            // This is the same thing done when a peak is detected on the first try.
                            //RR Average 1
                            rravg1 = 0;
                            for (j = 0; j < 7; j++)
                            {
                                rr1[j] = rr1[j+1];
                                rravg1 += rr1[j];
                            }
                            rr1[7] = sample - (current - i) - lastQRS;
                            qrs = true;
                            lastQRS = sample - (current - i);
                            rravg1 += rr1[7];
                            rravg1 *= 0.125;

                            //RR Average 2
                            if ( (rr1[7] >= rrlow) && (rr1[7] <= rrhigh) )
                            {
                                rravg2 = 0;
                                for (i = 0; i < 7; i++)
                                {
                                    rr2[i] = rr2[i+1];
                                    rravg2 += rr2[i];
                                }
                                rr2[7] = rr1[7];
                                rravg2 += rr2[7];
                                rravg2 *= 0.125;
                                rrlow = 0.92*rravg2;
                                rrhigh = 1.16*rravg2;
                                rrmiss = 1.66*rravg2;
                            }

                            prevRegular = regular;
                            if (rravg1 == rravg2)
                            {
                                regular = true;
                            }
                            else
                            {
                                regular = false;
                                if (prevRegular)
                                {
                                    threshold_i1 /= 2;
                                    threshold_f1 /= 2;
                                }
                            }

                            break;
                        }
                    }
				}

				if (qrs)
                {
                    outputSignal[current] = false;
                    outputSignal[i] = true;
                    if (sample > DELAY + BUFFSIZE)
                        outputSignal[sample]=outputSignal[sample - 1]; // fix this
                    continue;
                }
			}

			// Definitely no signal peak was detected.
			if (!qrs)
			{
				// If some kind of peak had been detected, then it's certainly a noise peak. Thresholds must be updated accordinly.
				if ((integral[current] >= threshold_i1) || (highpass[current] >= threshold_f1))
				{
					peak_i = integral[current];
					npk_i = 0.125*peak_i + 0.875*npk_i;
					threshold_i1 = npk_i + 0.25*(spk_i - npk_i);
					threshold_i2 = 0.5*threshold_i1;
					peak_f = highpass[current];
					npk_f = 0.125*peak_f + 0.875*npk_f;
					threshold_f1 = npk_f + 0.25*(spk_f - npk_f);
					threshold_f2 = 0.5*threshold_f1;
				}
			}
		}
		
        outputSignal[sample - 1] = qrs;
        if (sample > DELAY + BUFFSIZE)
			outputSignal[sample]=outputSignal[sample - 1]; // fix this
		
		sample++;

		
    }while (sample < inputSize);
	
    // Output the last remaining samples on the buffer
	for (int i = 0; i < inputSize; i++) {
		
        printf("%d\t%d\n", i, outputSignal[i]);
    }
	
	
    
	



	
}	




int main(void){

    int inputSignal[BUFFSIZE] = {926, 920, 916, 911, 902, 896, 896, 911, 935, 961, 992, 1036, 1092, 1144, 1181, 1194, 1172, 1115, 1040, 972};
	//942, 940, 941, 942, 941, 938, 937, 937, 937, 937, 938, 937, 934, 932, 934, 937, 936, 935, 933, 933, 934, 934, 935, 935, 930, 931, 933, 935, 934, 935, 932, 930, 926, 920, 916, 911, 902, 896, 896, 911, 935, 961, 992, 1036, 1092, 1144, 1181, 1194, 1172, 1115, 1040, 972, 927, 910, 908, 913, 923, 925, 929, 927, 927, 926, 927, 930, 929, 926, 925, 924, 923, 926, 925, 924, 925, 925, 925, 929, 929, 926, 925, 926, 928, 928, 928, 926, 924, 924, 928, 929, 927, 928, 923, 927, 928, 931, 928, 926, 925, 926, 927, 927, 929, 926, 925, 924, 926, 928, 927, 925, 924, 927, 927, 929, 928, 927, 925, 924, 926, 927, 927, 925, 924, 925, 927, 929, 930, 927, 926, 928, 924, 927, 929, 926, 923, 925, 924, 925, 924, 920, 918, 919, 921, 923, 921, 921, 918, 919, 922, 923, 923, 923, 918, 920, 921, 926, 926, 925, 923, 926, 932, 935, 938, 938, 938, 943, 942, 946, 944, 945, 944, 946, 947, 949, 949, 948, 949, 949, 951, 951, 951, 951, 946, 949, 950, 951, 951, 950, 946, 948, 951, 951, 950, 946, 946, 946, 948, 950, 948, 947, 944, 945, 946, 947, 946, 945, 943, 942, 945, 946, 944, 943, 945, 943, 946, 948, 948, 947, 945, 945, 945, 945, 946, 944, 943, 944, 944, 945, 945, 944, 940, 943, 941, 946, 943, 943, 941, 941, 943, 945, 945, 943, 940, 942, 944, 947, 947, 945, 941, 942, 944, 947, 946, 946, 942, 942, 945, 943, 945, 942, 941, 942, 943, 944, 945, 944, 941, 942, 945, 946, 945, 947, 948, 951, 955, 960, 961, 961, 956, 956, 961, 962, 962, 960, 961, 961, 965, 966, 966, 967, 963, 963, 964, 964, 962, 959, 957, 959, 960, 962, 961, 961, 962, 968, 970, 967, 961, 956, 951, 951, 949, 947, 946, 939, 937, 939, 941, 942, 943, 941, 938, 938, 940, 941, 942, 941, 936, 938, 941, 942, 940, 939, 937, 938, 937, 939, 936, 931, 921, 914, 913, 910, 903, 896, 894, 905, 930, 959, 988, 1023};
	
    int outputSignal[BUFFSIZE] = {0};

    // Call panTompkins function to process the input signal
	panTompkins(inputSignal, outputSignal, BUFFSIZE);

    return 0;

}
