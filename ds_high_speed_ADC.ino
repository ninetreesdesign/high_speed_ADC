/** @file ds_high_speed_ADC.ino
 *  @description Perform fast ADC sampling directly to RAM in a burst
 *  to capture impact profile measured by an accelerometer.
 *  Print the data immediately after.
 *  A change on an input pin (from a sensor) triggers the capture window
 *
 *  @reference Teensy Example: analogContinuousRead.ino
 *  @author David Smith
 */

#include <ADC.h>
#include <ADC_util.h>
                                                 

const String VERSION = "0.9.2";
const float v_ref = 3.297;          // measured reference voltage (3.3V nominal)
const uint_fast32_t LOOP_OVERHEAD = 1;  // observed
const uint32_t TIME_STEP_US      = 100; // can't do too much less than this
const uint32_t BURST_INTERVAL_US = 500 * 1000;
const uint32_t A_SIZE = BURST_INTERVAL_US / TIME_STEP_US;       // do not exceed available RAM ~ < 10000 words;
uint16_t ana_data[3][A_SIZE];
uint32_t times[A_SIZE];             // store relative times in ms of the samples
uint32_t t0;
 
elapsedMicros sample_timer;

ADC *adc = new ADC();               // adc object

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
 //   pinMode(A0, INPUT); // analog
  //  pinMode(A1, INPUT);
  //  pinMode(A2, INPUT);
 //   pinMode(A3, INPUT);
    // configure ADC
    adc->adc0->setAveraging(4);           // set number of averages
    adc->adc0->setResolution(16);         // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
    //adc->adc0->setReference(ADC_REFERENCE::REF_1V2, ADC_0);
    ///adc->adc0->enableCompare(3.29/v_ref * adc->adc0->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 3.3V (always)
    adc->adc0->enableCompare(3.29/v_ref *adc->adc0->getMaxValue(), 0); // measurement will be ready if value < xV
    //adc->adc0->enableCompareRange(1.0*adc->adc0->getMaxValue()/3.3, 2.0*adc->adc0->getMaxValue()/3.3, 0, 1); // ready if value lies out of [1.0,2.0] V

    adc->adc0->startContinuous(A0);  
    adc->adc0->startContinuous(A1);  
    adc->adc0->startContinuous(A2);  


    // initialize USB serial port (The IDE's console window must be opened)
    const uint16_t TIMEOUT_INTERVAL = 5000;
    uint32_t t_elapsed = millis();
    Serial.begin(115200);   // on USB port, baudrate is always 14Mbit/s
    while (!Serial && (millis() - t_elapsed < TIMEOUT_INTERVAL)) {
        ; // wait for port to connect or skip after timeout. Req'd on arduinos with USB port systems
    }
    //Serial.println("\n  USB port initialized");

    t0 = micros();
                                                          
 }
void loop() {
    // do a single burst read of data for a few ms
    static int16_t first_loop_flag = 1;
    static uint16_t index = 0;
    static uint32_t read_time;
    static uint32_t print_time;
    uint32_t t1;

    if (first_loop_flag == 1) {
        if (sample_timer >= (TIME_STEP_US-LOOP_OVERHEAD) && index < A_SIZE) {
            //times[index] = micros() - t0;   //index * TIME_STEP_US;
            times[index] = micros() - t0; //sample_timer;
                                                                         
            sample_timer = 0; //-= TIME_STEP_US;
            ana_data[0][index] = (uint16_t)adc->adc0->analogRead(A0);//analogReadContinuous(); // way faster (<2ms vs. 59ms for this loop)
            ana_data[1][index] = (uint16_t)adc->adc0->analogRead(A1);//analogReadContinuous(); 
            ana_data[2][index] = (uint16_t)adc->adc0->analogRead(A2);//analogReadContinuous(); 
            index++;
        }
        if (index == A_SIZE) {  // done sampling
            t1 = micros();
            read_time = (t1 - t0);
            first_loop_flag = -1;
        }
        //delayMicroseconds(TIME_STEP_US - LOOP_OVERHEAD); // tmp
    }
    else if (first_loop_flag == -1) {    // print data
        // measure printing time
        t0 = micros();
        for (uint16_t i = 1; i < A_SIZE; i++) {
            Serial.printf("  %6u\t%3u \t%5u \t%5u \t%5u \n",times[i],times[i] - times[i-1], 
            ana_data[0][i], ana_data[1][i], ana_data[2][i]);
        }
        t1 = micros();
        print_time = (t1 - t0);

        Serial.println("\n  Version: " + VERSION);
        Serial.printf("  Time step [us]: \t%5u \n", TIME_STEP_US);
        Serial.printf("  Number of samples: \t%5u \n", A_SIZE);
        Serial.printf("  Calc.  Burst interval  [ms]:\t%5u \n", BURST_INTERVAL_US/1000);
        Serial.printf("  Actual Burst interval  [ms]:\t %7.3f \n", (float)read_time/1000.0);
        //Serial.printf("  Print time [ms]: \t %7.4f \n", (float)print_time/1000.0);
        first_loop_flag = 1;
    }
    else {
    }

}
