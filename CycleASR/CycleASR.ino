#include <Cycle_actual_inferencing.h>
#include <Arduino_LSM9DS1.h> 
#include <Arduino.h>
#include <GSMSimSMS.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead
#define RED 22
#define BLUE 24
#define GREEN 23
#define BUZZER_PIN 9 // Define the pin connected to the buzzer
#define SIM800_TX_PIN 0  // Nano 33 BLE RX1 pin
#define SIM800_RX_PIN 1  // Nano 33 BLE TX1 pin
#define RESET_PIN 10     // Any pin you choose for reset
 unsigned long startTime = 0; // Variable to store the start time



static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static uint32_t run_inference_every_ms = 200;
static rtos::Thread inference_thread(osPriorityLow);
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

void run_inference_background();

/**
* @brief      
*/
GSMSimSMS sms(Serial1, RESET_PIN); // Use Serial1 for SIM800L module

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(9600);  // Serial communication with SIM800L module

    while (!Serial1) {
    ; // Wait for SIM800L module to connect
  }

  
   
 

    pinMode(RED,OUTPUT);
    pinMode(BLUE,OUTPUT);
    pinMode(GREEN,OUTPUT);
    pinMode(2,OUTPUT);

    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

    inference_thread.start(mbed::callback(&run_inference_background));
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
 * @brief      Run inferencing in the background.
 */
void run_inference_background()
{
    // wait until we have a full buffer
    delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);

    // This is a structure that smoothens the output result
    // With the default settings 70% of readings should be the same before classifying.
    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 10 /* no. of readings */, 7 /* min. readings the same */, 0.8 /* min. confidence */, 0.3 /* max anomaly */);

    while (1) {
        // copy the buffer
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        // Run the classifier
        ei_impulse_result_t result = { 0 };

        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": ");

        // ei_classifier_smooth_update yields the predicted label
        const char *prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("%s ", prediction);
        // print the cumulative results
        ei_printf(" [ ");
        for (size_t ix = 0; ix < smooth.count_size; ix++) {
            ei_printf("%u", smooth.count[ix]);
            if (ix != smooth.count_size + 1) {
                ei_printf(", ");
            }
            else {
              ei_printf(" ");
            }
        }
        if (strcmp(prediction, "left-tilt") == 0 || strcmp(prediction, "right-tilt") == 0) {
               // Turn on red LED
              digitalWrite(BLUE,HIGH); 
              digitalWrite(GREEN,HIGH);
              digitalWrite(RED,LOW);  
              digitalWrite(2,HIGH);
                    Serial.println("Initializing SIM800L...");
                    // Initialize module
                    sms.init();

              // Set phone function
                    Serial.print("Set Phone Function... ");
                    Serial.println(sms.setPhoneFunc(1));
                    delay(1000);

                    // Check if module is registered to network
                    Serial.print("Is Module Registered to Network?... ");
                    Serial.println(sms.isRegistered());
                    delay(1000);

                    // Check signal quality
                    Serial.print("Signal Quality... ");
                    Serial.println(sms.signalQuality());
                    delay(1000);

                    // Get operator name
                    Serial.print("Operator Name... ");
                    Serial.println(sms.operatorNameFromSim());
                    delay(1000);

                    // Initialize SMS
                    Serial.print("Init SMS... ");
                    Serial.println(sms.initSMS());
                    delay(1000);

                    // List unread SMS
                    Serial.print("List Unread SMS... ");
                    Serial.println(sms.list(true));
                    delay(1000);

                    // Send SMS
                    Serial.print("Sending SMS... ");
                    Serial.println(sms.send("+919307787643", "Message indicating Cycle Accident")); 
                  
            } else if (strcmp(prediction, "idle") == 0) {
                // Turn on Green LED
              digitalWrite(BLUE,HIGH); 
              digitalWrite(GREEN,LOW);
              digitalWrite(RED,HIGH);  
              digitalWrite(2,HIGH);  

            }  else if (strcmp(prediction, "uncertain") == 0) {
                // Turn on Blue LEDs
              digitalWrite(BLUE,LOW); 
              digitalWrite(GREEN,HIGH);
              digitalWrite(RED,HIGH);  
              digitalWrite(2,HIGH);
            
    }
        ei_printf("]\n");

        delay(run_inference_every_ms);
    }

    ei_classifier_smooth_free(&smooth);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    while (1) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        // roll the buffer -3 points so we can overwrite the last one
        numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

        // read to the end of the buffer
        IMU.readAcceleration(
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1]
        );

        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) > MAX_ACCEPTED_RANGE) {
                buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] = ei_get_sign(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] *= CONVERT_G_TO_MS2;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] *= CONVERT_G_TO_MS2;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] *= CONVERT_G_TO_MS2;

        // and wait for next tick
        uint64_t time_to_wait = next_tick - micros();
        delay((int)floor((float)time_to_wait / 1000.0f));
        delayMicroseconds(time_to_wait % 1000);
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
