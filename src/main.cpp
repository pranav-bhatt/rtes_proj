#include "mbed.h"
#include <cmath>
#include <vector>
#include <algorithm>

// ------------------- Gyro Register and Config -------------------
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00010000
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b00001000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define OUT_X_L 0x28

EventFlags flags;

// ISR Callbacks: No printing here
void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

void data_cb()
{
    flags.set(DATA_READY_FLAG);
}

// Scale factor for converting raw data to angular velocity
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// ------------------- Gesture & State Management -------------------
enum State
{
    IDLE,
    RECORDING,
    VALIDATING
};

#define NUM_SAMPLES 50
#define AXES 3

float recorded_gyro_data[NUM_SAMPLES][AXES];
float validate_gyro_data[NUM_SAMPLES][AXES];

// Track if a gesture has been recorded
bool gestureRecorded = false;

// LEDs on STM32F4 Discovery
DigitalOut led_green(PD_12);
DigitalOut led_orange(PD_13);
DigitalOut led_red(PD_14);
DigitalOut led_blue(PD_15);

// User button on PA_0
InterruptIn button(PA_0, PullDown);
Timer buttonTimer;
volatile bool buttonPressed = false;
volatile bool buttonReleased = false;

// Button Handling: No printing in ISRs
void onButtonPressed()
{
    buttonTimer.reset();
    buttonTimer.start();
    buttonPressed = true;
    buttonReleased = false;
}

void onButtonReleased()
{
    buttonTimer.stop();
    buttonReleased = true;
}

// SPI and interrupt pin for gyro
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
InterruptIn int2(PA_2, PullDown);

// DTW function runs in main thread, safe to print
float dtw_distance(const float seq1_gyro_data[][AXES], int len1,
                   const float seq2_gyro_data[][AXES], int len2)
{
    printf("[DEBUG] Computing DTW distance.\n");
    std::vector<std::vector<float>> dp(len1 + 1, std::vector<float>(len2 + 1, INFINITY));
    dp[0][0] = 0.0f;

    for (int i = 1; i <= len1; i++)
    {
        for (int j = 1; j <= len2; j++)
        {
            float dist = std::sqrt((seq1_gyro_data[i - 1][0] - seq2_gyro_data[j - 1][0]) * (seq1_gyro_data[i - 1][0] - seq2_gyro_data[j - 1][0]) +
                                   (seq1_gyro_data[i - 1][1] - seq1_gyro_data[j - 1][1]) * (seq1_gyro_data[i - 1][1] - seq1_gyro_data[j - 1][1]) +
                                   (seq1_gyro_data[i - 1][2] - seq2_gyro_data[j - 1][2]) * (seq1_gyro_data[i - 1][2] - seq2_gyro_data[j - 1][2]));
            dp[i][j] = dist + std::min({dp[i - 1][j], dp[i][j - 1], dp[i - 1][j - 1]});
        }
    }

    float finalDist = dp[len1][len2];
    printf("[DEBUG] DTW complete. Distance: %.2f\n", finalDist);
    return finalDist;
}

// Clear recording arrays before re-recording
void clear_recording_arrays()
{
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        recorded_gyro_data[i][0] = 0.0f;
        recorded_gyro_data[i][1] = 0.0f;
        recorded_gyro_data[i][2] = 0.0f;
    }
}

// Reading gyro samples in main context is safe to print
bool read_gyro_samples(float g_arr[][AXES], int numSamples)
{
    float alpha = 0.1f;
    float filtered_gx = 0.0f;
    float filtered_gy = 0.0f;
    float filtered_gz = 0.0f;
    float new_gx = 0.0f;
    float new_gy = 0.0f;
    float new_gz = 0.0f;

    for (int i = 0; i < numSamples; i++)
    {
        printf("[DEBUG] Reading %d samples from gyro.\n", numSamples);
        uint8_t write_buf[32], read_buf[32];
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);

        // Wait for SPI flag, also in main context
        flags.wait_all(SPI_FLAG);

        uint16_t raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        uint16_t raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        uint16_t raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // get scaled gyro readings
        new_gx = ((float)raw_gx) * SCALING_FACTOR;
        new_gy = ((float)raw_gy) * SCALING_FACTOR;
        new_gz = ((float)raw_gz) * SCALING_FACTOR;

        // update filtered gyro data
        filtered_gx = alpha * new_gx + (1.0f - alpha) * filtered_gx;
        filtered_gy = alpha * new_gy + (1.0f - alpha) * filtered_gy;
        filtered_gz = alpha * new_gz + (1.0f - alpha) * filtered_gz;

        // store filtered values into the buffer
        g_arr[i][0] = filtered_gx;
        g_arr[i][1] = filtered_gy;
        g_arr[i][2] = filtered_gz;

        printf("[DEBUG] Sample %d: gx=%.5f, gy=%.5f, gz=%.5f\n", i, g_arr[i][0], g_arr[i][1], g_arr[i][2]);
        ThisThread::sleep_for(50ms);
    }

    printf("[DEBUG] Finished reading gyro samples.\n");

    return true;
}

// show_result runs in main context, safe to print
void show_result(bool success)
{
    printf("[DEBUG] Showing result. Success: %d\n", success ? 1 : 0);
    led_green = 0;
    led_orange = 0;
    led_red = 0;
    led_blue = 0;

    if (success)
    {
        for (int i = 0; i < 3; i++)
        {
            led_green = 1;
            ThisThread::sleep_for(300ms);
            led_green = 0;
            ThisThread::sleep_for(300ms);
        }
    }
    else
    {
        led_red = 1;
        ThisThread::sleep_for(3s);
        led_red = 0;
    }
    printf("[DEBUG] Result indication complete.\n");
}

DigitalIn userButton(PA_0, PullDown);

int main()
{
    printf("[DEBUG] Starting main...\n");

    int2.rise(&data_cb);

    // Configure SPI - same as before...
    // Initialize gyro registers - same as before...

    State currentState = IDLE;
    printf("[DEBUG] Entering main loop. State: IDLE\n");

    bool buttonWasPressed = false;

    while (true)
    {
        bool buttonState = userButton.read(); // 1 if pressed (assuming PullDown), 0 if not
        if (buttonState && !buttonWasPressed)
        {
            // Button just pressed
            buttonTimer.reset();
            buttonTimer.start();
            buttonWasPressed = true;
        }
        else if (!buttonState && buttonWasPressed)
        {
            // Button was just released
            buttonTimer.stop();
            int pressDuration = buttonTimer.read_ms();
            printf("[DEBUG] Button released after %d ms.\n", pressDuration);

            // Decide state change logic
            if (pressDuration > 1000)
            {
                // Long press -> RECORDING
                printf("[DEBUG] Long press: Prepare to record new gesture.\n");
                gestureRecorded = false;
                clear_recording_arrays();
                currentState = RECORDING;
                printf("[DEBUG] State changed to RECORDING.\n");
            }
            else
            {
                // Short press -> VALIDATING if gesture recorded, else IDLE
                if (gestureRecorded)
                {
                    currentState = VALIDATING;
                    printf("[DEBUG] State changed to VALIDATING.\n");
                }
                else
                {
                    printf("[DEBUG] No recorded gesture. Staying in IDLE.\n");
                    currentState = IDLE;
                }
            }

            buttonWasPressed = false;
        }

        switch (currentState)
        {
        case IDLE:
            ThisThread::sleep_for(100ms);
            break;

        case RECORDING:
            printf("[DEBUG] Recording gesture...\n");
            if (read_gyro_samples(recorded_gyro_data, NUM_SAMPLES))
            {
                printf("[DEBUG] Gesture recorded successfully.\n");
                gestureRecorded = true;
            }
            else
            {
                printf("[DEBUG] Gesture recording failed.\n");
            }
            currentState = IDLE;
            printf("[DEBUG] State changed to IDLE.\n");
            break;

        case VALIDATING:
            printf("[DEBUG] Validating gesture...\n");
            if (read_gyro_samples(validate_gyro_data, NUM_SAMPLES))
            {
                printf("[DEBUG] Validation samples read. Computing DTW...\n");
                float dtwDist = dtw_distance(recorded_gyro_data, NUM_SAMPLES,
                                             validate_gyro_data, NUM_SAMPLES);

                float maxDist = 500.0f;
                float similarity = (1.0f - (dtwDist / maxDist)) * 100.0f;
                printf("[DEBUG] DTW Distance: %.2f, Similarity: %.2f%%\n", dtwDist, similarity);

                bool success = (similarity >= 90.0f);
                show_result(success);
            }
            else
            {
                printf("[DEBUG] Validation sample reading failed.\n");
            }
            currentState = IDLE;
            printf("[DEBUG] State changed to IDLE.\n");
            break;
        }
    }
}