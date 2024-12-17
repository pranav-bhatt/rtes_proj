#include "mbed.h"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "drivers/LCD_DISCO_F429ZI.h"
#include "stm32f4xx.h"
#include <string.h>

// Color definitions
#define RED 0xF800   // RGB: 1111100000000000
#define GREEN 0x07E0 // RGB: 0000011111100000
#define BLUE 0x001F  // RGB: 0000000000011111
#define WHITE 0xFFFF // RGB: 1111111111111111
#define BLACK 0x0000 // RGB: 0000000000000000

// ------------------- Gyro Register and Config -------------------
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0x6F
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0x00
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0x08

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
#define SCALING_FACTOR (8.75f * 0.0174532925199432957692236907684886f / 1000.0f)

// ------------------- Gesture & State Management -------------------
enum State
{
    IDLE,
    RECORDING,
    VALIDATING
};

// Gyro data artifacts
#define NUM_SAMPLES 100
#define AXES 3

struct RollingStats
{
    float mean;
    float variance;
    uint16_t count; // Changed from int to uint16_t

    RollingStats() : mean(0.0f), variance(0.0f), count(0) {}
};

RollingStats stats_x, stats_y, stats_z;

// Function to update rolling statistics
void update_rolling_stats(RollingStats &stats, float new_value)
{
    stats.count++;
    float delta = new_value - stats.mean;
    stats.mean += delta / stats.count;
    stats.variance += delta * (new_value - stats.mean);
}

// Function to get rolling standard deviation
float get_rolling_stddev(RollingStats &stats)
{
    return stats.count > 1 ? std::sqrt(stats.variance / (stats.count - 1)) : 1.0f; // Avoid division by zero
}

float recorded_gyro_data[NUM_SAMPLES][AXES];
float validate_gyro_data[NUM_SAMPLES][AXES];
// Calibration variables
float gyro_bias[AXES] = {0.0f, 0.0f, 0.0f};

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

static float dp[(NUM_SAMPLES + 1) * (NUM_SAMPLES + 1)];

static float dtw_distance(const float seq1_gyro_data[][AXES], uint16_t len1,
                          const float seq2_gyro_data[][AXES], uint16_t len2)
{
    printf("[DEBUG] Computing DTW distance.\n");

    // Initialize dp array to INFINITY
    for (uint16_t i = 0; i <= len1; i++)
    {
        for (uint16_t j = 0; j <= len2; j++)
        {
            dp[i * (len2 + 1) + j] = INFINITY;
        }
    }
    dp[0 * (len2 + 1) + 0] = 0.0f;

    for (uint16_t i = 1; i <= len1; i++)
    {
        for (uint16_t j = 1; j <= len2; j++)
        {
            float dist = std::sqrt((seq1_gyro_data[i - 1][0] - seq2_gyro_data[j - 1][0]) *
                                       (seq1_gyro_data[i - 1][0] - seq2_gyro_data[j - 1][0]) +
                                   (seq1_gyro_data[i - 1][1] - seq2_gyro_data[j - 1][1]) *
                                       (seq1_gyro_data[i - 1][1] - seq2_gyro_data[j - 1][1]) +
                                   (seq1_gyro_data[i - 1][2] - seq2_gyro_data[j - 1][2]) *
                                       (seq1_gyro_data[i - 1][2] - seq2_gyro_data[j - 1][2]));

            float min_val = std::min({
                dp[(i - 1) * (len2 + 1) + j],      // from top
                dp[i * (len2 + 1) + (j - 1)],      // from left
                dp[(i - 1) * (len2 + 1) + (j - 1)] // from diagonal
            });
            dp[i * (len2 + 1) + j] = dist + min_val;
        }
    }

    float finalDist = dp[len1 * (len2 + 1) + len2];
    printf("[DEBUG] DTW complete. Distance: %.2f\n", finalDist);
    return finalDist;
}

// Clear recording arrays before re-recording
void clear_recording_arrays()
{
    for (uint16_t i = 0; i < NUM_SAMPLES; i++)
    {
        recorded_gyro_data[i][0] = 0.0f;
        recorded_gyro_data[i][1] = 0.0f;
        recorded_gyro_data[i][2] = 0.0f;
    }
}

// Function to read raw gyroscope data
void read_raw_gyro(float *gx, float *gy, float *gz)
{
    uint8_t write_buf[32], read_buf[32];
    write_buf[0] = OUT_X_L | 0x80 | 0x40; // Read command with auto-increment
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);

    // Wait for SPI transfer to complete
    flags.wait_all(SPI_FLAG);

    uint16_t raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    uint16_t raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    uint16_t raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    // Convert raw data to angular velocity using scaling factor
    *gx = ((float)raw_gx) * SCALING_FACTOR;
    *gy = ((float)raw_gy) * SCALING_FACTOR;
    *gz = ((float)raw_gz) * SCALING_FACTOR;
}

class LPFFilter
{
private:
    float alpha;       // Filter coefficient
    float prev_output; // Previous filtered output

public:
    LPFFilter(float cutoff_freq, float sample_rate)
    {
        float RC = 1.0f / (2.0f * 3.14159f * cutoff_freq);
        float dt = 1.0f / sample_rate;
        alpha = dt / (RC + dt);
        prev_output = 0.0f;
    }

    LPFFilter() : alpha(0.1f), prev_output(0.0f) {}

    float update(float input)
    {
        prev_output = alpha * input + (1.0f - alpha) * prev_output;
        return prev_output;
    }

    void reset()
    {
        prev_output = 0.0f;
    }
};

bool read_gyro_samples(float g_arr[][AXES], uint16_t numSamples) // Changed parameter type
{
    static LPFFilter z_axis_filter(0.5f, 1000.0f);

    float alphax = 0.9f;
    float alphay = 0.9f;
    float filtered_gx = 0.0f;
    float filtered_gy = 0.0f;
    float new_gx = 0.0f;
    float new_gy = 0.0f;
    float new_gz = 0.0f;

    printf("[DEBUG] Reading %d samples from gyro with Z-axis LPF.\n", numSamples);

    z_axis_filter.reset();

    for (uint16_t i = 0; i < numSamples; i++)
    {
        read_raw_gyro(&new_gx, &new_gy, &new_gz);

        printf("%.2f %.2f %.2f\n", new_gx, new_gy, new_gz);

        // Remove bias
        new_gx -= gyro_bias[0];
        new_gy -= gyro_bias[1];
        new_gz -= gyro_bias[2];

        if (filtered_gx == 0.0f && filtered_gy == 0.0f)
        {
            filtered_gx = new_gx;
            filtered_gy = new_gy;
        }

        filtered_gx = alphax * new_gx + (1.0f - alphax) * filtered_gx;
        filtered_gy = alphay * new_gy + (1.0f - alphay) * filtered_gy;

        float filtered_gz = z_axis_filter.update(new_gz);

        g_arr[i][0] = filtered_gx;
        g_arr[i][1] = filtered_gy;
        g_arr[i][2] = filtered_gz;

        printf("[DEBUG] Sample %d: gx=%.5f, gy=%.5f, gz=%.5f\n", i, g_arr[i][0], g_arr[i][1], g_arr[i][2]);

        ThisThread::sleep_for(1ms);
    }

    printf("[DEBUG] Finished reading gyro samples with Z-axis LPF.\n");

    return true;
}

void show_result(bool success)
{
    printf("[DEBUG] Showing result. Success: %d\n", success ? 1 : 0);
    led_green = 0;
    led_orange = 0;
    led_red = 0;
    led_blue = 0;

    if (success)
    {
        led_green = 1;
        ThisThread::sleep_for(300ms);
        led_green = 0;
        ThisThread::sleep_for(300ms);

        LCD_DISCO_F429ZI lcd;
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.SetTextColor(LCD_COLOR_BLACK);
        lcd.SetBackColor(LCD_COLOR_GREEN);
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"IT'S A MATCH!", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"DEVICE UNLOCKED.", CENTER_MODE);
        ThisThread::sleep_for(3s);
    }
    else
    {
        led_red = 1;
        ThisThread::sleep_for(300ms);
        led_red = 0;
        ThisThread::sleep_for(300ms);

        LCD_DISCO_F429ZI lcd;
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.SetTextColor(LCD_COLOR_BLACK);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"WRONG!", CENTER_MODE);
        ThisThread::sleep_for(3s);
    }
    printf("[DEBUG] Result indication complete.\n");
}

void show_calib_prompt(u_int8_t completed)
{
    printf("[DEBUG] Showing Calibration prompt.");
    led_green = 0;
    led_orange = 0;
    led_red = 0;
    led_blue = 0;

    led_green = 1;
    ThisThread::sleep_for(300ms);
    led_green = 0;
    ThisThread::sleep_for(300ms);

    LCD_DISCO_F429ZI lcd;
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_MAGENTA);

    if (completed == 0)
    {
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"PLACE DEVICE ON", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"FLAT SURFACE AND", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"PRESS BUTTON", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"TO CALIBRATE.", CENTER_MODE);
    }
    else if (completed == 1)
    {
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"CALIBRATING...", CENTER_MODE);
    }
    else if (completed == 2)
    {
        lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"CALIBRATION COMPLETE!", CENTER_MODE);
    }
    ThisThread::sleep_for(3s);
}

DigitalIn userButton(PA_0, PullDown);

void calibrate_gyroscope()
{
    printf("[DEBUG] Calibrating gyroscope. Place on flat surface and press the button.\n");
    show_calib_prompt(0);
    while (!userButton.read())
    {
        ThisThread::sleep_for(100ms);
    }

    printf("[DEBUG] Button pressed. Starting calibration...\n");
    show_calib_prompt(1);

    float sum[AXES] = {0.0f, 0.0f, 0.0f};
    float gx, gy, gz;

    uint16_t calib_samples = 300; // Changed from int to uint16_t

    for (uint16_t i = 0; i < calib_samples; i++)
    {
        read_raw_gyro(&gx, &gy, &gz);
        sum[0] += gx;
        sum[1] += gy;
        sum[2] += gz;
        ThisThread::sleep_for(10ms);
    }

    gyro_bias[0] = sum[0] / calib_samples;
    gyro_bias[1] = sum[1] / calib_samples;
    gyro_bias[2] = sum[2] / calib_samples;

    printf("[DEBUG] Calibration complete. Bias X=%.5f, Y=%.5f, Z=%.5f\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    show_calib_prompt(2);
}

int main()
{
    printf("[DEBUG] Starting main...\n");

    int2.rise(&data_cb);

    State currentState = IDLE;
    State previousState = (State)-1; // Invalid initial state, ensuring first iteration triggers an update.
    printf("[DEBUG] Entering main loop. State: IDLE\n");

    bool buttonWasPressed = false;

    // Calibration
    calibrate_gyroscope();

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
            if (pressDuration > 3000)
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

        // Only update the display if the state has changed
        if (currentState != previousState)
        {
            LCD_DISCO_F429ZI lcd;

            // Clear the screen with a black background
            lcd.Clear(LCD_COLOR_BLACK);

            switch (currentState)
            {
            case IDLE:
                // Set text color to white
                lcd.SetTextColor(LCD_COLOR_WHITE);
                // Set background color to black
                lcd.SetBackColor(LCD_COLOR_BLACK);
                lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"IDLE", CENTER_MODE);
                break;

            case RECORDING:
                printf("[DEBUG] Recording gesture...\n");
                // Set text color to white
                lcd.SetTextColor(LCD_COLOR_WHITE);
                // Set background color
                lcd.SetBackColor(LCD_COLOR_DARKBLUE);
                lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"*RECORDING*", CENTER_MODE);
                break;

            case VALIDATING:
                printf("[DEBUG] Validating gesture...\n");
                // Set text color to white
                lcd.SetTextColor(LCD_COLOR_WHITE);
                // Set background color to black
                lcd.SetBackColor(LCD_COLOR_BLACK);
                lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"VALIDATING", CENTER_MODE);
                break;
            }

            previousState = currentState;
        }

        // Perform actions associated with the states if needed
        switch (currentState)
        {
        case IDLE:
            // In IDLE, just sleep for a short duration to save power/CPU cycles
            ThisThread::sleep_for(100ms);
            break;

        case RECORDING:
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
            if (read_gyro_samples(validate_gyro_data, NUM_SAMPLES))
            {
                printf("[DEBUG] Validation samples read. Computing DTW...\n");
                float dtwDist = dtw_distance(recorded_gyro_data, NUM_SAMPLES,
                                             validate_gyro_data, NUM_SAMPLES);

                float threshold = 300.0f;
                bool success = (dtwDist < threshold);
                printf("[DEBUG] DTW Distance: %.2f, Success: %d\n", dtwDist, success);

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
