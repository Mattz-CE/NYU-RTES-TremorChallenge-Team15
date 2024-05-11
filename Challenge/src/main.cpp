#include <mbed.h>
#include <math.h>
#include <stddef.h>

// =================================================
// * Recitation 5: SPI and Gyroscope *
// =================================================

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot.
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

EventFlags flags;
#include <iostream>

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

void controlLight(double intensityThreshold, DigitalOut &led1, int count, float freq)
{
    // we only trigger if vibration is in frequency of 3 to 6 hz
    if (freq > 3 && freq < 6)
    {
        // Reset the count based on the intensity to create a looped flicker effect
        if (intensityThreshold > 3)
        {
            // Higher intensity: Faster flickering
            if (count % 64 < 32)
            {             // Modulo operation creates a repeating pattern
                led1 = 1; // LED on
            }
            else
            {
                led1 = 0; // LED off
            }
        }
        else if (intensityThreshold > 1.5)
        {
            // Lower intensity: Slower flickering
            if (count % 96 < 48)
            {             // Longer on-phase
                led1 = 1; // LED on
            }
            else
            {
                led1 = 0; // LED off
            }
        }
        else
        {
            // Very low intensity: Light stays off
            led1 = 0; // LED off
        }
    }
}

float oscillations_count(float ray[])
{                         // goal of this function is to calculate the frequency by checking the number of times the array oscillates
    int signNow = 0;      // keep track of current sign, 1 is neg, 0 is pos
    int signPast = 0;     // keep track of previous sign, 1 is neg, 0 is pos
    float oscCount = 0.0; // counts the total number of oscillations
    for (int i = 0; i <= 1000; i++)
    {
        // print the value of the array
        // printf("ray[%d]: %f\n", i, ray[i]);
        if (ray[i] > 0)
        {
            signNow = 0;
        }
        else if (ray[i] < 0)
        {
            signNow = 1;
        }
        if (signNow != signPast)
        {
            oscCount++;
        }
        signPast = signNow;
    }
    float out = oscCount / 4.0;
    return out;
}

int main()
{
    class CircularBuffer
    {
    private:
        int maxSize;   // Maximum size of the buffer
        float *data;   // Dynamically allocated array to store data points
        int head = 0;  // Index of the newest element
        int tail = 0;  // Index of the oldest element
        int count = 0; // Number of elements currently in the buffer

    public:
        // Constructor with buffer size parameter
        CircularBuffer(int size) : maxSize(size), data(new float[size]) {}

        // Destructor to free allocated memory
        ~CircularBuffer()
        {
            delete[] data;
        }

        // Function to add a data point to the buffer
        void addData(float value)
        {
            if (count >= maxSize)
            {                                // Check if buffer is full
                tail = (tail + 1) % maxSize; // Increment tail to remove the oldest data point
            }
            else
            {
                count++; // Increase count if buffer is not yet full
            }
            data[head] = value;          // Insert new data point at the head
            head = (head + 1) % maxSize; // Move head to the next empty spot
        }

        // Function to print the current contents of the buffer for verification
        void printBuffer()
        {
            std::cout << "Current Buffer: ";
            for (int i = 0; i < count; i++)
            {
                int index = (tail + i) % maxSize; // Calculate the actual index
                std::cout << data[index] << " ";
            }
            std::cout << std::endl;
        }

        // print buffer size
        void printSize()
        {
            std::cout << "Buffer Size: " << count << std::endl;
        }

        // get data
        float *getData()
        {
            return data;
        }
        // Function to calculate the average of the absolute values of the last 50 data points
        float getAbsAvg()
        {
            float sum = 0;
            for (int i = 0; i < 50; i++)
            {
                int index = (head - i) % maxSize;
                sum += abs(data[index]);
            }
            return sum / 50;
        }

        // get datapoint with index based off of tail head offset
        float getDataPoint(int index)
        {
            return data[(tail + index) % maxSize];
        }
    };

    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Instantiate circular buffer object
    CircularBuffer buffer(1000);

    int count = 0;
    // instanciate led1
    DigitalOut led1(LED1);
    led1.write(0);
    float intensity;
    float freq;

    while (1)
    {
        // Guarentees that we won't need to active wait
        count++;

        // resets count if it goes over 1000
        if (count > 1000)
        {
            count = 0;
        }

        uint16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // Center the values around 0
        uint16_t si = int(65535 / 2);

        raw_gx = raw_gx - si;
        raw_gy = raw_gy - si;
        raw_gz = raw_gz - si;

        // Convert raw data to actual values using a scaling factor
        // -10 is the offset
        gx = ((float)raw_gx) * SCALING_FACTOR - 10;
        gy = ((float)raw_gy) * SCALING_FACTOR - 10;
        gz = ((float)raw_gz) * SCALING_FACTOR - 10;

        // Combine all three vectors into a single vector while maintining negative values
        float g = gy + gx + gz;
        // printf("g: %f\n", g);
        buffer.addData(g);

        // detect frequency

        // Print the current buffer size every 100 iterations
        // if (count % 100 == 0)
        // {
        //     printf("freq: %f\n", freq);
        // }
        thread_sleep_for(1);
        intensity = buffer.getAbsAvg();
        freq = oscillations_count(buffer.getData());
        controlLight(intensity, led1, count, freq);
        // printf("intensity: %f\n", intensity);
    }
}
