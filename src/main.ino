#include <Servo.h>
#include <PPMReader.h>

// Loop delay
#define LOOP_DELAY 20      // ms

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
#define RC_PIN 3            // D3
#define CHANNEL_AMOUNT 6    // number of PPM channel
PPMReader ppm(RC_PIN, CHANNEL_AMOUNT);

// Initialize actuators outputs
#define THRUSTER_PIN 5      // D5
#define D_FIN_PIN 6         // D6
#define P_FIN_PIN 9         // D9
#define S_FIN_PIN 10        // D10

Servo thruster;
Servo d_fin;
Servo p_fin;
Servo s_fin;

// Manual mode threshold
#define MANUAL_THRESHOLD 1750
bool manual=false;

// Temporary serial read buffer 
uint8_t read_cursor = 0;
uint8_t temporary_read_serial_buffer[9];

// Read serial buffer
uint8_t read_serial_buffer[8];

// RC read buffer
uint8_t read_rc_buffer[12];

// Initialize serial read and write buffers
uint8_t write_buffer[8];

void write_actuators() {
    thruster.writeMicroseconds(write_buffer[0]*255+write_buffer[1]);
    d_fin.writeMicroseconds(write_buffer[2]*255+write_buffer[3]);
    p_fin.writeMicroseconds(write_buffer[4]*255+write_buffer[5]);
    s_fin.writeMicroseconds(write_buffer[6]*255+write_buffer[7]);
}

void setup() {
    // Serial communication with raspberry
    Serial.begin(115200);

    // Servo creation
    thruster.attach(THRUSTER_PIN);
    d_fin.attach(D_FIN_PIN);
    p_fin.attach(P_FIN_PIN);
    s_fin.attach(S_FIN_PIN);

    // PPM tuning
    ppm.minChannelValue = 1000;     // (us)
    ppm.maxChannelValue = 2000;     // (us)
    ppm.channelValueMaxError = 40;  // (us)
    ppm.blankTime = 2500;           // (us)

    // Filling read buffers to neutral commands
    for (uint8_t i=0; i<4; ++i) {
        read_serial_buffer[2*i] = highByte(1500);
        read_serial_buffer[2*i+1] = lowByte(1500);
    }
    for (uint8_t i=0; i<5; ++i) {
        read_rc_buffer[2*i] = highByte(1500);
        read_rc_buffer[2*i+1] = lowByte(1500);
    }
}

void loop() {
    // Reading PPM RC Receiver into write buffer
    for (uint8_t channel = 0; channel < CHANNEL_AMOUNT; ++channel) {
        uint16_t value = ppm.latestValidChannelValue(channel, 1500);
        read_rc_buffer[2*channel] = highByte(value);
        read_rc_buffer[2*channel+1] = lowByte(value);
    }

    // Reading commands from serial
    while ((Serial.available()) and (read_cursor < 9)) {
        // Reading incomming char
        uint8_t inByte = Serial.read();
        temporary_read_serial_buffer[read_cursor] = inByte;
        read_cursor++;

        // If a complete frame has been recevied
        if (read_cursor > 8) {
            // Put the read cursor at the beginning
            read_cursor = 0;

            // If the frame is finishing by '\n' then copying the temporary serial buffer to the read_serial_buffer
            if (temporary_read_serial_buffer[8] == '\n') {
                memcpy(read_serial_buffer, temporary_read_serial_buffer, 8);
            }
            break;
        }
    }

    // Writing commands on actuators depending on the manual or automatic mode
    uint16_t manual_value = read_rc_buffer[0]*255+read_rc_buffer[1];
    if (manual_value > MANUAL_THRESHOLD) {
        memcpy(write_buffer, read_rc_buffer+4, 8);
    }
    else {
        memcpy(write_buffer, read_serial_buffer, 8);
    }
    write_actuators();

    // Writing actuators state on the serial port
    Serial.write(write_buffer, 8);

    // Writing RC Receiver state
    Serial.write(read_rc_buffer, 12);

    // Serial new line
    Serial.write('\n');

    // Loop delay
    delay(LOOP_DELAY);
}