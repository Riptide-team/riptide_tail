#include <NMEAParser.h>
#include <PPMReader.h>
#include <Servo.h>
#include <string.h>


#define ms() 96*millis()


// Loop delay
#define LOOP_DELAY 20      // ms
uint32_t t0_loop_time;

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

// PWM Neutral in us
#define PWM_NEUTRAL 1500

// Manual mode threshold
#define MANUAL_THRESHOLD 1750
bool manual=false;

// Multiplexer RC Timer button pressed
bool rc_timer_button_pressed=false;

// NMEA Parser declared with 1 handler for RHACT sentences
NMEAParser<1> parser;

// Read serial buffer
uint16_t read_serial_buffer[4];

// Read stopping criterion to not block the loop
bool complete_frame = false;
uint8_t read_cursor = 0;
uint8_t read_cursor_max = 128;

// Last time command received from serial
#define SERIAL_TIMEOUT 3000
uint32_t time_read_serial = 0;

// RC read buffer
uint16_t read_rc_buffer[CHANNEL_AMOUNT];

// Initialize serial write buffers
uint16_t write_buffer[4];

// Maximum time for multiplexer in ms (compared to millis() to know how to multiplex RC and Raspberry Pi)
unsigned long t_multiplexer_max;

void write_actuators() {
    thruster.writeMicroseconds(write_buffer[0]);
    d_fin.writeMicroseconds(write_buffer[1]);
    p_fin.writeMicroseconds(write_buffer[2]);
    s_fin.writeMicroseconds(write_buffer[3]);
}

void RHACT_handler() {
    // Set complete_frame to true to perform a write after the parsing of this NMEA sentence
    complete_frame = true;

    // If the number of arguments is correct
    if (parser.argCount() == 4) {

        // Try reading each arguments
        bool read_flag = true;
        uint16_t temporary_read_buffer[4];

        for (uint8_t i = 0; i<4; ++i) {
            int value;
            read_flag &= parser.getArg(i, value);
            temporary_read_buffer[i] = value;
        }

        // If there is no errors, copying it in read_buffer
        if (read_flag) {
            memcpy(read_serial_buffer, temporary_read_buffer, 4*sizeof(uint16_t));
        }
        time_read_serial = ms();
    }
}

uint8_t CRC(char* buffer, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; ++i){ // XOR every character
      crc = crc ^ buffer[i] ;  // compute CRC
    }
    return crc;
}

void CheckSum(uint8_t &checksum, char c) {
    checksum = checksum ^ c;  // compute CRC
}

char toUpper(char c) {
    if(c >= 'a' && c <= 'z') c += 'A' - 'a';
    return c;
}


void RTRCR_write() {
    // Header
    uint8_t checksum = 105; 
    Serial.print("$RTRCR,");

    for (uint8_t channel = 0; channel < CHANNEL_AMOUNT; ++channel) {
        // Printing Channel
        Serial.print(read_rc_buffer[channel]);

        // Computing CheckSum
        char channel_buffer[4];
        itoa(read_rc_buffer[channel], channel_buffer, 10);
        for (uint8_t i=0; i<4; ++i) {
            CheckSum(checksum, channel_buffer[i]);
        }

        // Adding ',' except for the last loop 
        if (channel < CHANNEL_AMOUNT-1) {
            Serial.print(',');
            CheckSum(checksum, ',');
        }
    }

    // Printing *
    Serial.print("*");

    // Printing CRC and ending the frame
    char crc_buffer[2];
    itoa (checksum, crc_buffer, 16);
    Serial.print(toUpper(crc_buffer[0]));
    Serial.print(toUpper(crc_buffer[1]));

    // Printing \r\n
    Serial.print("\r\n");
}

void RTACT_write() {
    // Header
    uint8_t checksum = 124; 
    Serial.print("$RTACT,");

    // Building NMEA sentence
    for (uint8_t channel = 0; channel < 4; ++channel) {
        // Printing channel value
        Serial.print(write_buffer[channel]);

        // Computing checksum
        char channel_buffer[4];
        itoa(write_buffer[channel], channel_buffer, 10);
        for (uint8_t i=0; i<4; ++i) {
            CheckSum(checksum, channel_buffer[i]);
        }

        // Adding ',' except for the last loop 
        if (channel < 3) {
            Serial.print(',');
            CheckSum(checksum, ',');
        }
    }

    // Printing *
    Serial.print("*");

    // Printing CRC and ending the frame
    char crc_buffer[2];
    itoa (checksum, crc_buffer, 16);
    Serial.print(toUpper(crc_buffer[0]));
    Serial.print(toUpper(crc_buffer[1]));

    // Printing \r\n
    Serial.print("\r\n");
}

void RTMPX_write() {
    // Header
    uint8_t checksum = 111; 
    Serial.print("$RTMPX,");

    // Computing remaining time
    float remaining_time = ((float)t_multiplexer_max - (float)ms()) / 1000.f;

    // Printing first argument (0 for RC / 1 for RH)
    if (manual or remaining_time <= 0.) {
        Serial.print('0');
        CheckSum(checksum, '0');
    }
    else {
        Serial.print('1');
        CheckSum(checksum, '1');
    }
    
    // Printing ','
    Serial.print(',');
    CheckSum(checksum, ',');

    // Printing remaining seconds
    if (remaining_time >= 0.f) {
        // Computing seconds, dsecs, csecs and msecs
        int secs = int(remaining_time);
        int msecs = int(remaining_time*1000 - secs*1000);

        // Putting int part
        uint8_t i = 0;
        char time_buffer[3];
        itoa(secs, time_buffer, 10);
        while ((i<3) and (time_buffer[i] != '\0')) {
            Serial.print(time_buffer[i]);
            CheckSum(checksum, time_buffer[i]);
            i++;
        }

        Serial.print('.');
        CheckSum(checksum, '.');

        // Putting int part
        i = 0;
        itoa(msecs, time_buffer, 10);
        while ((i<3) and (time_buffer[i] != '\0')) {
            Serial.print(time_buffer[i]);
            CheckSum(checksum, time_buffer[i]);
            i++;
        }
    }
    else {
        Serial.print("0.000");
        CheckSum(checksum, '0');
        CheckSum(checksum, '.');
        CheckSum(checksum, '0');
        CheckSum(checksum, '0');
        CheckSum(checksum, '0');
    }

    // Printing *
    Serial.print("*");

    // Printing CRC and ending the frame
    char crc_buffer[2];
    itoa(checksum, crc_buffer, 16);
    Serial.print(toUpper(crc_buffer[0]));
    Serial.print(toUpper(crc_buffer[1]));

    // Printing \r\n
    Serial.print("\r\n");
}


void setup() {
    // Serial communication with raspberry
    Serial.begin(115200);

    // Add handler to the parser
    parser.addHandler("RHACT", RHACT_handler);

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
        read_serial_buffer[i] = PWM_NEUTRAL;
    }
    for (uint8_t i=0; i<CHANNEL_AMOUNT; ++i) {
        read_rc_buffer[i] = PWM_NEUTRAL;
    }

    // Init t0
    t_multiplexer_max = ms();
}

void loop() {
    // Getting t0_loop time
    t0_loop_time = ms();

    // Reading PPM RC Receiver into write buffer
    for (uint8_t channel = 0; channel < CHANNEL_AMOUNT; ++channel) {
        read_rc_buffer[channel] = ppm.latestValidChannelValue(channel, PWM_NEUTRAL);
    }

    // Write RTRCR NMEA sentences
    // RTRCR_write();

    // Reading commands from serial
    while ((read_cursor < read_cursor_max) and (!complete_frame) and (Serial.available())) {
        // Reading incomming char into the NMEA parser
        parser << Serial.read();
        read_cursor++;
    }

    // Reseting the complete frame variable to parse sentences
    complete_frame = false;
    read_cursor = 0;

    // Checking if the manual button is triggered
    uint16_t manual_value = read_rc_buffer[0];
    if (manual_value > MANUAL_THRESHOLD) {
        manual = true;
    }
    else {
        manual = false;
    }

    // Getting the current time
    unsigned long current_time = ms();

    // Getting time from RC if provided
    uint16_t time_value = read_rc_buffer[1];
    if (time_value > PWM_NEUTRAL) {
        rc_timer_button_pressed = true;
    }
    else {
        // If the button was previously pressed
        if (rc_timer_button_pressed) {
            // Setting end mission time in ms
            t_multiplexer_max = ((unsigned long)current_time + 200ul * ((unsigned long)time_value - 1500ul));
            rc_timer_button_pressed = false;
        }
    }

    // Writing commands on actuators depending on the manual or automatic mode and remaining multiplexer time
    if (manual) {
        memcpy(write_buffer, read_rc_buffer+2, 4*sizeof(uint16_t));
    }
    else {
        // If the current time is over the multiplexer time or the last time a command was received is greater than SERIAL_TIMEOUT
        if ((current_time - time_read_serial > SERIAL_TIMEOUT) or (current_time > t_multiplexer_max)) {
            // Setting all servos to neutral
            for (uint8_t i=0; i<4; ++i) {
                write_buffer[i] = PWM_NEUTRAL;
            }
        }
        else {
            // Write serial received commands to actuators
            memcpy(write_buffer, read_serial_buffer, 4*sizeof(uint16_t));
        }
    }

    write_actuators();

    // Write RTACT NMEA sentences
    RTACT_write();

    // Write RTMPX NMEA sentences
    // RTMPX_write();

    // Loop delay
    while(ms() < t0_loop_time + LOOP_DELAY) {
        delayMicroseconds(1);
    }
}