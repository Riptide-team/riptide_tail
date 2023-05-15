#include <NMEAParser.h>
#include <PPMReader.h>
#include <Servo.h>
#include <string.h>


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
bool complete_frame = false;

// RC read buffer
uint16_t read_rc_buffer[CHANNEL_AMOUNT];

// Initialize serial read and write buffers
uint16_t write_buffer[4];

// Maximum time for multiplexer in ms (compared to millis() to know how to multiplex RC and Raspberry Pi)
uint32_t t_multiplexer_max;
// float remaining_time = 0;

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
            memcpy(read_serial_buffer, temporary_read_buffer, 4);
        }
    }
}

byte CRC(char* buffer, uint8_t n) {
    byte crc = 0;
    for (uint8_t i = 1; i < n; ++i){ // XOR every character
      crc = crc ^ buffer[i] ;  // compute CRC
    }
    return crc;
}


void RTRCR_write() {
    char rtrcr[128] = "\0";

    // Building NMEA sentence
    strcat(rtrcr, "RTRCR,");
    for (uint8_t channel = 0; channel < CHANNEL_AMOUNT; ++channel) {
        char channel_buffer[4];
        itoa(read_rc_buffer[channel], channel_buffer, 10);
        strcat(rtrcr, channel_buffer);
        strcat(rtrcr, ",");
    }
    uint8_t length = strlen(rtrcr);
    rtrcr[length-1] = '\0';

    // Computing CRC
    byte crc = CRC(rtrcr, length-1);

    Serial.write("$");
    Serial.write(rtrcr);
    Serial.write("*");
    Serial.write(crc);
}

void RTACT_write() {
    char rtact[128] = "\0";

    // Building NMEA sentence
    strcat(rtact, "RTACT,");
    for (uint8_t channel = 0; channel < 4; ++channel) {
        char channel_buffer[4];
        itoa(write_buffer[channel], channel_buffer, 10);
        strcat(rtact, channel_buffer);
        strcat(rtact, ",");
    }
    uint8_t length = strlen(rtact);
    rtact[length-1] = '\0';

    // Computing CRC
    byte crc = CRC(rtact, length-1);

    Serial.write("$");
    Serial.write(rtact);
    Serial.write("*");
    Serial.write(crc);
}

void RTMPX_write() {
    char rtmpx[128] = "\0";

    // Building NMEA sentence
    strcat(rtmpx, "RTMPX,");
    if (manual) {
        strcat(rtmpx, "0,");
    }
    else {
        strcat(rtmpx, "1,");
    }
    
    char time_buffer[7];
    dtostrf((t_multiplexer_max - millis()), 0, 3, time_buffer);
    strcat(rtmpx, time_buffer);

    // Computing CRC
    uint8_t length = strlen(rtmpx);
    byte crc = CRC(rtmpx, length-1);

    Serial.write("$");
    Serial.write(rtmpx);
    Serial.write("*");
    Serial.write(crc);
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
    t_multiplexer_max = millis();
}

void loop() {
    // Reading PPM RC Receiver into write buffer
    for (uint8_t channel = 0; channel < CHANNEL_AMOUNT; ++channel) {
        read_rc_buffer[channel] = ppm.latestValidChannelValue(channel, PWM_NEUTRAL);
    }

    // Write RTRCR NMEA sentences
    RTRCR_write();

    // Reading commands from serial
    while ((!complete_frame) and (Serial.available())) {
        // Reading incomming char into the NMEA parser
        parser << Serial.read();
    }

    // Checking if the manual button is triggered
    uint16_t manual_value = read_rc_buffer[0];
    if (manual_value > MANUAL_THRESHOLD) {
        manual = true;
    }
    else {
        manual = false;
    }

    // Getting the current time
    uint32_t current_time = millis();

    // Getting time from RC if provided
    uint16_t time_value = read_rc_buffer[1];
    if (time_value > PWM_NEUTRAL) {
        rc_timer_button_pressed = true;
    }
    else {
        // If the button was previously pressed
        if (rc_timer_button_pressed) {
            // Setting end mission time in ms
            t_multiplexer_max = current_time + 2 * (time_value - 1500);
        }
        rc_timer_button_pressed = false;
    }

    // Writing commands on actuators depending on the manual or automatic mode and remaining multiplexer time
    if (manual or (current_time > t_multiplexer_max)) {
        memcpy(write_buffer, read_rc_buffer+3, 4);
    }
    else {
        memcpy(write_buffer, read_serial_buffer, 4);
    }

    write_actuators();

    // Write RTACT NMEA sentences
    RTACT_write();

    // Write RTMPX NMEA sentences
    RTMPX_write();

    // Reseting the complete frame variable to parse sentences
    complete_frame = false;

    // Loop delay
    delay(LOOP_DELAY);
}