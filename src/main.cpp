#include <mbed.h>
#include <rtos.h>
#include <stdint.h>
#include <string>
#include <cmath>

#include "crc.cpp"

typedef uint8_t byte;
typedef uint16_t byte2;

#define HALF_WRITING_PERIOD 5 

using namespace std;

InterruptIn input_pin(p6);
DigitalOut output_pin(p5);

Serial pc(USBTX, USBRX);
Timer preamble_interval_timer;
Thread read_th;
Thread write_th;

typedef struct {
    char str[128];
} data_t;
Mail<data_t, 16> output_mail;
Mail<data_t, 16> input_mail;

int preamble_total_time = 0;
int preamble_bit_ctn = 0;

enum DataLinkState {
    preamble,
    preamble_error,
    start_packet,
    type_n_flags,
    length,
    data,
    crc,
    end_packet
};
DataLinkState read_state = preamble;

void rising_edge() {
    if(preamble_bit_ctn == 0) {
        preamble_total_time = 0;
        preamble_interval_timer.start();
        preamble_interval_timer.reset();        
    } else {
        preamble_total_time += preamble_interval_timer.read_ms();
        // printf("RISING -- %d\n\r", preamble_interval_timer.read_ms());
        preamble_interval_timer.reset();
    }
    

    preamble_bit_ctn++;
}

void falling_edge() {
    preamble_total_time += preamble_interval_timer.read_ms();
    // printf("FALLING -- %d\n\r", preamble_interval_timer.read_ms());
    preamble_interval_timer.reset();

    preamble_bit_ctn++;

    if(preamble_bit_ctn == 8) {
        // preamble done
        preamble_bit_ctn = 0;
        read_state = start_packet;
        read_th.signal_set(0x1);
    }

}

void _read_enqueue_data() {    
    char data_buffer[81] = {0};
    char packet_buffer[2 + 81] = {0};

    while(true) {
        input_pin.rise(rising_edge);
        input_pin.fall(falling_edge);
        
        // Wait for end of preamble
        pc.printf("[DEBUG] Waiting for new msg\n\r");                
        Thread::signal_wait(0x1);

        input_pin.rise(NULL);
        input_pin.fall(NULL);

        pc.printf("[DEBUG] Incoming msg\n\r");        

        // Periode
        int read_period = round(preamble_total_time/7.0f);
        
        int half_read_period = read_period/2;
        int half_read_period_offset = min((int) (0.2 * half_read_period), 1);
        // pc.printf("READ PREAMBLE SUCCESS %dms -- %d\n\r", half_read_period, half_read_period_offset);
        
        Thread::wait(half_read_period + half_read_period_offset);        
        // Align our self on the signal and read a little after so we are not on the transition
        /*
            *********                *********
              ^     |                |  
              |     |                |  
              1     |                |
                    |                |
                    |                |
                    |                |
                    |                |
                    ******************
                                ^
                                |
                                0    
        */

        int read_value = 0;
        int tmp_value = 0;
        int ctn = 0;
        int data_length = 0;
        int data_index = 0;
        bool continue_reading = true;
        
        preamble_interval_timer.reset();

        // Start reading data
        while(continue_reading) {
            read_value = input_pin.read();

            // pc.printf("LOOP %d\n\r", preamble_interval_timer.read_ms());
            // preamble_interval_timer.reset();

            switch(read_state) {
                case start_packet: { // 8
                    
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    // pc.printf("bit %d -- %d\n\r", ctn, preamble_interval_timer.read_ms());
                    // preamble_interval_timer.reset();

                    if(ctn == 8) {
                        if(tmp_value == 0b01111110) {
                            // DONE
                            tmp_value = 0;
                            ctn = 0;

                            read_state = type_n_flags;
                            // pc.printf("READ start_packet OK\n\r");
                        } else {
                            // ERROR
                            pc.printf("ERROR start_packet %.2x -- %.2x\n\r", tmp_value, 0b01111110);
                            
                            read_state = preamble;
                            continue_reading = false;
                        }
                    }

                    break;
                }
                case type_n_flags: { // 8
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    if(ctn == 8) {
                        tmp_value = 0;
                        ctn = 0;

                        read_state = length;
                        // pc.printf("READ type_n_flags OK\n\r");
                    }

                    break;
                }
                case length: { // 8
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    if(ctn == 8) {
                        data_length = tmp_value;

                        tmp_value = 0;
                        ctn = 0;

                        read_state = data;
                        // pc.printf("READ length OK\n\r");
                        // preamble_interval_timer.reset();
                    }

                    break;
                }
                case data: { // data_bit_length
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    // New byte
                    if(ctn % 8 == 0) {
                        data_buffer[data_index] = (char) tmp_value;
                        data_index++;
                        tmp_value = 0;

                        // pc.printf("READ data %d -- %dms\n\r", data_index, preamble_interval_timer.read_ms());
                        // preamble_interval_timer.reset();
                    }

                    if(data_index == data_length) {
                        tmp_value = 0;
                        ctn = 0;

                        read_state = crc;

                        // for(int i=0; i<data_length; i++) {
                        //     pc.printf("%c", data_buffer[i]);
                        // }

                        // pc.printf("\n\r");
                        // pc.printf("READ data OK\n\r");
                    }

                    break;
                }
                case crc: { // 16
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    if(ctn == 16) {

                        packet_buffer[0] = 0; //type_n_flags
                        packet_buffer[1] = data_length; // length
                        strncpy(packet_buffer + 2, data_buffer, data_length); // data

                        int crc = (int) compute_crc16(packet_buffer, 2 + data_length);
                        if(tmp_value != crc) {
                            // ERROR
                            read_state = preamble;
                            continue_reading = false;
                            pc.printf("BAD CRC\n\r");
                            break;
                        }
                        // Else, crc is good

                        tmp_value = 0;
                        ctn = 0;

                        read_state = end_packet;
                    }

                    break;
                }
                case end_packet: { // 8
                    tmp_value = (tmp_value << 1) + read_value;
                    ctn++;

                    if(ctn == 8) {
                        if(tmp_value == 0b01111110) {
                            // DONE
                            tmp_value = 0;
                            ctn = 0;

                            data_t *mail = input_mail.alloc();
                            strncpy(mail->str, data_buffer, data_length);

                            input_mail.put(mail);

                            // pc.printf("READ END OK\n\r");
                        } else {
                            // ERROR
                        }

                        read_state = preamble;
                        continue_reading = false;
                    }

                    break;
                }
                default:
                    break;
            }

            // Wait for the next bit setup
            Thread::wait(read_period + 1); // +1 to patch weird behavior of the wait
        }
    }
}

void _write_1() {
    output_pin = 1;
    Thread::wait(HALF_WRITING_PERIOD + 1);// +1 to patch weird behavior of the wait

    output_pin = 0;
    Thread::wait(HALF_WRITING_PERIOD);
}

void _write_0() {
    output_pin = 0;
    Thread::wait(HALF_WRITING_PERIOD + 1);// +1 to patch weird behavior of the wait

    output_pin = 1;
    Thread::wait(HALF_WRITING_PERIOD);
}

void _write_byte(byte val) {
    for(signed char i=7; i>=0; i--) {
        if((val >> i) & 0b00000001) {
            _write_1();
        } else {
            _write_0();
        }
    }
}

void _manchester_encode_and_send(string str) {
    // Preamble
    _write_byte(0b01010101);

    // Start
    _write_byte(0b01111110);

    // Header
        // Type + Flags
        _write_byte(0b00000000);

        // Length
        _write_byte(str.length());

    // Body
    for(byte i=0; i<str.length(); i++) {
        _write_byte(str[i]);
    }

    // CRC (include Header + Body)
    char crc_str[2 + str.length()] = {0, (char) str.length()};
    strcat(crc_str + 2, str.c_str());

    byte2 crc = compute_crc16(crc_str, 2 + str.length());
    _write_byte(crc >> 8); // high
    _write_byte((byte) crc); // low

    //pc.printf("%.4x -- %.2x -- %.2x", crc, (crc >> 8), (byte) crc);

    // End
    _write_byte(0b01111110);
}

void _write_dequeue_data() {
    while (true) {
        osEvent evt = output_mail.get();
        if (evt.status == osEventMail) {
            data_t *mail = (data_t*) evt.value.p;
            // pc.printf("_write_dequeue_data : %s\n\r", mail->str);

            _manchester_encode_and_send(mail->str);
            
            output_mail.free(mail);
        }
    }
}

void write(string str) {
    data_t *mail = output_mail.alloc();

    str.copy(mail->str, 80, 0);

    output_mail.put(mail);
}

void read_n_print() {    
    while(true) {
        osEvent evt = input_mail.get();
        if (evt.status == osEventMail) {
            data_t *mail = (data_t*)evt.value.p;
            
            pc.printf("READ : %s\n\r\n\r", mail->str);
            
            input_mail.free(mail);
        }
    }
}

int main() {
    pc.baud(460800);
    pc.printf("\n\r========= START MBED =========\n\r");

    read_th.start(_read_enqueue_data);
    write_th.start(_write_dequeue_data);

    write("123456789123456789");
    write("asdfsadfasdfasdfas");
    write("Sed nec purus in justo fringilla suscipit.");
    write("Lorem ipsum dolor sit amet, consectetur adipiscing elit.");
    
    read_n_print();

    Thread::wait(osWaitForever);
}