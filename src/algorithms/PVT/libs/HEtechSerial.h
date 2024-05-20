//#include <string>
#ifndef _HETECHSERIAL_H_
#define _HETECHSERIAL_H_
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>   // I/O definitions

#include <unistd.h>  // Standart Symbolic Constants
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <string.h>  // String Manipulation Definitions
#include <errno.h>   // Error Code Definitions

//Experimental
// #include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/poll.h>

#define BUFF_SIZE 2048
#define POLL_TIMEOUT 200


    struct serial_s
    {
        int fd;
        int state;
        int running;
        int flags;
        char txbuff[BUFF_SIZE];
        char bufftx;
        char rxbuff[BUFF_SIZE];
        char buffrx;
        int start, end;
        pthread_t rx_thread;
        pthread_t tx_thread;
        struct termios tty;
        struct pollfd ufds;
    };


struct serial_s HEserial_connect(const char*, int);
int HEserial_envio(serial_s*, uint8_t*);
int HEserial_leitura(serial_s*, uint8_t*);
char HEserial_leitura_byte(serial_s*, uint8_t*);
void HEserial_disconnect(serial_s*);
int serial4send(uint8_t*);
int serial4read(uint8_t*);
void serial4readByte(uint8_t*);
int enviaar(char *msg);


#ifdef __cplusplus
}
#endif

void Double2Hex(uint8_t* output, double* input);

#endif




    // void serial_envio(const char *device, int flags, char *msg); 
    // void serial_envioByte(const char *device, int flags, double *msg);
    // int serial_leitura(/*int fd2/*const char* device2,*/ unsigned char *msg);
    // void serial4send(char *data);

// External Functions
    // void HEserial_connect();
    // void HEserial_send();
    // void HEserial_put();
    // void HEserial_get();
    // void HEserial_set();
    // void HEserial_available();
    // void HEserial_close();

// Internal Functions
    //  void HEserial_dataListener();
    //  void HEserial_rx_callback();
    //  void HEserial_tx_callback();
    //  void HEserial_start();
    //  void HEserial_stop();
    //  void HEserial_buffer_get();
    //  void HEserial_buffer_set();
    //  void HE_serial_buffer_available();
