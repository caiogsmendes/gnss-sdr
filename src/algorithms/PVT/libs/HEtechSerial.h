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


    typedef struct {
        int fd;
        int state;
        int running;
        int flags;
        char txbuff[BUFF_SIZE];
        char bufftx;
        uint8_t rxbuff[BUFF_SIZE];
        uint8_t buffrx;
        int start, end;
        pthread_t rx_thread;
        pthread_t tx_thread;
        struct termios tty;
        struct pollfd ufds;
    } serial_s_t;


// serial_s_t HEserial_connect(const char*, int);
serial_s_t HEserial_connect(const char *, int, int);
// int HEserial_envio(serial_s_t*, uint8_t*);
int HEserial_envio(serial_s_t* comm, uint8_t* msg, int* tam);
// int HEserial_envio(serial_s_t* comm, char* msg);
int HEserial_leitura(serial_s_t*, uint8_t*);
uint8_t HEserial_leitura_byte(serial_s_t*);
// char HEserial_leitura_byte(serial_s_t*, char*);
void HEserial_disconnect(serial_s_t*);
int serial4send(uint8_t*, int*);
// int serial4send(double*);
int serial4read(uint8_t*);
void serial4readByte(uint8_t*);
int enviaar(char *msg);

// Utils
void Hex2IntegerAlt(uint32_t*, uint8_t*);
void Double2Hex(uint8_t*, const double*);
void Double2Hexx(uint8_t*, double); 
void Hex2Double(double*, uint8_t*);
void Hex2Float(float *, uint8_t *);
void Float2Hex(uint8_t *, const float *);
void Float2Hexx(uint8_t *, float);
void Hex2Integer(uint32_t *, uint8_t *);
void Integer2Hex(uint8_t *, const uint32_t *);
void Integer2Hexx(uint8_t *, const uint32_t);
void Hex2char(char *, uint8_t *);
void char2Hex(uint8_t *, const char *);
void msgPrep(uint8_t*, int);

#ifdef __cplusplus
}
#endif



#endif




    // void serial_envio(const char *device, int flags, char *msg); 
    // void serial_envioByte(const char *device, int flags, double *msg);
    // int serial_leitura(/*int fd2/*const char* device2,*/ unsigned char *msg);
    // void serial4send(char *data);

// External Functions
    // void HEserial_connect();
    // void HEserial_s_tend();
    // void HEserial_put();
    // void HEserial_get();
    // void HEserial_s_tet();
    // void HEserial_available();
    // void HEserial_close();

// Internal Functions
    //  void HEserial_dataListener();
    //  void HEserial_rx_callback();
    //  void HEserial_s_tx_callback();
    //  void HEserial_s_ttart();
    //  void HEserial_s_ttop();
    //  void HEserial_buffer_get();
    //  void HEserial_buffer_set();
    //  void HE_serial_buffer_available();
