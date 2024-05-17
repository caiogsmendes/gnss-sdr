#ifndef _FRONTEND_H_
#define _FRONTEND_H_

#include <unistd.h>
#include <stdint.h>
// #include "RGL.h"
// ################     Experimental      ######################
// extern "C"
// {
//     typedef struct Monitor
//     {
//         uint8_t RGL_address;
//         uint8_t cmd; 
//         uint8_t CRC;
//     }RGL_ctrl_handler;

// }


void frontend_init(void);
void frontend_rst(void);


#endif