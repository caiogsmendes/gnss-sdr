#ifndef _FRONTEND_H_
#define _FRONTEND_H_

#include <unistd.h>
#include <stdint.h>
#include <gpiod.h>
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
// class frontend{

// public:
// frontend();
// ~frontend();
void frontend_init(void);
void frontend_SOFT_rst(void);
void frontend_HARD_rst(void);
// int frontend_WTD_init(void);
// int frontend_WTD_open(const char*);
// int frontend_WTD_hwfeed(void);


// ##### GPIO #####
// Obs: 2 gpio's
// SODIMM 127 ------> Reset
// SODIMM 129 ------> Power UP
// int line_value = 0;
// int line_value_pwr = 1;
// int line;
// int ret;
// char chip[10];
// unsigned int offset;
// const char* gpiochip4 = "4";
// const char* gpiochip5 = "5";
// int SODIMM127 = 5;
// int SODIMM129 = 3;

// private:
// int line_value;
// int line_value_pwr;
// int line;
// int ret;
// char chip[10];
// unsigned int offset;
// const char* gpiochip4;
// const char* gpiochip5;
// int SODIMM127;
// int SODIMM129;
// static int api_watchdog_fd = -1;
// };
#endif