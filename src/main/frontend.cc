#include "frontend.h"
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>


// frontend::frontend()
// {
//     // ##### GPIO #####
//     // Obs: 2 gpio's
//     // SODIMM 127 ------> Reset
//     // SODIMM 129 ------> Power UP
//     line_value = 0;
//     line_value_pwr = 1;
//     const char* gpiochip4 = "4";
//     const char* gpiochip5 = "5";
//     int SODIMM127 = 5;
//     int SODIMM129 = 3;
// }

// frontend::~frontend() {}

void frontend_init(void)
{
    // // ##### GPIO #####
    // // Obs: 2 gpio's
    // // SODIMM 127 ------> Reset
    // // SODIMM 129 ------> Power UP
    int line_value = 1;
    int line;
    int ret;
    char chip[10];
    unsigned int offset;
    const char* gpiochip4 = "4";
    const char* gpiochip5 = "5";
    int SODIMM127 = 5;
    int SODIMM129 = 3;

    // snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip4);
    // offset = SODIMM127;
    // gpiod_ctxless_set_value(chip, offset, 0, false, "gpio-toggle", NULL, NULL);  // Lower the hackRF RST_pin


    // snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip5);
    // offset = SODIMM127;
    // gpiod_ctxless_set_value(chip, offset, line_value, false, "gpio-toggle", NULL, NULL);

    // gpiod_ctxless_set_value(chip, offset, 0, false, "gpio-toggle", NULL, NULL);  // Lower the hackRF RST_pin
    // gpiod_ctxless_set_value(chip, SODIMM129, 0, false, "gpio-toggle", NULL, NULL);

    // if(fork()==0){
    // char* const argin[] = {"hackrf_info"};
    // char* cmd = "/usr/bin";
    // int result = execvp(cmd, argin);
    int result = system("/usr/bin/hackrf_info");
    // if (result < 0)
    //     {
    //         std::cout << "\33[31m" << "Front RF Initialization Failed!!! ... " << "\33[0m" << "\n";
    //     }
    // else{
    //     // perror("execvp");
    //     std::cout<<"\33[32m"<<"Front RF Initialized!!..." <<"\33[0m"<<"\n";
    // }
}

void frontend_SOFT_rst(void)
{
    int line_value = 0;
    int line;
    int ret;
    char chip[10];
    unsigned int offset;
    const char* gpiochip5 = "5";
    int SODIMM127 = 5;

    snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip5);
    offset = SODIMM127;

    gpiod_ctxless_set_value(chip, offset, ~line_value, false, "gpio-toggle", NULL, NULL);
    usleep(1000);
    gpiod_ctxless_set_value(chip, offset, ~line_value, false, "gpio-toggle", NULL, NULL);
}

void frontend_HARD_rst(void)
{
    int line_value_pwr = 1;
    int line;
    int ret;
    char chip[10];
    unsigned int offset;
    const char* gpiochip5 = "5";
    int SODIMM129 = 3;

    snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip5);
    offset = SODIMM129;

    gpiod_ctxless_set_value(chip, offset, ~line_value_pwr, false, "gpio-toggle", NULL, NULL);
    usleep(1000);
    gpiod_ctxless_set_value(chip, offset, ~line_value_pwr, false, "gpio-toggle", NULL, NULL);
}