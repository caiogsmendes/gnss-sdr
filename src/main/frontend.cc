#include "frontend.h"
// extern "C"
// {
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>

    void frontend_init(void)
    {
        // ##### GPIO #####
        // Obs: 2 gpio's
        // SODIMM 127 ------> Reset
        // SODIMM 129 ------> Power UP
        int line_value = 0;
        int line;
        int ret;
        char chip[10];
        unsigned int offset;
        const char* gpiochip4 = "4";
        const char* gpiochip5 = "5";
        int SODIMM127 = 5;
        int SODIMM129 = 3;

        snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip4);
        offset = SODIMM127;
        gpiod_ctxless_set_value(chip, offset, 0, false, "gpio-toggle", NULL, NULL);  // Lower the hackRF RST_pin

        // snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip5);
        // offset = SODIMM127;
        // gpiod_ctxless_set_value(chip, offset, 0, false, "gpio-toggle", NULL, NULL); // Lower the hackRF RST_pin
    }
// }