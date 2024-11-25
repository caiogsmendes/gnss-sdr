/*!
 * \file gnss_synchro_monitor.cc
 * \brief Implementation of a receiver monitoring block which allows sending
 * a data stream with the receiver internal parameters (Acq/Trk/Tlm/PVT/Sync)
 * Over the serial port.
 * \author Caio Guedes de Souza Mendes,MSc. caio.mendes@horuseye.com.br
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gnss_serial_monitor.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_synchro.h"
#include <algorithm>
#include <iostream>
#include <utility>

#include "gps_ephemeris.h"
#include <gpiod.h>

gnss_serial_monitor_sptr gnss_serial_make_monitor(int n_channels,
    // int decimation_factor,
    // const std::vector<std::string>& udp_ports,
    // const std::vector<std::string>& udp_addresses,
    // bool enable_protobuf
    std::string dev_serial,
    int baudrate
    )
{
    return gnss_serial_monitor_sptr(new gnss_serial_monitor(n_channels,
        // decimation_factor,
        // udp_ports,
        // udp_addresses,
        // enable_protobuf
        dev_serial,
        baudrate
        )
        );
}

// In sizeof(Gnss_Synchro), we highjack
gnss_serial_monitor::gnss_serial_monitor(int n_channels,
    // int decimation_factor,
    // const std::vector<std::string>& udp_ports,
    // const std::vector<std::string>& udp_addresses,
    // bool enable_protobuf
    std::string dev_serial,
    int baudrate
    )
    : gr::block("gnss_serial_monitor",
          gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro)), 
          gr::io_signature::make(0, 0, 0)),
        //   gr::io_signature::make(n_channels, n_channels, sizeof(Gps_Ephemeris)), 
        //   gr::io_signature::make(0, 0, 0)),
      count(0),
      d_nchannels(n_channels),
    //   d_decimation_factor(decimation_factor)
      d_dev_serial(dev_serial),
      d_baudrate(baudrate)
{
    //
    //udp_sink_ptr = std::make_unique<Gnss_Synchro_Udp_Sink>(udp_addresses, udp_ports, enable_protobuf);
    //

}


void gnss_serial_monitor::forecast(int noutput_items __attribute__((unused)), gr_vector_int& ninput_items_required)
{
    for (int32_t channel_index = 0; channel_index < d_nchannels; channel_index++)
        {
            // Set the required number of inputs to 0 so that a lone input on any channel can be pushed to UDP
            ninput_items_required[channel_index] = 0;
        }
}


int gnss_serial_monitor::general_work(int noutput_items __attribute__((unused)), gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items __attribute__((unused)))
{
    // // Get the input buffer pointer
    const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);
    // // const auto** in1 = reinterpret_cast<const Gps_Ephemeris**>(&input_items[0]);


    typedef struct gpiod_line gpiod_pin;
    typedef struct gpiod_line_event gpiod_pin_event;
    struct gpiod_chip *chip;
    gpiod_pin *pin;
    const char bank[] = "gpiochip2";
    int SODIMM_55 = 18;
    // int SODIMM_63;
    unsigned int line = SODIMM_55;

    timespec ts;
    ts.tv_nsec=100000000000;

    chip = gpiod_chip_open_by_name(&bank[0]);
    pin = gpiod_chip_get_line(chip, line);
    // gpiod_pin *input_pin;
    gpiod_pin_event event;
    // int pin_value = 0;
    int ret;
    ret = gpiod_line_request_rising_edge_events(pin, "gpio-test");
    int count = 0;
    // while (1)
    //     {
            // mtx.lock();
            // // gnss_synchro = pvt_ptr->get_gnss_observables();
            // // uint8_t *msgvec_ptr = pvt_ptr->get_msgvec_ptr();
            // do
            //     {
            //     }
            // while (counter <= bytess);
            // mtx.unlock();

            /* Waiting for an event on the input pin */
            gpiod_line_event_wait(pin, NULL);
            // else
            // {
            //     std::cout<<"Trigged"<<"\n";
            // }
            // gpiod_line_event_wait(pin, &ts);

            /* Reading next pending event from the GPIO pin */
            if (gpiod_line_event_read(pin, &event) != 0)
                {
                    std::cout<<"Error"<<"\n";
                }

            /* Checking if it is a rising event as previously defined */
            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
                {
                    std::cout << "Trigged" << "\n";
                }
            
            
            // else{
            //     std::cout<<""<<"\n";
            // }
            // continue;

            // std::cout << "Detected " << count++;
            // // int result = write(,&pvt_ptr->msgvec[0],bytes);
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // }


            // Loop through each input stream channel
            for (int channel_index = 0; channel_index < d_nchannels; channel_index++)
                {
                    // Loop through each item in each input stream channel
                    for (int item_index = 0; item_index < ninput_items[channel_index]; item_index++)
                        {
                            // Use the count variable to limit how many items are sent per channel
                            count++;
                            if (count >= 10)
                                {
                                    // Convert to a vector and write to the UDP sink
                                    // std::vector<Gnss_Synchro> stocks;
                                    // stocks.push_back(in[channel_index][item_index]);
                                    // udp_sink_ptr->write_gnss_synchro(stocks);

                                    // Reset count variable
                                    // count = 0;

                                    // Consume the number of items for the input stream channel
                                    consume(channel_index, ninput_items[channel_index]);
                                }
                        }
        }
    // // Not producing any outputs

    return 0;
}


 