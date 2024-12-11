/*!
 * \file gnss_synchro_monitor.cc
 * \brief Implementation of a receiver monitoring block which allows sending
 * a data stream with receiver internal parameters (Acq/Trk/Tlm/PVT/Sync)
 * over the serial port*.
 * \author Caio Guedes de Souza Mendes,MSc. caio.mendes@horuseye.com.br
 *
 * *in a more controlled manner
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

#include <gnuradio/io_signature.h>      // for io_signature
#include <pmt/pmt_sugar.h>              // for mp
#include <gnuradio/basic_block.h>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

#include "gps_ephemeris.h"
#include <gpiod.h>
#include "HEtechSerial.h"



class gnss_serial_monitor;

gnss_serial_monitor_sptr gnss_serial_make_monitor(
    int n_channels,
    std::string dev_serial,
    int baudrate
    )
{
    return gnss_serial_monitor_sptr(new gnss_serial_monitor(n_channels,dev_serial,baudrate));
}

gnss_serial_monitor::gnss_serial_monitor(
    int n_channels,
    std::string dev_serial,
    int baudrate)
    : gr::block("gnss_serial_monitor",
          gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro)),
          gr::io_signature::make(0, 0, 0)),
      d_gps_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Gps_Ephemeris>).hash_code()),
      count(0),
      d_nchannels(n_channels),
      d_dev_serial(dev_serial),
      d_baudrate(baudrate)
{   
    // Caio - Teste de recepção do stream de "telemetry"
    this->message_port_register_in(pmt::mp("telemetry"));
    this->set_msg_handler(pmt::mp("telemetry"),[this](auto&& PH1){msg_handler_telemetry(PH1);});

    // // Caio -> PVT Solution data message Port in
    // this->message_port_register_in(pmt::mp("pvtsol_to_serial_monitor"));
    // this->set_msg_handler(pmt::mp("pvtsol_to_serial_monitor"),[this](auto&& PH1){msg_handler_pvtsol(PH1);});

    // GPS Ephemeris data message port in
    this->message_port_register_in(pmt::mp("telemetry_to_serial_monitor"));
    this->set_msg_handler(pmt::mp("telemetry_to_serial_monitor"),[this](auto&& PH2) { msg_handler_telemetry_2(PH2); });

    std::string devv = "/dev/ttyUSB0";
    // comms = HEserial_connect(dev_serial.c_str(), B921600, O_RDWR | O_NDELAY | O_NOCTTY | O_NONBLOCK);
    comms = HEserial_connect(devv.c_str(), B921600, O_RDWR | O_NDELAY | O_NOCTTY | O_NONBLOCK);
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
    msgvec[0] = 0xd4;
    msgvec[1] = 0x4f;
    msgvec[2] = 3;

    std::cout<<"contagem: "<<++count<<"\n";

    // // Get the input buffer pointer
    const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);

    // for(const auto& i:gps_ephemeris_map){
    //     int aux = i.second.PRN;
    // }
    // int aux = gps_ephemeris_map.at(0).PRN;
    // if(gps_ephemeris_map.size()!=0){
    //     std::cout<<"Novo GPS Data"<<"\n";
    // }
    char buf[100];
    // int result = read(comms.fd,&buf[0],1);

    // if(result >= 1)
    // {contador++;std::cout<<"Trigged "<<contador<<"\n";}
    // typedef struct gpiod_line gpiod_pin;
    // typedef struct gpiod_line_event gpiod_pin_event;
    // struct gpiod_chip *chip;
    // gpiod_pin *pin;
    // const char bank[] = "gpiochip2";
    // int SODIMM_55 = 18;
    // // int SODIMM_63;
    // unsigned int line = SODIMM_55;

    // timespec ts;
    // ts.tv_nsec=100000000000;

    // chip = gpiod_chip_open_by_name(&bank[0]);
    // pin = gpiod_chip_get_line(chip, line);
    // // gpiod_pin *input_pin;
    // gpiod_pin_event event;
    // // int pin_value = 0;
    // int ret;
    // ret = gpiod_line_request_rising_edge_events(pin, "gpio-test");
    // int count = 0;
    // // while (1)
    // //     {
    //         // mtx.lock();
    //         // // gnss_synchro = pvt_ptr->get_gnss_observables();
    //         // // uint8_t *msgvec_ptr = pvt_ptr->get_msgvec_ptr();
    //         // do
    //         //     {
    //         //     }
    //         // while (counter <= bytess);
    //         // mtx.unlock();

    //         /* Waiting for an event on the input pin */
    //         gpiod_line_event_wait(pin, NULL);
    //         // else
    //         // {
    //         //     std::cout<<"Trigged"<<"\n";
    //         // }
    //         // gpiod_line_event_wait(pin, &ts);

    //         /* Reading next pending event from the GPIO pin */
    //         if (gpiod_line_event_read(pin, &event) != 0)
    //             {
    //                 std::cout<<"Error"<<"\n";
    //             }

    //         /* Checking if it is a rising event as previously defined */
    //         if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
    //             {
    //                 std::cout << "Trigged" << "\n";
    //             }
            
            
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

                                    // Reset count variable
                                    count = 0;

                                    // Consume the number of items for the input stream channel
                                    consume(channel_index, ninput_items[channel_index]);
                                }
                        }
                }
            // // Not producing any outputs
            return 0;
}

// int gnss_serial_monitor::get_msgvec_w_GAL(const Rtklib_Solver* const pvt_data)
// {
//     msgvec[0]=0xd4;
//     msgvec[1]=0x4f;
//     msgvec[2]=4;
//     // msgvec[3]=pvt_data->pvt_sol.ns;
//     Double2Hex(&msgvec[6],&pvt_data->pvt_sol.rr[0]);
//     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
//     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
//     float velX = (float)pvt_data->pvt_sol.rr[3];
//     float velY = (float)pvt_data->pvt_sol.rr[4];
//     float velZ = (float)pvt_data->pvt_sol.rr[5];
//     Float2Hex(&msgvec[30], &velX);
//     Float2Hex(&msgvec[34], &velY);
//     Float2Hex(&msgvec[38], &velZ);
//     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);
//     int index = 46; int cont=0;
//     float dummyfloat = 456.7;
//     std::map<int,Gps_Ephemeris>gps_ephem = pvt_data->gps_ephemeris_map;
//     std::map<int,Gnss_Synchro>Syncmap = pvt_data->c_gnss_observables_map;
//     float satvX{0};
//     float satvY{0};
//     float satvZ{0};
//     for (const auto& y : Syncmap){
//             for (const auto& x : gps_ephem)
//                 {
//                     if (y.second.PRN == x.second.PRN)
//                         {
//                             if (y.second.System == 'G')
//                                 {
//                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
//                                     gps_ephem.at(x.first).satellitePosition(tempoo);
//                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
//                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
//                                     satvX = (float)x.second.satvel_X;
//                                     satvY = (float)x.second.satvel_Y;
//                                     satvZ = (float)x.second.satvel_Z;
//                                     dummyfloat = (float)y.second.CN0_dB_hz;

//                                     // #######  Check Sat. Elevation  #######
//                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
//                                     double clock_bias_s;
//                                     double sat_pos_variance_m2;
//                                     std::array<double, 3> r_sat{};
//                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
//                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
//                                     double Az;
//                                     double El;
//                                     double dist_m;
//                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
//                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
//                                     const arma::vec dx = r_sat_eb_e - r_rx;
//                                     topocent(&Az, &El, &dist_m, r_rx, dx);
//                                     // dummyfloat = (float)El;
//                                     // #################################################
//                                     if (El >= pvt_data->d_conf.elevation_mask)
//                                         {
//                                             msgvec[index + 0] = (uint8_t)x.second.PRN;
//                                             Double2Hex(&msgvec[index + 1], &prange);
//                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
//                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
//                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
//                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
//                                             Float2Hex(&msgvec[index + 37], &satvX);
//                                             Float2Hex(&msgvec[index + 41], &satvY);
//                                             Float2Hex(&msgvec[index + 45], &satvZ);
//                                             Float2Hex(&msgvec[index + 49], &dummyfloat);


//                                             cont += 1;
//                                             index += 53;
//                                         }
//                                 }
//                         }
//                 }
//         }
//     index += 1;
//     uint8_t checks{0};
//     msgvec[3]=(uint8_t)cont;
//     Int2Hex(&msgvec[4],&index);
//     for (int i = 0; i < index-1; i++)
//         {
//             checks ^= msgvec[i];
//         }
//     msgvec[index-1] = checks;
//     return index;
// }

void gnss_serial_monitor::msg_handler_telemetry(const pmt::pmt_t& msg)
{
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            // ************************* GPS telemetry *************************
            if (msg_type_hash_code == d_gps_ephemeris_sptr_type_hash_code)
                {
                    // ### GPS EPHEMERIS ###
                    const auto gps_eph = wht::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));

                    gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    // if (d_enable_rx_clock_correction == true)
                    //     {
                    //         d_user_pvt_solver->gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    //     }
                    if (gps_eph->SV_health != 0)
                        {
                            // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("GPS"), gps_eph->PRN)
                            //   << " reports an unhealthy status,";
                            // if (d_use_unhealthy_sats)
                            //     {
                            //         // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
                            //     }
                            // else
                            //     {
                            //         // std::cout << " not used for navigation" << TEXT_RESET << '\n';
                            //     }
                        }
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            // // // LOG(WARNING) << "msg_handler_telemetry Bad any_cast: " << e.what();
        }
}

 void gnss_serial_monitor::msg_handler_telemetry_2(const pmt::pmt_t& msg)
{
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            // ************************* GPS telemetry *************************
            if (msg_type_hash_code == d_gps_ephemeris_sptr_type_hash_code)
                {
                    // ### GPS EPHEMERIS ###
                    const auto gps_eph = wht::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));
                 
                    gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    // if (d_enable_rx_clock_correction == true)
                    //     {
                    //         d_user_pvt_solver->gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    //     }
                    if (gps_eph->SV_health != 0)
                        {
                            // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("GPS"), gps_eph->PRN)
                                    //   << " reports an unhealthy status,";
                            // if (d_use_unhealthy_sats)
                            //     {
                            //         // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
                            //     }
                            // else
                            //     {
                            //         // std::cout << " not used for navigation" << TEXT_RESET << '\n';
                            //     }
                        }
                }
                        }
    catch (const wht::bad_any_cast& e)
        {
            // // // LOG(WARNING) << "msg_handler_telemetry Bad any_cast: " << e.what();
        }
}

void gnss_serial_monitor::msg_handler_pvtsol(const pmt::pmt_t& msg)
{
    try
        {
            // const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            // // ************************* GPS telemetry *************************
            // if (msg_type_hash_code == d_gps_ephemeris_sptr_type_hash_code)
            //     {
                    // ### GPS EPHEMERIS ###
                    const auto gps_eph = wht::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));
                    gpsephem[gps_eph->PRN] = *gps_eph;
                    // gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    // if (d_enable_rx_clock_correction == true)
                    //     {
                    //         d_user_pvt_solver->gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    //     }
                    if (gps_eph->SV_health != 0)
                        {
                            // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("GPS"), gps_eph->PRN)
                            //   << " reports an unhealthy status,";
                            // if (d_use_unhealthy_sats)
                            //     {
                            //         // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
                            //     }
                            // else
                            //     {
                            //         // std::cout << " not used for navigation" << TEXT_RESET << '\n';
                            //     }
                        }
                // }
        }
    catch (const wht::bad_any_cast& e)
        {
            // // // LOG(WARNING) << "msg_handler_telemetry Bad any_cast: " << e.what();
        }
}
