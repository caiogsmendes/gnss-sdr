/*!
 * \file nmea_printer.cc
 * \brief Implementation of a NMEA 2.1 printer for GNSS-SDR
 * This class provides a implementation of a subset of the NMEA-0183 standard for interfacing
 * marine electronic devices as defined by the National Marine Electronics Association (NMEA).
 * See https://www.nmea.org/ for the NMEA 183 standard
 *
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
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

 #include "nmea_printer.h"
 #include "HEtechSerial.h"
 #include "geofunctions.h"
 #include "gnss_sdr_filesystem.h"
 #include "gps_ephemeris.h"
 #include "rtklib_ephemeris.h"
 #include "rtklib_solution.h"
 #include "rtklib_solver.h"
 #include <array>
 #include <cstdint>
 #include <exception>
 #include <fcntl.h>
 #include <iostream>  // for cout, cerr
 #include <map>
 #include <termios.h>
 #include <unistd.h>
 #include <utility>
 // //
 // #include "Comunicacao_RGL.h"
 // //
 
 #if USE_GLOG_AND_GFLAGS
 #include <glog/logging.h>
 #else
 #include <absl/log/log.h>
 #endif
 
 // Caio
 #include <fstream>
 
 
 Nmea_Printer::Nmea_Printer(const std::string& filename,
     bool flag_nmea_output_file,
     bool flag_nmea_tty_port,
     std::string nmea_dump_devname,
     const std::string& base_path) : nmea_base_path(base_path),
                                     d_flag_nmea_output_file(flag_nmea_output_file)
 {
     // comms = HEserial_connect(nmea_dump_devname.c_str(), B921600, O_RDWR | O_NDELAY | O_NOCTTY | O_NONBLOCK);
     if (d_flag_nmea_output_file == true)
         {
             fs::path full_path(fs::current_path());
             const fs::path p(nmea_base_path);
             if (!fs::exists(p))
                 {
                     std::string new_folder;
                     for (const auto& folder : fs::path(nmea_base_path))
                         {
                             new_folder += folder.string();
                             errorlib::error_code ec;
                             if (!fs::exists(new_folder))
                                 {
                                     if (!fs::create_directory(new_folder, ec))
                                         {
                                             // std::cout << "Could not create the " << new_folder << " folder." << std::endl;
                                             nmea_base_path = full_path.string();
                                         }
                                 }
                             new_folder += fs::path::preferred_separator;
                         }
                 }
             else
                 {
                     nmea_base_path = p.string();
                 }
 
             if ((nmea_base_path != ".") and (d_flag_nmea_output_file == true))
                 {
                     // std::cout << "NMEA files will be stored at " << nmea_base_path << std::endl;
                 }
 
             nmea_base_path = nmea_base_path + fs::path::preferred_separator;
 
             nmea_filename = nmea_base_path + filename;
 
             nmea_file_descriptor.open(nmea_filename.c_str(), std::ios::out);
             if (nmea_file_descriptor.is_open())
                 {
                     // D// LOG(INFO) << "NMEA printer writing on " << nmea_filename.c_str();
                 }
             else
                 {
                     // std::cout << "File " << nmea_filename << " cannot be saved. Wrong permissions?" << std::endl;
                 }
         }
 
     nmea_devname = std::move(nmea_dump_devname);
     if (flag_nmea_tty_port == true)
         {
             nmea_dev_descriptor = init_serial(nmea_devname);
             if (nmea_dev_descriptor != -1)
                 {
                     // D// LOG(INFO) << "NMEA printer writing on " << nmea_devname.c_str();
                 }
         }
     else
         {
             nmea_dev_descriptor = -1;
         }
 
     d_PVT_data = nullptr;
 }
 
 
 Nmea_Printer::~Nmea_Printer()
 {
     HEserial_disconnect(&comms);
     // D// LOG(INFO) << "NMEA printer destructor called.";
     const auto pos = nmea_file_descriptor.tellp();
     try
         {
             if (nmea_file_descriptor.is_open())
                 {
                     nmea_file_descriptor.close();
                 }
         }
     catch (const std::ofstream::failure& e)
         {
             std::cerr << "Problem closing NMEA dump file: " << nmea_filename << std::endl;
         }
     catch (const std::exception& e)
         {
             std::cerr << e.what() << std::endl;
         }
     if (pos == 0)
         {
             errorlib::error_code ec;
             if (!fs::remove(fs::path(nmea_filename), ec))
                 {
                     std::cerr << "Problem removing NMEA temporary file: " << nmea_filename << std::endl;
                 }
         }
     try
         {
             close_serial();
         }
     catch (const std::exception& e)
         {
             std::cerr << e.what() << std::endl;
         }
 }
 
 
 int Nmea_Printer::init_serial(const std::string& serial_device)
 {
     /*!
      * Opens the serial device and sets the default baud rate for a NMEA transmission (9600,8,N,1)
      */
     int fd = 0;
     // clang-format off
     struct termios options{};
     // clang-format on
     const int64_t BAUD = B921600;  // B9600;  // BAUD  =  B38400;
     const int64_t DATABITS = CS8;
     const int64_t STOPBITS = 1;
     const int64_t PARITYON = 0;
     const int64_t PARITY = 0;
 
     fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY /*| O_CLOEXEC*/);
     if (fd == -1)
         {
             return fd;  // failed to open TTY port
         }
 
     if (fcntl(fd, F_SETFL, 0) == -1)
         {
             // LOG(INFO) << "Error enabling direct I/O";  // clear all flags on descriptor, enable direct I/O
         }
     tcgetattr(fd, &options);  // read serial port options
 
     options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
     // enable receiver, set 8 bit data, ignore control lines
     // options.c_cflag |= (CLOCAL | CREAD | CS8);
     options.c_iflag = IGNPAR;
 
     // set the new port options
     tcsetattr(fd, TCSANOW, &options);
     return fd;
 }
 
 
 void Nmea_Printer::close_serial() const
 {
     if (nmea_dev_descriptor != -1)
         {
             close(nmea_dev_descriptor);
         }
 }


 bool Nmea_Printer::Print_Nmea_Line(const Rtklib_Solver* const pvt_data, serial_s_t commsS2, const bool d_thermal_enabled_, int HIL_Mode)
 {
     // set the new PVT data
     d_PVT_data = pvt_data;
     int bytes;

     // Caio
     switch (HIL_Mode)
         {
         case 8:
             bytes = get_msgvec_w_GAL_8(pvt_data, d_thermal_enabled_);
             break;
         case 16:
             bytes = get_msgvec_w_GAL_16(pvt_data, d_thermal_enabled_);
             break;
         case 32:
             bytes = get_msgvec_w_GAL_32(pvt_data, d_thermal_enabled_);
             break;
         case 64:
             bytes = get_msgvec_w_GAL_64(pvt_data, d_thermal_enabled_);
             break;
         case 128:
             bytes = get_msgvec_w_GAL_128(pvt_data, d_thermal_enabled_);
             break;
         default:
             bytes = get_msgvec_w_GAL(pvt_data, d_thermal_enabled_);
             break;
         }
     //

     // generate the NMEA sentences
     // GPRMC
     // const std::string GPRMC = get_GPRMC();
     // GPGGA (Global Positioning System Fixed Data)
     // const std::string GPGGA = get_GPGGA();
     // GPGSA
     // const std::string GPGSA = get_GPGSA();
     // GPGSV
     // const std::string GPGSV = get_GPGSV();

     // // write to log file
     // if (d_flag_nmea_output_file)
     //     {
     //         try
     //             {
     //                 nmea_file_descriptor
     //                     << msgvec_test
     //                     //<< GPRMC
     //                     // << GPGGA  // GPGGA (Global Positioning System Fixed Data)
     //                     // << GPGSA
     //                     // << GPGSV
     //                     << std::flush;
     //             }
     //         catch (const std::exception& ex)
     //             {
     //                 // D// LOG(INFO) << "NMEA printer can not write on output file" << nmea_filename.c_str();
     //             }
     //     }
 
     // write to serial device
     if (nmea_dev_descriptor != -1)
         {
             // int resultt = write(nmea_dev_descriptor, &msgvec_test[0], 365);
             // int resultt = write(comms.fd, &msgvec_test[0], 357);
             // int resultt = write(nmea_dev_descriptor, &msgvec[0], bytes);
             // tcdrain(commsS2->fd);
             // tcflush(commsS2.fd, TCIOFLUSH);
             int resultt = write(commsS2.fd, &msgvec[0], bytes);
             // tcdrain(commsS2.fd);
             if (resultt == -1)
                 {
                     // // D// LOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
                     return false;
                 }
 
             // if (write(nmea_dev_descriptor, GPRMC.c_str(), GPRMC.length()) == -1)
             //     {
             //         // D// LOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
             //         return false;
             //     }
             // if (write(nmea_dev_descriptor, GPGGA.c_str(), GPGGA.length()) == -1)
             //     {
             //         // D// LOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
             //         return false;
             //     }
             // if (write(nmea_dev_descriptor, GPGSA.c_str(), GPGSA.length()) == -1)
             //     {
             //         // D// LOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
             //         return false;
             //     }
             // if (write(nmea_dev_descriptor, GPGSV.c_str(), GPGSV.length()) == -1)
             //     {
             //         // D// LOG(INFO) << "NMEA printer cannot write on serial device" << nmea_devname.c_str();
             //         return false;
             //     }
         }
     return true;
 }


 char Nmea_Printer::checkSum(const std::string& sentence) const
 {
     char check = 0;
     // iterate over the string, XOR each byte with the total sum:
     for (char c : sentence)
         {
             check = static_cast<char>(check ^ c);
         }
     // return the result
     return check;
 }
 
 
 std::string Nmea_Printer::latitude_to_hm(double lat) const
 {
     bool north;
     if (lat < 0.0)
         {
             north = false;
             lat = -lat;
         }
     else
         {
             north = true;
         }
 
     const int deg = static_cast<int>(lat);
     double mins = lat - static_cast<double>(deg);
     mins *= 60.0;
     std::ostringstream out_string;
     out_string.setf(std::ios::fixed, std::ios::floatfield);
     out_string.fill('0');
     out_string.width(2);
     out_string << deg;
     out_string.width(2);
     out_string << static_cast<int>(mins) << ".";
     out_string.width(4);
     out_string << static_cast<int>((mins - static_cast<double>(static_cast<int>(mins))) * 1e4);
 
     if (north == true)
         {
             out_string << ",N";
         }
     else
         {
             out_string << ",S";
         }
     return out_string.str();
 }
 
 
 std::string Nmea_Printer::longitude_to_hm(double longitude) const
 {
     bool east;
     if (longitude < 0.0)
         {
             east = false;
             longitude = -longitude;
         }
     else
         {
             east = true;
         }
     const int deg = static_cast<int>(longitude);
     double mins = longitude - static_cast<double>(deg);
     mins *= 60.0;
     std::ostringstream out_string;
     out_string.setf(std::ios::fixed, std::ios::floatfield);
     out_string.width(3);
     out_string.fill('0');
     out_string << deg;
     out_string.width(2);
     out_string << static_cast<int>(mins) << ".";
     out_string.width(4);
     out_string << static_cast<int>((mins - static_cast<double>(static_cast<int>(mins))) * 1e4);
 
     if (east == true)
         {
             out_string << ",E";
         }
     else
         {
             out_string << ",W";
         }
     return out_string.str();
 }
 
 
 std::string Nmea_Printer::get_UTC_NMEA_time(const boost::posix_time::ptime d_position_UTC_time) const
 {
     // UTC Time: hhmmss.sss
     std::stringstream sentence_str;
 
     const boost::posix_time::time_duration td = d_position_UTC_time.time_of_day();
     const int utc_hours = td.hours();
     const int utc_mins = td.minutes();
     const int utc_seconds = td.seconds();
     const auto utc_milliseconds = static_cast<int>(td.total_milliseconds() - td.total_seconds() * 1000);
 
     if (utc_hours < 10)
         {
             sentence_str << "0";  //  two digits for hours
         }
     sentence_str << utc_hours;
 
     if (utc_mins < 10)
         {
             sentence_str << "0";  //  two digits for minutes
         }
     sentence_str << utc_mins;
 
     if (utc_seconds < 10)
         {
             sentence_str << "0";  //  two digits for seconds
         }
     sentence_str << utc_seconds;
 
     if (utc_milliseconds < 10)
         {
             sentence_str << ".00";  //  three digits for ms
             sentence_str << utc_milliseconds;
         }
     else if (utc_milliseconds < 100)
         {
             sentence_str << ".0";  //   three digits for ms
             sentence_str << utc_milliseconds;
         }
     else
         {
             sentence_str << ".";  //   three digits for ms
             sentence_str << utc_milliseconds;
         }
     return sentence_str.str();
 }
 
 
 std::string Nmea_Printer::get_GPRMC() const
 {
     // Sample -> $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598,*10
     std::stringstream sentence_str;
     std::array<unsigned char, 1024> buff{};
     outnmea_rmc(buff.data(), &d_PVT_data->pvt_sol);
     sentence_str << buff.data();
     return sentence_str.str();
 }
 
 
 std::string Nmea_Printer::get_GPGSA() const
 {
     // $GPGSA,A,3,07,02,26,27,09,04,15, , , , , ,1.8,1.0,1.5*33
     // GSA-GNSS DOP and Active Satellites
     std::stringstream sentence_str;
     std::array<unsigned char, 1024> buff{};
     outnmea_gsa(buff.data(), &d_PVT_data->pvt_sol, d_PVT_data->pvt_ssat.data());
     sentence_str << buff.data();
     return sentence_str.str();
 }
 
 
 std::string Nmea_Printer::get_GPGSV() const
 {
     // GSV-GNSS Satellites in View
     // $GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42,1*71
     // Notice that NMEA 2.1 only supports 12 channels
     std::stringstream sentence_str;
     std::array<unsigned char, 1024> buff{};
     outnmea_gsv(buff.data(), &d_PVT_data->pvt_sol, d_PVT_data->pvt_ssat.data());
     sentence_str << buff.data();
     return sentence_str.str();
 }
 
 
 std::string Nmea_Printer::get_GPGGA() const
 {
     std::stringstream sentence_str;
     std::array<unsigned char, 1024> buff{};
     outnmea_gga(buff.data(), &d_PVT_data->pvt_sol);
     sentence_str << buff.data();
     return sentence_str.str();
     // $GPGGA,104427.591,5920.7009,N,01803.2938,E,1,05,3.3,78.2,M,23.2,M,0.0,0000*4A
 }

 int Nmea_Printer::get_msgvec_w_GAL(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);

     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }
     Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
     int index = 58;
     //  int index = 46;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};
     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {
                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;
                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                             msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 return index;
 }

 // void conv_EST2FUN(std::map<int, Gps_Ephemeris>* map, Estruturas_UBLOX_t* est)
 // {
 
 // }
 
 /**
  * @brief Esta funcao verifica o inicio e o fim do cruzamento de uma semana para devolver o valor dentro de uma semana.
  * @param HW valor em segundos do tempo de meia semana [s]
  * @param tempo Tempo GPS em segundos[s]
  */
 double Nmea_Printer::CorretorTempo(double HW, double tempo)
 {
     volatile double tempo_corrigido;
     tempo_corrigido = tempo;
     if (tempo < (-1.0 * HW))
         {
             tempo_corrigido = tempo + 2.0 * HW;
         }
 
     if (tempo > HW)
         {
             tempo_corrigido = tempo - 2.0 * HW;
         }
 
     return tempo_corrigido;
 }
 
 /**
  * @brief Funcao para determinar o resto da divisao em double
  * @param a Numerador
  * @param b Denominador
  */
 double Nmea_Printer::rem(double a, double b)
 {
     volatile double saida;
     saida = a - b * floor(a / b);
     return saida;
 }
 
 /**
  * @brief Essa funcao faz o ABS de um double.
  * @param n numero cujo ABS deve ser executado
  */
 double Nmea_Printer::abs_d(double n)
 {
     volatile double saida;
     if (n >= 0.0)
         {
             saida = n;
         }
     else
         {
             saida = -1.0 * n;
         }
     return saida;
 }
 
 
 /**
  * @brief Funcao para calcular a posicao ECEF [m] e a velocidade ECEF [m/s] de um satelite a partir dos dados de Ephemerides e do dempo de Transmissao
  * @param Tempo_de_transmissao Tempo em que o sinal foi transmitido a partir do satelite [s]
  * @param prn do satelite que se deseja calcular a posicao e a velocidade.
  * @param Estruturas_GPS estrutura contendo todos os parâmetros que serao trocados durante a execucao da funcao UBLOX_MSG_FK
  * @param posi Vetor com a posicao estimada ECEF [m]
  * @param vel Vetor com a velocidade estimada ECEF [m/s]
  */
 int Nmea_Printer::CALC_POSI_VEL_SAT(double Tempo_de_transmissao, uint32_t prn, estrutura_gps gps, double* posi, double* vel)
 {
     volatile double gpsPi = 3.141592653589793;  // (pi)
     volatile double c = 299792458.0;            // Velocidade da Luz
     volatile double WGS84oe = 7.2921151467e-5;
     volatile double GravConstant = 398600500000000;   // Constante de Gravitacao
     volatile double F = -4.442807633393060e-10;       // -2*sqrt(GravConstant)/c^2;
     volatile double SECONDS_IN_HALF_WEEK = 302400.0;  // Metade de uma semana em segundos
     volatile double conv = 0.017453292519943;         // (pi/180)
 
     volatile double aux_double, aux_double1, dA, dN0, Mdot, dt, satClkCorr, timer, tk, M, E;
     volatile double sE, cE, dEdM, dTemp, relcorr, dDeltaFreq, Edot, sqrt1mee, P, Pdot, Pdot2;
     volatile double s2P, c2P, Rdot, R, I, Idot, U, Udot, sU, cU, Xp, Yp, Xpdot, Ypdot, L, Ldot;
     volatile double sL, cL, sI, cI, dX, dY, dZ, dtemp2, dtemp3, dtemp;
 
     double lla_rec[3];
     volatile int vis;
 
     // volatile double dCrs      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_rs; 		// sine correction to radius
     // volatile double dCuc      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_uc; 		// cosine correction to lattitude
     // volatile double dCus      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_us; 		// sine correction to lattitude
     // volatile double dCic      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_ic; 		// cosine correction to inclination
     // volatile double dCrc      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_rc; 		// cosine correction to radius
     // volatile double dCis      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].C_is; 		// sine correction to inclination
     // volatile double dToe      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].t_oe;
     // volatile double dn        =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].deltan; 	// mean motion difference from computed value
     // volatile double M0        =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].M_0; 		// mean anomaly at reference time
     // volatile double ecc       =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].e; 			// eccentricity
     // volatile double sqrta     =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].sqrtA;		// square root of semimajor axis
     // volatile double dOmega    =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].omega; 		// argument of perigee
     // volatile double dOmega0   =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].omega_0; 	// right ascencion at reference time
     // volatile double dOmegaDot =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].omegaDot;	// rate of right ascencion
     // volatile double dI0       =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].i_0; 		// orbital inclination
     // volatile double dIdot     =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].iDot;       // rate of inclination angle
     // volatile double a_f2      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].a_f2;
     // volatile double a_f1      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].a_f1;
     // volatile double a_f0      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].a_f0;
     // volatile double t_oc      =  Estrutura_GPS->Eph.GNSSID[0].svId[prn].t_oc;
 
    //  double dCrs = 0.0;  // sine correction to radius
    //  double dCuc = 0.0;  // cosine correction to lattitude
    //  double dCus = 0.0;  // sine correction to lattitude
    //  double dCic = 0.0;  // cosine correction to inclination
    //  double dCrc = 0.0;  // cosine correction to radius
    //  double dCis = 0.0;  // sine correction to inclination
    //  double dToe = 0.0;
    //  double dn = 0.0;         // mean motion difference from computed value
    //  double M0 = 0.0;         // mean anomaly at reference time
    //  double ecc = 0.0;        // eccentricity
    //  double sqrta = 0.0;      // square root of semimajor axis
    //  double dOmega = 0.0;     // argument of perigee
    //  double dOmega0 = 0.0;    // right ascencion at reference time
    //  double dOmegaDot = 0.0;  // rate of right ascencion
    //  double dI0 = 0.0;        // orbital inclination
    //  double dIdot = 0.0;      // rate of inclination angle
    //  double a_f2 = 0.0;
    //  double a_f1 = 0.0;
    //  double a_f0 = 0.0;
    //  double t_oc = 0.0;
 
    //  uint32_t PRNN = 0;
    //  int32_t IODC = 0;
    //  int32_t IODE_sf2 = 0;
    //  int32_t IODE_sf3 = 0;
 
    //  mtx.lock();

        //  mtx.unlock();
     // int prn2 = gpsephem.at(0).PRN;
 
     // double dCrs      =  C_rs; 		// sine correction to radius
     // double dCuc      =  C_uc; 		// cosine correction to lattitude
     // double dCus      =  C_us; 		// sine correction to lattitude
     // double dCic      =  C_ic; 		// cosine correction to inclination
     // double dCrc      =  C_rc; 		// cosine correction to radius
     // double dCis      =  C_is; 		// sine correction to inclination
     // double dToe      =  t_oe;
     // double dn        =  deltan; 	// mean motion difference from computed value
     // double M0        =  M_0; 		// mean anomaly at reference time
     // double ecc       =  e; 			// eccentricity
     // double sqrta     =  sqrtA;		// square root of semimajor axis
     // double dOmega    =  omega; 		// argument of perigee
     // double dOmega0   =  omega_0; 	// right ascencion at reference time
     // double dOmegaDot =  omegaDot;	// rate of right ascencion
     // double dI0       =  i_0; 		// orbital inclination
     // double dIdot     =  iDot;       // rate of inclination angle
     // double a_f2      =  a_f2;
     // double a_f1      =  a_f1;
     // double a_f0      =  a_f0;
     // double t_oc      =  t_oc;
 
     // #########################
     // lla_rec[0]=Estrutura_GPS->PVT.lat;
     // lla_rec[1]=Estrutura_GPS->PVT.lon;
     // lla_rec[2]=Estrutura_GPS->PVT.height;
     // // As ephemerides nao forem do mesmo conjunto nao devemos continuar a conta.
     // if ( (((int)Estrutura_GPS->Eph.GNSSID[0].svId[prn].IODC != (int)Estrutura_GPS->Eph.GNSSID[0].svId[prn].IODE_sf2) ||
     //     ((int)Estrutura_GPS->Eph.GNSSID[0].svId[prn].IODE_sf2 != (int)Estrutura_GPS->Eph.GNSSID[0].svId[prn].IODE_sf3)) ||
     //     ((int)Estrutura_GPS->Eph.GNSSID[0].svId[prn].IODC == 0))
     // {
     //     posi[0] = 0.0;
     //     posi[1] = 0.0;
     //     posi[2] = 0.0;
     //     vel[0]  = 0.0;
     //     vel[1]  = 0.0;
     //     vel[2]  = 0.0;
     //     vis     =   0;
     //     return vis;
     // }
     // #########################
 

    //  if (((gps.IODC != gps.IODE_sf2) || (gps.IODE_sf2 != gps.IODE_sf3)) || (gps.IODC == 0))
    //      {
    //          *posi = 0.0; posi++;
    //          *posi = 0.0; posi++;
    //          *posi = 0.0; posi++;
    //          *vel = 0.0; vel++;
    //          *vel = 0.0; vel++;
    //          *vel = 0.0; vel++;
    //          vis = 0;
    //          return 0;
    //      }
 
 
     // Se existem as condicoes necessarias para calcular a posicao e a velocidade aplicar o algoritmo do ICD-GPS-200C/GROV
     dA = gps.sqrta * gps.sqrta;
     dN0 = sqrt(GravConstant / (dA * dA * dA));
     Mdot = dN0;
     Mdot = Mdot + gps.dn;
     // Encontrar a diferenca de tempo
     aux_double = (Tempo_de_transmissao - gps.t_oc);
     dt = CorretorTempo(SECONDS_IN_HALF_WEEK, aux_double);
     // Calculando a correcao do Relógior GPS
     satClkCorr = (gps.a_f2 * dt + gps.a_f1) * dt + gps.a_f0;
     // Correct time difference
     timer = Tempo_de_transmissao - satClkCorr;
     aux_double = (timer - gps.dToe);
     tk = CorretorTempo(SECONDS_IN_HALF_WEEK, aux_double);  // OLHAR ISSO COM MUITO CUIDADO
     // Anomalia Media
     M = gps.M0 + Mdot * tk;
     aux_double = (M + 2.0 * gpsPi);
     aux_double1 = 2.0 * gpsPi;
     M = rem(aux_double, aux_double1);  // OLHAR ISSO COM MUITO CUIDADO.
     // Chute inicial para a anomalia ecentrica Initial
     E = M;
     for (int i = 0; i < 20; i++)
         {
             sE = sin(E);
             cE = cos(E);
             dEdM = 1.0 / (1.0 - gps.ecc * cE);
             dTemp = (M - E + gps.ecc * sE) * dEdM;
             aux_double = abs_d(dTemp);
             if (aux_double < 1.0e-14)
                 {
                     break;
                 }
             E = E + dTemp;
         }
     // Reduzindo par aum valor entre 0 e 360
     aux_double = (E + 2.0 * gpsPi);
     aux_double1 = 2.0 * gpsPi;
     E = rem(aux_double, aux_double1);
     // Calcaulando o termo de correcao relativistica
     relcorr = F * gps.ecc * gps.sqrta * sE;
     dDeltaFreq = gps.a_f1 + 2.0 * tk * gps.a_f2;
     Edot = dEdM * Mdot;
     // Calculate the true anomaly and angle phi
     aux_double = (1.0 - (gps.ecc * gps.ecc));
     sqrt1mee = sqrt(aux_double);
     P = atan2(sqrt1mee * sE, cE - gps.ecc) + gps.dOmega;  // OLHAR ISSO COM MUITO CUIDADO.
     // Reduzindo para um Valor entre 0 and 360 deg
     aux_double = 2.0 * gpsPi;
     P = rem(P, aux_double);  // OLHAR ISSO COM MUITO CUIDADO.
 
     Pdot = sqrt1mee * dEdM * Edot;
     Pdot2 = 2.0 * Pdot;
 
     dtemp = 2.0 * P;
     s2P = sin(dtemp);
     c2P = cos(dtemp);
 
     // Correcao de raio.
     R = dA * (1.0 - gps.ecc * cE);
     Rdot = dA * gps.ecc * sE * Edot;
     R = R + gps.dCrs * s2P + gps.dCrc * c2P;
     Rdot = Rdot + Pdot2 * (gps.dCrs * c2P - gps.dCrc * s2P);
     // Correcao de Inclinacao
     I = gps.dI0;
     I = I + gps.dIdot * tk + gps.dCis * s2P + gps.dCic * c2P;
     Idot = gps.dIdot + Pdot2 * (gps.dCis * c2P - gps.dCic * s2P);
     // Correcao de argumento de Latitude
     U = P + gps.dCus * s2P + gps.dCuc * c2P;
     Udot = Pdot + Pdot2 * (gps.dCus * c2P - gps.dCuc * s2P);
     sU = sin(U);
     cU = cos(U);
     Xp = R * cU;
     Yp = R * sU;
     Xpdot = Rdot * cU - Yp * Udot;
     Ypdot = Rdot * sU + Xp * Udot;
     // Calculando o ângulo entre o nó de ascencao e o Meridiano de Greenwich [Satelite nao estacionario]
     L = gps.dOmega0 + tk * (gps.dOmegaDot - WGS84oe);
     L = L - WGS84oe * gps.dToe;
     Ldot = gps.dOmegaDot - WGS84oe;
     // Reduzindo para um Valor entre 0 and 360 deg
     aux_double = L + 2.0 * gpsPi;
     aux_double1 = 2.0 * gpsPi;
     L = rem(aux_double, aux_double1);
     sL = sin(L);
     cL = cos(L);
     sI = sin(I);
     cI = cos(I);
     dtemp = Yp * cI;
     // Coordenadas Calculadas
     *posi = Xp * cL - dtemp * sL; posi++;
     *posi = Xp * sL + dtemp * cL; posi++;
     *posi = Yp * sI; posi++;
 
    //  satClkCorr = satClkCorr - relcorr;
 
     dX = Xp * cL - dtemp * sL;
     dY = Xp * sL + dtemp * cL;
     dZ = Yp * sI;
 
     dtemp2 = dZ * Idot;
     dtemp3 = Ypdot * cI;
 
     *vel = -Ldot * (dY) + Xpdot * cL - (dtemp3 + dtemp2) * sL; vel++;
     *vel = Ldot * (dX) + Xpdot * sL + (dtemp3 - dtemp2) * cL; vel++;
     *vel = dtemp * Idot + Ypdot * sI; vel++;
     *vel = satClkCorr; vel++;
     // vis = visivel(posi, lla_rec, 10.0);
 
     // return vis;
     return 10;
 }




 int Nmea_Printer::get_msgvec_w_GAL_8(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);

     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }
     Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
     int index = 58;
     //  int index = 46;
     int sat2 = 0;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};
     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {
                                    sat2++;
                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;

                                     int32_t prnn = x.second.PRN;
                                     if ((pvt_data->tow_symbol_ms == 169871) || (pvt_data->tow_symbol_ms == 169971) || (pvt_data->tow_symbol_ms == 170027) || (pvt_data->tow_symbol_ms == 170073))
                                         {
                                             if (sat2 > 1)
                                                 {
                                                     prnn = prnn + 33 + sat2++;
                                                     sat2 = 0;
                                                 }
                                         }

                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                            //  msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             msgvec[index + 0] = (uint8_t)prnn;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 return index;
 }

 int Nmea_Printer::get_msgvec_w_GAL_16(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
    int sat2=0;
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);

     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }
     Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
     int index = 58;
     //  int index = 46;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};
     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {
                                    sat2++;
                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;

                                     if ((pvt_data->tow_symbol_ms == 169871) || (pvt_data->tow_symbol_ms == 169971) || (pvt_data->tow_symbol_ms == 170027) || (pvt_data->tow_symbol_ms == 170073))
                                         {
                                             if (sat2 > 2)
                                                 {
                                                     prange = (x.second.PRN % 2) != 0 ? 16980000 : 35000000;
                                                     sat2 = 0;
                                                 }
                                         }

                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                             msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 
 return index;
 }

 int Nmea_Printer::get_msgvec_w_GAL_32(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     if((pvt_data->tow_symbol_ms == 169871)||(pvt_data->tow_symbol_ms == 169971)||(pvt_data->tow_symbol_ms == 170027)||(pvt_data->tow_symbol_ms == 170073)){
        uint32_t tow_errado;
        tow_errado = pvt_data->tow_symbol_ms - 10;
        Integer2Hex(&msgvec[42], &tow_errado);
     }else{
        Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);
     }
     

     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }
     Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
     int index = 58;
     //  int index = 46;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};

     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {

                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;


                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                             msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 return index;
 }

 int Nmea_Printer::get_msgvec_w_GAL_64(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);

     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }

         if((pvt_data->tow_symbol_ms == 169871)||(pvt_data->tow_symbol_ms == 169971)||(pvt_data->tow_symbol_ms == 170027)||(pvt_data->tow_symbol_ms == 170073)){
            double usr_clk_errado;
            usr_clk_errado = pvt_data->usr_clk_offset*10000.0;
            Double2Hex(&msgvec[50], &usr_clk_errado);
         }else{
            Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
         }
     
     int index = 58;
     //  int index = 46;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};
     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {
                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;
                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                             msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 return index;
 }

 int Nmea_Printer::get_msgvec_w_GAL_128(const Rtklib_Solver* const pvt_data, const bool d_thermal_enabled_)
 {
    int sat2=0;
    uint32_t ult_prn;
    uint32_t prnn;
    bool check = false;
     std::ifstream thermal;
     mtx.lock();
     msgvec[0] = 0xd4;
     msgvec[1] = 0x4f;
     msgvec[2] = 4;
     // msgvec[3]=pvt_data->pvt_sol.ns;
     Double2Hex(&msgvec[6], &pvt_data->pvt_sol.rr[0]);
     Double2Hex(&msgvec[14], &pvt_data->pvt_sol.rr[1]);
     Double2Hex(&msgvec[22], &pvt_data->pvt_sol.rr[2]);
     float velX = (float)pvt_data->pvt_sol.rr[3];
     float velY = (float)pvt_data->pvt_sol.rr[4];
     float velZ = (float)pvt_data->pvt_sol.rr[5];
     Float2Hex(&msgvec[30], &velX);
     Float2Hex(&msgvec[34], &velY);
     Float2Hex(&msgvec[38], &velZ);
     Integer2Hex(&msgvec[42], &pvt_data->tow_symbol_ms);
    
     // Caio-Derso - Experimental
     //  Float2Hexxx(&velX, &velY, &pvt_data->usr_clk_offset);
     //  Float2Hex(&msgvec[30], &velX);
     //  Float2Hex(&msgvec[34], &velY);
     //


     if (d_thermal_enabled_)
         {
             thermal.open("/sys/devices/virtual/thermal/thermal_zone0/temp");
             getline(thermal, temper);
             float i_temper = (float)stoi(temper) / 1000;
             Float2Hex(&msgvec[46], &i_temper);
             thermal.close();
             // index += 4;
         }

         if((pvt_data->tow_symbol_ms == 169871)||(pvt_data->tow_symbol_ms == 169971)||(pvt_data->tow_symbol_ms == 170027)||(pvt_data->tow_symbol_ms == 170073)){
            check = true;
         }else{
            check = false;
         }

     Double2Hex(&msgvec[50], &pvt_data->usr_clk_offset);
     int index = 58;
     //  int index = 46;
     int cont = 0;
     float dummyfloat = 456.7;
     std::map<int, Gps_Ephemeris> gps_ephem = pvt_data->gps_ephemeris_map;
     std::map<int, Gnss_Synchro> Syncmap = pvt_data->c_gnss_observables_map;
     float satvX{0};
     float satvY{0};
     float satvZ{0};
     //  double satP[3];
     //  double satV[3];
     //  double tempoRX = Syncmap.begin()->second.RX_time;
     for (const auto& y : Syncmap)
         {
             for (const auto& x : gps_ephem)
                 {
                     if (y.second.PRN == x.second.PRN)
                         {
                             if (y.second.System == 'G')
                                 {
                                     prnn = x.second.PRN;
                                     if (check == true)
                                         {
                                             sat2++;

                                             if (sat2 > 2)
                                                 {
                                                     prnn = ult_prn;
                                                     sat2 = 0;
                                                 }
                                         }
                                    //  double satP[3]{0};
                                    //  double satV[4]{0};                                                                 // satV[3] é correção relativistica do clk do Sat
                                     double tempoo = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite
                                                                                                                        // double tempoo = tempoRX - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;  // Tempo de transmissão do satélite

                                     gps_ephem.at(x.first).satellitePosition(tempoo);
                                     //
                                     //  mtx.lock();
                                    //  estrutura_gps gps;
                                    //  gps.dCrs = x.second.Crs;
                                    //  gps.dCuc = x.second.Cuc;
                                    //  gps.dCus = x.second.Cus;
                                    //  gps.dCic = x.second.Cic;
                                    //  gps.dCrc = x.second.Crc;
                                    //  gps.dCis = x.second.Cis;
                                    //  gps.dToe = x.second.toe;
                                    //  gps.dn = x.second.delta_n;
                                    //  gps.M0 = x.second.M_0;
                                    //  gps.ecc = x.second.ecc;
                                    //  gps.sqrta = x.second.sqrtA;
                                    //  gps.dOmega = x.second.omega;
                                    //  gps.dOmega0 = x.second.OMEGA_0;
                                    //  gps.dOmegaDot = x.second.OMEGAdot;
                                    //  gps.dI0 = x.second.i_0;
                                    //  gps.dIdot = x.second.idot;
                                    //  gps.a_f2 = x.second.af2;
                                    //  gps.a_f1 = x.second.af1;
                                    //  gps.a_f0 = x.second.af0;
                                    //  gps.t_oc = x.second.toc;

                                    //  gps.PRNN = x.second.PRN;
                                    //  gps.IODC = x.second.IODC;
                                    //  gps.IODE_sf2 = x.second.IODE_SF2;
                                    //  gps.IODE_sf3 = x.second.IODE_SF3;
                                     //  mtx.unlock();
                                     //  if(CALC_POSI_VEL_SAT(tempoo, x.second.PRN, gps, &satP[0], &satV[0])!=0){

                                     //
                                     float deltaprange_f = -SPEED_OF_LIGHT_M_S * (y.second.Carrier_Doppler_hz / 1575420000) - ((pvt_data->get_clock_drift_ppm() * 1e-6) - x.second.af1) * SPEED_OF_LIGHT_M_S;
                                     double prange = y.second.Pseudorange_m + (x.second.dtr) * SPEED_OF_LIGHT_M_S;
                                    //  double prange = y.second.Pseudorange_m + (satV[3]) * SPEED_OF_LIGHT_M_S;

                                     // double prange = y.second.Pseudorange_m;
                                     satvX = (float)x.second.satvel_X;
                                     satvY = (float)x.second.satvel_Y;
                                     satvZ = (float)x.second.satvel_Z;
                                     //  satvX = (float)satV[0];
                                     //  satvY = (float)satV[1];
                                     //  satvZ = (float)satV[2];
                                     dummyfloat = (float)y.second.CN0_dB_hz;
                                     // #######  Check Sat. Elevation  #######
                                     const eph_t rtklib_eph = eph_to_rtklib(x.second, 0);
                                     double clock_bias_s;
                                     double sat_pos_variance_m2;
                                     std::array<double, 3> r_sat{};
                                     // eph2pos(gps_gtime, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     eph2pos(pvt_data->rtklib_pvt_sol_time, &rtklib_eph, r_sat.data(), &clock_bias_s, &sat_pos_variance_m2);
                                     double Az;
                                     double El;
                                     double dist_m;
                                     const arma::vec r_rx = arma::vec{pvt_data->pvt_sol.rr[0], pvt_data->pvt_sol.rr[1], pvt_data->pvt_sol.rr[2]};
                                     const arma::vec r_sat_eb_e = arma::vec{r_sat[0], r_sat[1], r_sat[2]};
                                     const arma::vec dx = r_sat_eb_e - r_rx;
                                     topocent(&Az, &El, &dist_m, r_rx, dx);
                                     // dummyfloat = (float)El;
                                     // #################################################
                                     if (El >= pvt_data->d_conf.elevation_mask)
                                         {
                                            //  msgvec[index + 0] = (uint8_t)x.second.PRN;
                                             msgvec[index + 0] = (uint8_t)prnn;
                                             Double2Hex(&msgvec[index + 1], &prange);
                                             Float2Hex(&msgvec[index + 9], &deltaprange_f);
                                             Double2Hex(&msgvec[index + 13], &x.second.satpos_X);
                                             Double2Hex(&msgvec[index + 21], &x.second.satpos_Y);
                                             Double2Hex(&msgvec[index + 29], &x.second.satpos_Z);
                                             Float2Hex(&msgvec[index + 37], &satvX);
                                             Float2Hex(&msgvec[index + 41], &satvY);
                                             Float2Hex(&msgvec[index + 45], &satvZ);

                                             //  Double2Hex(&msgvec[index + 13], &satP[0]);
                                             //  Double2Hex(&msgvec[index + 21], &satP[1]);
                                             //  Double2Hex(&msgvec[index + 29], &satP[2]);
                                             //  Float2Hex(&msgvec[index + 37], &satvX);
                                             //  Float2Hex(&msgvec[index + 41], &satvY);
                                             //  Float2Hex(&msgvec[index + 45], &satvZ);
                                             Float2Hex(&msgvec[index + 49], &dummyfloat);
                                             cont += 1;
                                             index += 53;
                                         }
                                         ult_prn = x.second.PRN;
                                 }
                         }
                 }
         }
 
 index += 1;
 uint8_t checks{0};
 msgvec[3] = (uint8_t)cont;
 Int2Hex(&msgvec[4], &index);
 for (int i = 0; i < index - 1; i++)
     {
         checks ^= msgvec[i];
     }
 msgvec[index - 1] = checks;
 mtx.unlock();
 return index;
 }