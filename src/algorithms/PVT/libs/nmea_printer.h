/*!
 * \file nmea_printer.h
 * \brief Interface of a NMEA 2.1 printer for GNSS-SDR
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

#ifndef GNSS_SDR_NMEA_PRINTER_H
#define GNSS_SDR_NMEA_PRINTER_H

#include <boost/date_time/posix_time/ptime.hpp>  // for ptime
#include <fstream>                               // for ofstream
#include <memory>                                // for shared_ptr
#include <string>                                // for string


#include <vector>
#include <unistd.h>
#include "HEtechSerial.h"

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Rtklib_Solver;

/*!
 * \brief This class provides a implementation of a subset of the NMEA-0183 standard for interfacing
 * marine electronic devices as defined by the National Marine Electronics Association (NMEA).
 *
 * See https://en.wikipedia.org/wiki/NMEA_0183
 */
class Nmea_Printer
{
public:
    /*!
     * \brief Default constructor.
     */
    Nmea_Printer(const std::string& filename, bool flag_nmea_output_file, bool flag_nmea_tty_port, std::string nmea_dump_devname, const std::string& base_path = ".");

    /*!
     * \brief Default destructor.
     */
    ~Nmea_Printer();

    /*!
     * \brief Print NMEA PVT and satellite info to the initialized device
     */
    bool Print_Nmea_Line(const Rtklib_Solver* const pvt_data);

    // uint8_t msgvec_test[357] = {212, 79, 4, 6, 101, 1, 109, 212, 216, 235, 33, 29, 79, 65, 233, 13, 59, 57, 38, 21, 80, 193, 86, 207, 203,99, 72, 15, 67, 193, 161, 40, 12, 189, 68, 231, 43, 189, 106, 66, 80, 59, 116, 162, 7, 0, 23, 119, 95, 198, 232,104, 36, 115, 65, 190, 155, 197, 190, 155, 171, 114, 194, 69, 131, 110, 65, 89, 235, 220, 97, 165, 109, 113, 193, 71,135, 220, 234, 118, 236, 99, 193, 94, 125, 181, 195, 20, 203, 162, 68, 167, 252, 49, 197, 204, 189, 54, 66, 18, 137,14, 158, 79, 91, 166, 116, 65, 13, 167, 80, 67, 83, 208, 252, 197, 9, 31, 105, 65, 118, 127, 30, 25, 94, 228,90, 193, 115, 171, 109, 148, 114, 237, 116, 193, 83, 254, 188, 68, 56, 81, 18, 69, 166, 63, 5, 67, 47, 211, 58,66, 10, 112, 39, 247, 195, 233, 144, 116, 65, 204, 20, 0, 196, 151, 162, 163, 222, 156, 242, 98, 65, 205, 189, 88,229, 106, 45, 119, 193, 232, 27, 217, 234, 47, 198, 68, 65, 163, 237, 172, 67, 29, 126, 60, 195, 130, 196, 73, 197,221, 43, 54, 66, 25, 238, 90, 241, 127, 28, 210, 118, 65, 150, 240, 40, 68, 106, 54, 93, 7, 178, 226, 112, 65,35, 128, 44, 80, 0, 252, 100, 193, 97, 86, 246, 225, 225, 114, 110, 65, 45, 213, 39, 196, 224, 9, 251, 68, 75,223, 3, 69, 145, 38, 8, 66, 15, 90, 41, 195, 22, 237, 114, 118, 65, 30, 72, 38, 67, 4, 60, 170, 145, 167,147, 117, 65, 27, 100, 127, 161, 230, 104, 83, 65, 34, 219, 127, 138, 91, 247, 105, 193, 18, 39, 193, 196, 140, 187,25, 68, 182, 2, 23, 197, 252, 151, 57, 66, 29, 244, 255, 105, 45, 160, 232, 116, 65, 251, 141, 131, 67, 70, 158,67, 92, 213, 147, 120, 65, 124, 113, 86, 113, 199, 49, 76, 193, 93, 10, 105, 168, 168, 122, 85, 193, 126, 56, 48,68, 231, 225, 112, 67, 241};
    uint8_t msgvec_test[365]={212, 79, 4, 6, 109, 1, 109, 212, 216, 235, 33, 29, 79, 65, 233, 13, 59, 57, 38, 21, 80, 193, 86, 207, 203, 99, 72, 15, 67, 193, 161, 40, 12, 189, 68, 231, 43, 189, 106, 66, 80, 59, 116, 162, 7, 0, 23, 119, 95, 198, 232, 104, 36, 115, 65, 190, 155, 197, 190, 155, 171, 114, 194, 69, 131, 110, 65, 89, 235, 220, 97, 165, 109, 113, 193, 71, 135, 220, 234, 118, 236, 99, 193, 94, 125, 181, 195, 20, 203, 162, 68, 167, 252, 49, 197, 204, 189, 54, 66, 18, 137, 14, 158, 79, 91, 166, 116, 65, 13, 167, 80, 67, 83, 208, 252, 197, 9, 31, 105, 65, 118, 127, 30, 25, 94, 228, 90, 193, 115, 171, 109, 148, 114, 237, 116, 193, 83, 254, 188, 68, 56, 81, 18, 69, 166, 63, 5, 67, 47, 211, 58, 66, 10, 112, 39, 247, 195, 233, 144, 116, 65, 204, 20, 0, 196, 151, 162, 163, 222, 156, 242, 98, 65, 205, 189, 88, 229, 106, 45, 119, 193, 232, 27, 217, 234, 47, 198, 68, 65, 163, 237, 172, 67, 29, 126, 60, 195, 130, 196, 73, 197, 221, 43, 54, 66, 25, 238, 90, 241, 127, 28, 210, 118, 65, 150, 240, 40, 68, 106, 54, 93, 7, 178, 226, 112, 65, 35, 128, 44, 80, 0, 252, 100, 193, 97, 86, 246, 225, 225, 114, 110, 65, 45, 213, 39, 196, 224, 9, 251, 68, 75, 223, 3, 69, 145, 38, 8, 66, 15, 90, 41, 195, 22, 237, 114, 118, 65, 30, 72, 38, 67, 4, 60, 170, 145, 167, 147, 117, 65, 27, 100, 127, 161, 230, 104, 83, 65, 34, 219, 127, 138, 91, 247, 105, 193, 18, 39, 193, 196, 140, 187, 25, 68, 182, 2, 23, 197, 252, 151, 57, 66, 29, 244, 255, 105, 45, 160, 232, 116, 65, 251, 141, 131, 67, 70, 158, 67, 92, 213, 147, 120, 65, 124, 113, 86, 113, 199, 49, 76, 193, 93, 10, 105, 168, 168, 122, 85, 193, 126, 56, 48, 68, 231, 225, 112, 67, 239, 105, 64, 69, 238, 218, 65, 66, 77};
    
    uint8_t msgvec[12*53+46]; //Output Buffer
    serial_s_t comms;

private:
    int init_serial(const std::string& serial_device);  // serial port control
    void close_serial() const;
    std::string get_GPGGA() const;  // fix data
    std::string get_GPGSV() const;  // satellite data
    std::string get_GPGSA() const;  // overall satellite reception data
    std::string get_GPRMC() const;  // minimum recommended data
    std::string get_UTC_NMEA_time(const boost::posix_time::ptime d_position_UTC_time) const;
    std::string longitude_to_hm(double longitude) const;
    std::string latitude_to_hm(double lat) const;
    char checkSum(const std::string& sentence) const;

    const Rtklib_Solver* d_PVT_data;

    std::ofstream nmea_file_descriptor;  // Output file stream for NMEA log file

    std::string nmea_filename;  // String with the NMEA log filename
    std::string nmea_base_path;
    std::string nmea_devname;

    int nmea_dev_descriptor;  // NMEA serial device descriptor (i.e. COM port)
    bool d_flag_nmea_output_file;

    int get_msgvec_w_GAL(const Rtklib_Solver* const pvt_data);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_NMEA_PRINTER_H
