/*!
 * \file gnss_synchro_monitor.h
 * \brief Interface of a receiver monitoring block which allows sending
 * a data stream with the receiver internal parameters (Gnss_Synchro objects)
 * to local or remote clients over UDP.
 *
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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

#ifndef GNSS_SDR_GNSS_SERIAL_MONITOR_H
#define GNSS_SDR_GNSS_SERIAL_MONITOR_H

#include "gnss_block_interface.h"
#include "gnss_synchro_udp_sink.h"
#include <gnuradio/block.h>
#include <gnuradio/runtime_types.h>  // for gr_vector_void_star
#include <memory>
#include <string>
#include <vector>

#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <pmt/pmt.h>              // for pmt_t


#include "HEtechSerial.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup Gnss_Serial_Monitor core_monitor
 * Classes for the Gnss_Serial monitor.
 * \{ */

class Gps_Ephemeris;
class gnss_serial_monitor;

using gnss_serial_monitor_sptr = gnss_shared_ptr<gnss_serial_monitor>;

gnss_serial_monitor_sptr gnss_serial_make_monitor(int n_channels,
    std::string dev_serial,
    int baudrate
    );

/*!
 * \brief This class implements a monitoring block which allows sending
 * a data stream with the receiver internal parameters (Gnss_Synchro objects)
 * to local or remote clients over UDP.
 */
class gnss_serial_monitor : public gr::block
{
public:
    ~gnss_serial_monitor() = default;  //!< Default destructor
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

    size_t d_gps_ephemeris_sptr_type_hash_code;
    void msg_handler_telemetry(const pmt::pmt_t& msg);
    void msg_handler_telemetry_2(const pmt::pmt_t& msg);
    void msg_handler_pvtsol(const pmt::pmt_t& msg);

private:
    friend gnss_serial_monitor_sptr gnss_serial_make_monitor(int n_channels,
        std::string dev_serial,
        int baudrate
        );

    gnss_serial_monitor(int n_channels,
        std::string dev_serial,
        int baudrate
        );

    // std::unique_ptr<Gnss_Synchro_Udp_Sink> udp_sink_ptr;
    int count;
    int d_nchannels;
    // int d_decimation_factor;
    std::string d_dev_serial;
    int d_baudrate;
    serial_s_t comms;
    int contador{0};
    // Caio
    uint8_t msgvec[12 * 53 + 46];  // Output Buffer
    std::map<int, Gnss_Synchro> sync;
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;
    std::map<int, Gps_Ephemeris> gpsephem;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SYNCHRO_MONITOR_H
