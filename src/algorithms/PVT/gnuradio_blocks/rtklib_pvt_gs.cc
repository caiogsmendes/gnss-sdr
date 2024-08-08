/*!
 * \file rtklib_pvt_gs.cc
 * \brief Interface of a Position Velocity and Time computation block
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#include "rtklib_pvt_gs.h"
#include "MATH_CONSTANTS.h"
// #include "an_packet_printer.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_iono.h"
#include "beidou_dnav_utc_model.h"
#include "display.h"
#include "galileo_almanac.h"
#include "galileo_almanac_helper.h"
#include "galileo_ephemeris.h"
#include "galileo_has_data.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "geohash.h"
// #include "geojson_printer.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_frequencies.h"
#include "gnss_satellite.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"
#include "gps_almanac.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "gps_cnav_utc_model.h"
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
// #include "gpx_printer.h"
// #include "has_simple_printer.h"
// #include "kml_printer.h"
#include "monitor_ephemeris_udp_sink.h"
#include "monitor_pvt.h"
#include "monitor_pvt_udp_sink.h"
// #include "nmea_printer.h"
#include "pvt_conf.h"
// #include "rinex_printer.h"
// #include "rtcm_printer.h"
#include "rtklib_rtkcmn.h"
#include "rtklib_solver.h"
#include "trackingcmd.h"
#include <boost/archive/xml_iarchive.hpp>  // for xml_iarchive
#include <boost/archive/xml_oarchive.hpp>  // for xml_oarchive
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>  // for nvp, make_nvp
#include <glog/logging.h>               // for LOG
#include <gnuradio/io_signature.h>      // for io_signature
#include <pmt/pmt_sugar.h>              // for mp
#include <algorithm>                    // for sort, unique
#include <cerrno>                       // for errno
#include <cstring>                      // for strerror
#include <exception>                    // for exception
#include <fstream>                      // for ofstream
#include <iomanip>                      // for put_time, setprecision
#include <iostream>                     // for operator<<
#include <locale>                       // for locale
#include <sstream>                      // for ostringstream
#include <stdexcept>                    // for length_error
#include <sys/ipc.h>                    // for IPC_CREAT
#include <sys/msg.h>                    // for msgctl
#include <typeinfo>                     // for std::type_info, typeid
#include <utility>                      // for pair

// Caio
#include "HEtechSerial.h"
#include "matio.h"
#include "pvt_interface.h"
#include "rtklib_ephemeris.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <pthread.h>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if USE_STD_COMMON_FACTOR
#include <numeric>
namespace bc = std;
#else
#if USE_OLD_BOOST_MATH_COMMON_FACTOR
#include <boost/math/common_factor_rt.hpp>
namespace bc = boost::math;
#else
#include <boost/integer/common_factor_rt.hpp>
namespace bc = boost::integer;
#endif
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

// Caio
// class thread : public std::thread
// {
//   public:
//     thread() {}
//     static void setScheduling(std::thread &th, int policy, int priority) {
//         sch_params.sched_priority = priority;
//         if(pthread_setschedparam(th.native_handle(), policy, &sch_params)) {
//             std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
//         }
//     }
//   private:
//     sched_param sch_params;
// };
// #


rtklib_pvt_gs_sptr rtklib_make_pvt_gs(uint32_t nchannels,
    const Pvt_Conf& conf_,
    const rtk_t& rtk)
{
    return rtklib_pvt_gs_sptr(new rtklib_pvt_gs(nchannels,
        conf_,
        rtk));
}


rtklib_pvt_gs::rtklib_pvt_gs(uint32_t nchannels,
    const Pvt_Conf& conf_,
    const rtk_t& rtk)
    : gr::sync_block("rtklib_pvt_gs",
          gr::io_signature::make(nchannels, nchannels, sizeof(Gnss_Synchro)),
          gr::io_signature::make(0, 0, 0)),
      d_dump_filename(conf_.dump_filename),
      d_geohash(std::make_unique<Geohash>()),
      d_gps_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Gps_Ephemeris>).hash_code()),
      d_gps_iono_sptr_type_hash_code(typeid(std::shared_ptr<Gps_Iono>).hash_code()),
      d_gps_utc_model_sptr_type_hash_code(typeid(std::shared_ptr<Gps_Utc_Model>).hash_code()),
      d_gps_cnav_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Gps_CNAV_Ephemeris>).hash_code()),
      d_gps_cnav_iono_sptr_type_hash_code(typeid(std::shared_ptr<Gps_CNAV_Iono>).hash_code()),
      d_gps_cnav_utc_model_sptr_type_hash_code(typeid(std::shared_ptr<Gps_CNAV_Utc_Model>).hash_code()),
      d_gps_almanac_sptr_type_hash_code(typeid(std::shared_ptr<Gps_Almanac>).hash_code()),
      d_galileo_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_Ephemeris>).hash_code()),
      d_galileo_iono_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_Iono>).hash_code()),
      d_galileo_utc_model_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_Utc_Model>).hash_code()),
      d_galileo_almanac_helper_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_Almanac_Helper>).hash_code()),
      d_galileo_almanac_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_Almanac>).hash_code()),
      d_glonass_gnav_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Glonass_Gnav_Ephemeris>).hash_code()),
      d_glonass_gnav_utc_model_sptr_type_hash_code(typeid(std::shared_ptr<Glonass_Gnav_Utc_Model>).hash_code()),
      d_glonass_gnav_almanac_sptr_type_hash_code(typeid(std::shared_ptr<Glonass_Gnav_Almanac>).hash_code()),
      d_beidou_dnav_ephemeris_sptr_type_hash_code(typeid(std::shared_ptr<Beidou_Dnav_Ephemeris>).hash_code()),
      d_beidou_dnav_iono_sptr_type_hash_code(typeid(std::shared_ptr<Beidou_Dnav_Iono>).hash_code()),
      d_beidou_dnav_utc_model_sptr_type_hash_code(typeid(std::shared_ptr<Beidou_Dnav_Utc_Model>).hash_code()),
      d_beidou_dnav_almanac_sptr_type_hash_code(typeid(std::shared_ptr<Beidou_Dnav_Almanac>).hash_code()),
      d_galileo_has_data_sptr_type_hash_code(typeid(std::shared_ptr<Galileo_HAS_data>).hash_code()),
    //   d_rinex_version(conf_.rinex_version),
      d_rx_time(0.0),
      d_local_counter_ms(0ULL),
      d_timestamp_rx_clock_offset_correction_msg_ms(0LL),
    //   d_rinexobs_rate_ms(conf_.rinexobs_rate_ms),
    //   d_kml_rate_ms(conf_.kml_rate_ms),
    //   d_gpx_rate_ms(conf_.gpx_rate_ms),
    //   d_geojson_rate_ms(conf_.geojson_rate_ms),
    //   d_nmea_rate_ms(conf_.nmea_rate_ms),
    //   d_an_rate_ms(conf_.an_rate_ms),
      d_output_rate_ms(conf_.output_rate_ms),
      d_display_rate_ms(conf_.display_rate_ms),
      d_report_rate_ms(1000),
      d_max_obs_block_rx_clock_offset_ms(conf_.max_obs_block_rx_clock_offset_ms),
      d_nchannels(nchannels),
      d_type_of_rx(conf_.type_of_receiver),
      d_observable_interval_ms(conf_.observable_interval_ms),
      d_pvt_errors_counter(0),
      d_dump(conf_.dump),
      d_dump_mat(conf_.dump_mat && conf_.dump),
    //   d_rinex_output_enabled(conf_.rinex_output_enabled),
    //   d_geojson_output_enabled(conf_.geojson_output_enabled),
    //   d_gpx_output_enabled(conf_.gpx_output_enabled),
    //   d_kml_output_enabled(conf_.kml_output_enabled),
    //   d_nmea_output_file_enabled(conf_.nmea_output_file_enabled || conf_.flag_nmea_tty_port),
    //   d_xml_storage(conf_.xml_output_enabled),
      d_flag_monitor_pvt_enabled(conf_.monitor_enabled),
      d_flag_monitor_ephemeris_enabled(conf_.monitor_ephemeris_enabled),
      d_show_local_time_zone(conf_.show_local_time_zone),
      d_enable_rx_clock_correction(conf_.enable_rx_clock_correction),
    //   d_an_printer_enabled(conf_.an_output_enabled),
      d_log_timetag(conf_.log_source_timetag),
      d_use_e6_for_pvt(conf_.use_e6_for_pvt),
      d_use_has_corrections(conf_.use_has_corrections),
      d_device_serial(conf_.dev_serial),
      d_use_unhealthy_sats(conf_.use_unhealthy_sats)
{
    // Send feedback message to observables block with the receiver clock offset
    this->message_port_register_out(pmt::mp("pvt_to_observables"));
    // Experimental: VLT commands from PVT to tracking channels
    this->message_port_register_out(pmt::mp("pvt_to_trk"));
    // Send PVT status to gnss_flowgraph
    this->message_port_register_out(pmt::mp("status"));

    // GPS Ephemeris data message port in
    this->message_port_register_in(pmt::mp("telemetry"));
    this->set_msg_handler(pmt::mp("telemetry"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_telemetry(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&rtklib_pvt_gs::msg_handler_telemetry, this, boost::placeholders::_1));
#else
        boost::bind(&rtklib_pvt_gs::msg_handler_telemetry, this, _1));
#endif
#endif

    // Galileo E6 HAS messages port in
    this->message_port_register_in(pmt::mp("E6_HAS_to_PVT"));
    this->set_msg_handler(pmt::mp("E6_HAS_to_PVT"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_has_data(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&rtklib_pvt_gs::msg_handler_has_data, this, boost::placeholders::_1));
#else
        boost::bind(&rtklib_pvt_gs::msg_handler_has_data, this, _1));
#endif
#endif

    d_initial_carrier_phase_offset_estimation_rads = std::vector<double>(nchannels, 0.0);
    d_channel_initialized = std::vector<bool>(nchannels, false);

    std::string dump_ls_pvt_filename = conf_.dump_filename;


    // char device[] = {"/dev/ttyUSB0"};
    // char device[] = {"/dev/ttyLP2"};
    char device[d_device_serial.length()+1];
    strcpy(device, d_device_serial.c_str());
    // comms = HEserial_connect(&device[0], B921600, O_RDWR | O_NDELAY | O_NOCTTY | O_NONBLOCK);
    comms = HEserial_connect(device, B921600, O_RDWR | O_NDELAY | O_NOCTTY | O_NONBLOCK);
    if (comms.fd == -1)
        {
            // std::cout << TEXT_BOLD_RED << "Falha ao abrir Porta Serial" << TEXT_RESET << "\n";
        }
    else
        {
            // std::cout << TEXT_BOLD_GREEN << "Port UART aberta: " << comms.fd << TEXT_RESET << "\n";
        }

    // // // Caio
    serial_temp_thread_ = std::thread(&rtklib_pvt_gs::serialcmd_, this);
    // sched_param sch_params;
    // int policy;
    // pthread_getschedparam(serial_temp_thread_.native_handle(), &policy, &sch_params);
    // sch_params.sched_priority = 80;
    // if (pthread_setschedparam(serial_temp_thread_.native_handle(), SCHED_FIFO, &sch_params))
    //     std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';


    if (d_dump)
        {
            std::string dump_path;
            // Get path
            if (d_dump_filename.find_last_of('/') != std::string::npos)
                {
                    std::string dump_filename_ = d_dump_filename.substr(d_dump_filename.find_last_of('/') + 1);
                    dump_path = d_dump_filename.substr(0, d_dump_filename.find_last_of('/'));
                    d_dump_filename = dump_filename_;
                }
            else
                {
                    dump_path = std::string(".");
                }
            if (d_dump_filename.empty())
                {
                    d_dump_filename = "pvt";
                }
            // remove extension if any
            if (d_dump_filename.substr(1).find_last_of('.') != std::string::npos)
                {
                    d_dump_filename = d_dump_filename.substr(0, d_dump_filename.find_last_of('.'));
                }
            dump_ls_pvt_filename = dump_path + fs::path::preferred_separator + d_dump_filename;
            dump_ls_pvt_filename.append(".dat");
            // create directory
            if (!gnss_sdr_create_directory(dump_path))
                {
                    std::cerr << "GNSS-SDR cannot create dump file for the PVT block. Wrong permissions?\n";
                    d_dump = false;
                }
        }   

    // PVT MONITOR
    if (d_flag_monitor_pvt_enabled)
        {
            std::string address_string = conf_.udp_addresses;
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            d_udp_sink_ptr = std::make_unique<Monitor_Pvt_Udp_Sink>(udp_addr_vec, conf_.udp_port, conf_.protobuf_enabled);
        }
    else
        {
            d_udp_sink_ptr = nullptr;
        }

    // EPHEMERIS MONITOR
    if (d_flag_monitor_ephemeris_enabled)
        {
            std::string address_string = conf_.udp_eph_addresses;
            std::vector<std::string> udp_addr_vec = split_string(address_string, '_');
            std::sort(udp_addr_vec.begin(), udp_addr_vec.end());
            udp_addr_vec.erase(std::unique(udp_addr_vec.begin(), udp_addr_vec.end()), udp_addr_vec.end());

            d_eph_udp_sink_ptr = std::make_unique<Monitor_Ephemeris_Udp_Sink>(udp_addr_vec, conf_.udp_eph_port, conf_.protobuf_enabled);
        }
    else
        {
            d_eph_udp_sink_ptr = nullptr;
        }

    // // Create Sys V message queue
    d_first_fix = true;
    d_sysv_msg_key = 1101;
    const int msgflg = IPC_CREAT | 0666;
    if ((d_sysv_msqid = msgget(d_sysv_msg_key, msgflg)) == -1)
        {
            // std::cout << "GNSS-SDR cannot create System V message queues.\n";
            // LOG(WARNING) << "The System V message queue is not available. Error: " << errno << " - " << strerror(errno);
        }

    // Display time in local time zone
    std::ostringstream os;
#ifdef HAS_PUT_TIME
    time_t when = std::time(nullptr);
    auto const tm = *std::localtime(&when);
    os << std::put_time(&tm, "%z");
#endif
    std::string utc_diff_str = os.str();  // in ISO 8601 format: "+HHMM" or "-HHMM"
    if (utc_diff_str.empty())
        {
            utc_diff_str = "+0000";
        }
    const int h = std::stoi(utc_diff_str.substr(0, 3), nullptr, 10);
    const int m = std::stoi(utc_diff_str[0] + utc_diff_str.substr(3), nullptr, 10);
    d_utc_diff_time = boost::posix_time::hours(h) + boost::posix_time::minutes(m);
    std::ostringstream os2;
#ifdef HAS_PUT_TIME
    os2 << std::put_time(&tm, "%Z");
#endif
    const std::string time_zone_abrv = os2.str();
    if (time_zone_abrv.empty())
        {
            if (utc_diff_str == "+0000")
                {
                    d_local_time_str = " UTC";
                }
            else
                {
                    d_local_time_str = " (UTC " + utc_diff_str.substr(0, 3) + ":" + utc_diff_str.substr(3, 2) + ")";
                }
        }
    else
        {
            d_local_time_str = std::string(" ") + time_zone_abrv + " (UTC " + utc_diff_str.substr(0, 3) + ":" + utc_diff_str.substr(3, 2) + ")";
        }

    if (d_enable_rx_clock_correction == true)
        {
            // setup two PVT solvers: internal solver for rx clock and user solver
            // user PVT solver
            d_user_pvt_solver = std::make_shared<Rtklib_Solver>(rtk, dump_ls_pvt_filename, d_type_of_rx, d_dump, d_dump_mat, d_use_e6_for_pvt);
            d_user_pvt_solver->set_averaging_depth(1);
            d_user_pvt_solver->set_pre_2009_file(conf_.pre_2009_file);

            // internal PVT solver, mainly used to estimate the receiver clock
            rtk_t internal_rtk = rtk;
            internal_rtk.opt.mode = PMODE_SINGLE;  // use single positioning mode in internal PVT solver
            d_internal_pvt_solver = std::make_shared<Rtklib_Solver>(internal_rtk, dump_ls_pvt_filename, d_type_of_rx, false, false, d_use_e6_for_pvt);
            d_internal_pvt_solver->set_averaging_depth(1);
            d_internal_pvt_solver->set_pre_2009_file(conf_.pre_2009_file);
        }
    else
        {
            // only one solver, customized by the user options
            d_internal_pvt_solver = std::make_shared<Rtklib_Solver>(rtk, dump_ls_pvt_filename, d_type_of_rx, d_dump, d_dump_mat, d_use_e6_for_pvt);
            d_internal_pvt_solver->set_averaging_depth(1);
            d_internal_pvt_solver->set_pre_2009_file(conf_.pre_2009_file);
            d_user_pvt_solver = d_internal_pvt_solver;
        }

    // set the RTKLIB trace (debug) level
    tracelevel(conf_.rtk_trace_level);

    // timetag
    if (d_log_timetag)
        {
            try
                {
                    d_log_timetag_file.open(conf_.log_source_timetag_file, std::ios::out | std::ios::binary);
                    // std::cout << "Log PVT timetag metadata enabled, log file: " << conf_.log_source_timetag_file << '\n';
                }
            catch (const std::exception& e)
                {
                    std::cerr << "Log PVT timetag metadata file cannot be created: " << e.what() << '\n';
                    d_log_timetag = false;
                }
        }

    d_start = std::chrono::system_clock::now();
}


rtklib_pvt_gs::~rtklib_pvt_gs()
{
    flag_interrupt_serial = true;
    // DLOG(INFO) << "PVT block destructor called.";
    if (d_sysv_msqid != -1)
        {
            msgctl(d_sysv_msqid, IPC_RMID, nullptr);
        }
    try
        {
            HEserial_disconnect(&comms);
            if (serial_temp_thread_.joinable())
                {
                    serial_temp_thread_.join();
                    std::cout << TEXT_CYAN << "serial_temp_thread_rtklib joined.." << TEXT_RESET << "\n";
                }
            
            pthread_t id6 = serial_temp_thread_.native_handle();
            serial_temp_thread_.detach();
            pthread_cancel(id6);

            if (d_log_timetag_file.is_open())
                {
                    try
                        {
                            d_log_timetag_file.close();
                        }
                    catch (const std::exception& e)
                        {
                            // LOG(WARNING) << "Problem closing Log PVT timetag metadata file: " << e.what();
                        }
                }
        }
    catch (const std::exception& e)
        {
            // LOG(WARNING) << e.what();
        }
}



void rtklib_pvt_gs::msg_handler_telemetry(const pmt::pmt_t& msg)
{
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            // ************************* GPS telemetry *************************
            if (msg_type_hash_code == d_gps_ephemeris_sptr_type_hash_code)
                {
                    // ### GPS EPHEMERIS ###
                    const auto gps_eph = wht::any_cast<std::shared_ptr<Gps_Ephemeris>>(pmt::any_ref(msg));
                    // DLOG(INFO) << "Ephemeris record has arrived from SAT ID "
                    //    << gps_eph->PRN << " (Block "
                    //    << gps_eph->satelliteBlock[gps_eph->PRN] << ")"
                    //    << "inserted with Toe=" << gps_eph->toe << " and GPS Week="
                    //    << gps_eph->WN;

                    // todo: Send only new sets of ephemeris (new TOE), not sent to the client
                    // send the new eph to the eph monitor (if enabled)
                    if (d_flag_monitor_ephemeris_enabled)
                        {
                            d_eph_udp_sink_ptr->write_gps_ephemeris(gps_eph);
                        }
                    // // update/insert new ephemeris record to the global ephemeris map
                    // if (d_rinex_output_enabled && d_rp->is_rinex_header_written())  // The header is already written, we can now log the navigation message data
                    //     {
                    //         bool new_annotation = false;
                    //         if (d_internal_pvt_solver->gps_ephemeris_map.find(gps_eph->PRN) == d_internal_pvt_solver->gps_ephemeris_map.cend())
                    //             {
                    //                 new_annotation = true;
                    //             }
                    //         else
                    //             {
                    //                 if (d_internal_pvt_solver->gps_ephemeris_map[gps_eph->PRN].toe != gps_eph->toe)
                    //                     {
                    //                         new_annotation = true;
                    //                     }
                    //             }
                    //         if (new_annotation == true)
                    //             {
                    //                 // New record!
                    //                 std::map<int32_t, Gps_Ephemeris> new_eph;
                    //                 new_eph[gps_eph->PRN] = *gps_eph;
                    //                 d_rp->log_rinex_nav_gps_nav(d_type_of_rx, new_eph);
                    //             }
                    //     }
                    d_internal_pvt_solver->gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_ephemeris_map[gps_eph->PRN] = *gps_eph;
                        }
                    if (gps_eph->SV_health != 0)
                        {
                            // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("GPS"), gps_eph->PRN)
                            //   << " reports an unhealthy status,";
                            if (d_use_unhealthy_sats)
                                {
                                    // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
                                }
                            else
                                {
                                    // std::cout << " not used for navigation" << TEXT_RESET << '\n';
                                }
                        }
                }
            else if (msg_type_hash_code == d_gps_iono_sptr_type_hash_code)
                {
                    // ### GPS IONO ###
                    const auto gps_iono = wht::any_cast<std::shared_ptr<Gps_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_iono = *gps_iono;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_iono = *gps_iono;
                        }
                    // DLOG(INFO) << "New IONO record has arrived";
                }
            else if (msg_type_hash_code == d_gps_utc_model_sptr_type_hash_code)
                {
                    // ### GPS UTC MODEL ###
                    const auto gps_utc_model = wht::any_cast<std::shared_ptr<Gps_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_utc_model = *gps_utc_model;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_utc_model = *gps_utc_model;
                        }
                    // DLOG(INFO) << "New UTC record has arrived";
                }
            else if (msg_type_hash_code == d_gps_cnav_ephemeris_sptr_type_hash_code)
                {
                    // ### GPS CNAV message ###
                    const auto gps_cnav_ephemeris = wht::any_cast<std::shared_ptr<Gps_CNAV_Ephemeris>>(pmt::any_ref(msg));
                    // update/insert new ephemeris record to the global ephemeris map
                    // if (d_rinex_output_enabled && d_rp->is_rinex_header_written())  // The header is already written, we can now log the navigation message data
                    //     {
                    //         bool new_annotation = false;
                    //         if (d_internal_pvt_solver->gps_cnav_ephemeris_map.find(gps_cnav_ephemeris->PRN) == d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                    //             {
                    //                 new_annotation = true;
                    //             }
                    //         else
                    //             {
                    //                 if (d_internal_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->PRN].toe1 != gps_cnav_ephemeris->toe1)
                    //                     {
                    //                         new_annotation = true;
                    //                     }
                    //             }
                    //         if (new_annotation == true)
                    //             {
                    //                 // New record!
                    //                 std::map<int32_t, Gps_CNAV_Ephemeris> new_cnav_eph;
                    //                 new_cnav_eph[gps_cnav_ephemeris->PRN] = *gps_cnav_ephemeris;
                    //                 d_rp->log_rinex_nav_gps_cnav(d_type_of_rx, new_cnav_eph);
                    //             }
                    //     }
                    d_internal_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->PRN] = *gps_cnav_ephemeris;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_cnav_ephemeris_map[gps_cnav_ephemeris->PRN] = *gps_cnav_ephemeris;
                        }
                    if (gps_cnav_ephemeris->signal_health != 0)
                        {
                            // std::cout << "Satellite " << Gnss_Satellite(std::string("GPS"), gps_cnav_ephemeris->PRN)
                            //   << " reports an unhealthy status in the CNAV message,";
                            if (d_use_unhealthy_sats)
                                {
                                    // std::cout << " use PVT solutions at your own risk.\n";
                                }
                            else
                                {
                                    // std::cout << " not used for navigation.\n";
                                }
                        }
                    // DLOG(INFO) << "New GPS CNAV ephemeris record has arrived";
                }
            else if (msg_type_hash_code == d_gps_cnav_iono_sptr_type_hash_code)
                {
                    // ### GPS CNAV IONO ###
                    const auto gps_cnav_iono = wht::any_cast<std::shared_ptr<Gps_CNAV_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_cnav_iono = *gps_cnav_iono;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_cnav_iono = *gps_cnav_iono;
                        }
                    // DLOG(INFO) << "New CNAV IONO record has arrived";
                }
            else if (msg_type_hash_code == d_gps_cnav_utc_model_sptr_type_hash_code)
                {
                    // ### GPS CNAV UTC MODEL ###
                    const auto gps_cnav_utc_model = wht::any_cast<std::shared_ptr<Gps_CNAV_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_cnav_utc_model = *gps_cnav_utc_model;
                    {
                        d_user_pvt_solver->gps_cnav_utc_model = *gps_cnav_utc_model;
                    }
                    // DLOG(INFO) << "New CNAV UTC record has arrived";
                }

            else if (msg_type_hash_code == d_gps_almanac_sptr_type_hash_code)
                {
                    // ### GPS ALMANAC ###
                    const auto gps_almanac = wht::any_cast<std::shared_ptr<Gps_Almanac>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->gps_almanac_map[gps_almanac->PRN] = *gps_almanac;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->gps_almanac_map[gps_almanac->PRN] = *gps_almanac;
                        }
                    // DLOG(INFO) << "New GPS almanac record has arrived";
                }

            // *********************** Galileo telemetry ***********************
            else if (msg_type_hash_code == d_galileo_ephemeris_sptr_type_hash_code)
                {
                    // ### Galileo EPHEMERIS ###
                    const auto galileo_eph = wht::any_cast<std::shared_ptr<Galileo_Ephemeris>>(pmt::any_ref(msg));
                    // insert new ephemeris record
                    // DLOG(INFO) << "Galileo New Ephemeris record inserted in global map with TOW =" << galileo_eph->tow
                    //    << ", GALILEO Week Number =" << galileo_eph->WN
                    //    << " and Ephemeris IOD = " << galileo_eph->IOD_ephemeris;
                    // todo: Send only new sets of ephemeris (new TOE), not sent to the client
                    // send the new eph to the eph monitor (if enabled)
                    if (d_flag_monitor_ephemeris_enabled)
                        {
                            d_eph_udp_sink_ptr->write_galileo_ephemeris(galileo_eph);
                        }
                    // update/insert new ephemeris record to the global ephemeris map
                    // if (d_rinex_output_enabled && d_rp->is_rinex_header_written())  // The header is already written, we can now log the navigation message data
                    //     {
                    //         bool new_annotation = false;
                    //         if (d_internal_pvt_solver->galileo_ephemeris_map.find(galileo_eph->PRN) == d_internal_pvt_solver->galileo_ephemeris_map.cend())
                    //             {
                    //                 new_annotation = true;
                    //             }
                    //         else
                    //             {
                    //                 if (d_internal_pvt_solver->galileo_ephemeris_map[galileo_eph->PRN].toe != galileo_eph->toe)
                    //                     {
                    //                         new_annotation = true;
                    //                     }
                    //             }
                    //         if (new_annotation == true)
                    //             {
                    //                 // New record!
                    //                 std::map<int32_t, Galileo_Ephemeris> new_gal_eph;
                    //                 new_gal_eph[galileo_eph->PRN] = *galileo_eph;
                    //                 d_rp->log_rinex_nav_gal_nav(d_type_of_rx, new_gal_eph);
                    //             }
                    //     }
                    d_internal_pvt_solver->galileo_ephemeris_map[galileo_eph->PRN] = *galileo_eph;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->galileo_ephemeris_map[galileo_eph->PRN] = *galileo_eph;
                        }
                    if (((galileo_eph->E1B_HS != 0) || (galileo_eph->E1B_DVS == true)) ||
                        ((galileo_eph->E5a_HS != 0) || (galileo_eph->E5a_DVS == true)) ||
                        ((galileo_eph->E5b_HS != 0) || (galileo_eph->E5b_DVS == true)))
                        {
                            // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("Galileo"), galileo_eph->PRN)
                            //   << " reports an unhealthy status,";
                            if (d_use_unhealthy_sats)
                                {
                                    // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
                                }
                            else
                                {
                                    // std::cout << " not used for navigation" << TEXT_RESET << '\n';
                                }
                        }
                }
            else if (msg_type_hash_code == d_galileo_iono_sptr_type_hash_code)
                {
                    // ### Galileo IONO ###
                    const auto galileo_iono = wht::any_cast<std::shared_ptr<Galileo_Iono>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->galileo_iono = *galileo_iono;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->galileo_iono = *galileo_iono;
                        }
                    // DLOG(INFO) << "New IONO record has arrived";
                }
            else if (msg_type_hash_code == d_galileo_utc_model_sptr_type_hash_code)
                {
                    // ### Galileo UTC MODEL ###
                    const auto galileo_utc_model = wht::any_cast<std::shared_ptr<Galileo_Utc_Model>>(pmt::any_ref(msg));
                    d_internal_pvt_solver->galileo_utc_model = *galileo_utc_model;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->galileo_utc_model = *galileo_utc_model;
                        }
                    // DLOG(INFO) << "New UTC record has arrived";
                }
            else if (msg_type_hash_code == d_galileo_almanac_helper_sptr_type_hash_code)
                {
                    // ### Galileo Almanac ###
                    const auto galileo_almanac_helper = wht::any_cast<std::shared_ptr<Galileo_Almanac_Helper>>(pmt::any_ref(msg));
                    const Galileo_Almanac sv1 = galileo_almanac_helper->get_almanac(1);
                    const Galileo_Almanac sv2 = galileo_almanac_helper->get_almanac(2);
                    const Galileo_Almanac sv3 = galileo_almanac_helper->get_almanac(3);

                    if (sv1.PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv1.PRN] = sv1;
                            if (d_enable_rx_clock_correction == true)
                                {
                                    d_user_pvt_solver->galileo_almanac_map[sv1.PRN] = sv1;
                                }
                        }
                    if (sv2.PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv2.PRN] = sv2;
                            if (d_enable_rx_clock_correction == true)
                                {
                                    d_user_pvt_solver->galileo_almanac_map[sv2.PRN] = sv2;
                                }
                        }
                    if (sv3.PRN != 0)
                        {
                            d_internal_pvt_solver->galileo_almanac_map[sv3.PRN] = sv3;
                            if (d_enable_rx_clock_correction == true)
                                {
                                    d_user_pvt_solver->galileo_almanac_map[sv3.PRN] = sv3;
                                }
                        }
                    // DLOG(INFO) << "New Galileo Almanac data have arrived";
                }
            else if (msg_type_hash_code == d_galileo_almanac_sptr_type_hash_code)
                {
                    // ### Galileo Almanac ###
                    const auto galileo_alm = wht::any_cast<std::shared_ptr<Galileo_Almanac>>(pmt::any_ref(msg));
                    // update/insert new almanac record to the global almanac map
                    d_internal_pvt_solver->galileo_almanac_map[galileo_alm->PRN] = *galileo_alm;
                    if (d_enable_rx_clock_correction == true)
                        {
                            d_user_pvt_solver->galileo_almanac_map[galileo_alm->PRN] = *galileo_alm;
                        }
                }

            // // **************** GLONASS GNAV Telemetry *************************
            // else if (msg_type_hash_code == d_glonass_gnav_ephemeris_sptr_type_hash_code)
            //     {
            //         // ### GLONASS GNAV EPHEMERIS ###
            //         const auto glonass_gnav_eph = wht::any_cast<std::shared_ptr<Glonass_Gnav_Ephemeris>>(pmt::any_ref(msg));
            //         // TODO Add GLONASS with gps week number and tow,
            //         // insert new ephemeris record
            //         // DLOG(INFO) << "GLONASS GNAV New Ephemeris record inserted in global map with TOW =" << glonass_gnav_eph->d_TOW
            //                    << ", Week Number =" << glonass_gnav_eph->d_WN
            //                    << " and Ephemeris IOD in UTC = " << glonass_gnav_eph->compute_GLONASS_time(glonass_gnav_eph->d_t_b)
            //                    << " from SV = " << glonass_gnav_eph->i_satellite_slot_number;
            //         // update/insert new ephemeris record to the global ephemeris map
            //         // if (d_rinex_output_enabled && d_rp->is_rinex_header_written())  // The header is already written, we can now log the navigation message data
            //         //     {
            //         //         bool new_annotation = false;
            //         //         if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(glonass_gnav_eph->PRN) == d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
            //         //             {
            //         //                 new_annotation = true;
            //         //             }
            //         //         else
            //         //             {
            //         //                 if (d_internal_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->PRN].d_t_b != glonass_gnav_eph->d_t_b)
            //         //                     {
            //         //                         new_annotation = true;
            //         //                     }
            //         //             }
            //         //         if (new_annotation == true)
            //         //             {
            //         //                 // New record!
            //         //                 std::map<int32_t, Glonass_Gnav_Ephemeris> new_glo_eph;
            //         //                 new_glo_eph[glonass_gnav_eph->PRN] = *glonass_gnav_eph;
            //         //                 d_rp->log_rinex_nav_glo_gnav(d_type_of_rx, new_glo_eph);
            //         //             }
            //         //     }
            //         d_internal_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->PRN] = *glonass_gnav_eph;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->glonass_gnav_ephemeris_map[glonass_gnav_eph->PRN] = *glonass_gnav_eph;
            //             }
            //     }
            // else if (msg_type_hash_code == d_glonass_gnav_utc_model_sptr_type_hash_code)
            //     {
            //         // ### GLONASS GNAV UTC MODEL ###
            //         const auto glonass_gnav_utc_model = wht::any_cast<std::shared_ptr<Glonass_Gnav_Utc_Model>>(pmt::any_ref(msg));
            //         d_internal_pvt_solver->glonass_gnav_utc_model = *glonass_gnav_utc_model;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->glonass_gnav_utc_model = *glonass_gnav_utc_model;
            //             }
            //         // DLOG(INFO) << "New GLONASS GNAV UTC record has arrived";
            //     }
            // else if (msg_type_hash_code == d_glonass_gnav_almanac_sptr_type_hash_code)
            //     {
            //         // ### GLONASS GNAV Almanac ###
            //         const auto glonass_gnav_almanac = wht::any_cast<std::shared_ptr<Glonass_Gnav_Almanac>>(pmt::any_ref(msg));
            //         d_internal_pvt_solver->glonass_gnav_almanac = *glonass_gnav_almanac;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->glonass_gnav_almanac = *glonass_gnav_almanac;
            //             }
            //         // DLOG(INFO) << "New GLONASS GNAV Almanac has arrived"
            //                    << ", GLONASS GNAV Slot Number =" << glonass_gnav_almanac->d_n_A;
            //     }

            // // *********************** BeiDou telemetry ************************
            // else if (msg_type_hash_code == d_beidou_dnav_ephemeris_sptr_type_hash_code)
            //     {
            //         // ### Beidou EPHEMERIS ###
            //         const auto bds_dnav_eph = wht::any_cast<std::shared_ptr<Beidou_Dnav_Ephemeris>>(pmt::any_ref(msg));
            //         // DLOG(INFO) << "Ephemeris record has arrived from SAT ID "
            //                    << bds_dnav_eph->PRN << " (Block "
            //                    << bds_dnav_eph->satelliteBlock[bds_dnav_eph->PRN] << ")"
            //                    << "inserted with Toe=" << bds_dnav_eph->toe << " and BDS Week="
            //                    << bds_dnav_eph->WN;
            //         // update/insert new ephemeris record to the global ephemeris map
            //         // if (d_rinex_output_enabled && d_rp->is_rinex_header_written())  // The header is already written, we can now log the navigation message data
            //         //     {
            //         //         bool new_annotation = false;
            //         //         if (d_internal_pvt_solver->beidou_dnav_ephemeris_map.find(bds_dnav_eph->PRN) == d_internal_pvt_solver->beidou_dnav_ephemeris_map.cend())
            //         //             {
            //         //                 new_annotation = true;
            //         //             }
            //         //         else
            //         //             {
            //         //                 if (d_internal_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->PRN].toc != bds_dnav_eph->toc)
            //         //                     {
            //         //                         new_annotation = true;
            //         //                     }
            //         //             }
            //         //         if (new_annotation == true)
            //         //             {
            //         //                 // New record!
            //         //                 std::map<int32_t, Beidou_Dnav_Ephemeris> new_bds_eph;
            //         //                 new_bds_eph[bds_dnav_eph->PRN] = *bds_dnav_eph;
            //         //                 d_rp->log_rinex_nav_bds_dnav(d_type_of_rx, new_bds_eph);
            //         //             }
            //         //     }
            //         d_internal_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->PRN] = *bds_dnav_eph;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->beidou_dnav_ephemeris_map[bds_dnav_eph->PRN] = *bds_dnav_eph;
            //             }
            //         if (bds_dnav_eph->SV_health != 0)
            //             {
            //                 // std::cout << TEXT_RED << "Satellite " << Gnss_Satellite(std::string("Beidou"), bds_dnav_eph->PRN)
            //                           << " reports an unhealthy status,";
            //                 if (d_use_unhealthy_sats)
            //                     {
            //                         // std::cout << " use PVT solutions at your own risk" << TEXT_RESET << '\n';
            //                     }
            //                 else
            //                     {
            //                         // std::cout << " not used for navigation" << TEXT_RESET << '\n';
            //                     }
            //             }
            //     }
            // else if (msg_type_hash_code == d_beidou_dnav_iono_sptr_type_hash_code)
            //     {
            //         // ### BeiDou IONO ###
            //         const auto bds_dnav_iono = wht::any_cast<std::shared_ptr<Beidou_Dnav_Iono>>(pmt::any_ref(msg));
            //         d_internal_pvt_solver->beidou_dnav_iono = *bds_dnav_iono;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->beidou_dnav_iono = *bds_dnav_iono;
            //             }
            //         // DLOG(INFO) << "New BeiDou DNAV IONO record has arrived";
            //     }
            // else if (msg_type_hash_code == d_beidou_dnav_utc_model_sptr_type_hash_code)
            //     {
            //         // ### BeiDou UTC MODEL ###
            //         const auto bds_dnav_utc_model = wht::any_cast<std::shared_ptr<Beidou_Dnav_Utc_Model>>(pmt::any_ref(msg));
            //         d_internal_pvt_solver->beidou_dnav_utc_model = *bds_dnav_utc_model;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->beidou_dnav_utc_model = *bds_dnav_utc_model;
            //             }
            //         // DLOG(INFO) << "New BeiDou DNAV UTC record has arrived";
            //     }
            // else if (msg_type_hash_code == d_beidou_dnav_almanac_sptr_type_hash_code)
            //     {
            //         // ### BeiDou ALMANAC ###
            //         const auto bds_dnav_almanac = wht::any_cast<std::shared_ptr<Beidou_Dnav_Almanac>>(pmt::any_ref(msg));
            //         d_internal_pvt_solver->beidou_dnav_almanac_map[bds_dnav_almanac->PRN] = *bds_dnav_almanac;
            //         if (d_enable_rx_clock_correction == true)
            //             {
            //                 d_user_pvt_solver->beidou_dnav_almanac_map[bds_dnav_almanac->PRN] = *bds_dnav_almanac;
            //             }
            //         // DLOG(INFO) << "New BeiDou DNAV almanac record has arrived";
            //     }
            else
                {
                    // LOG(WARNING) << "msg_handler_telemetry unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            // LOG(WARNING) << "msg_handler_telemetry Bad any_cast: " << e.what();
        }
}


void rtklib_pvt_gs::msg_handler_has_data(const pmt::pmt_t& msg)
{
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == d_galileo_has_data_sptr_type_hash_code)
                {
                    const auto has_data = wht::any_cast<std::shared_ptr<Galileo_HAS_data>>(pmt::any_ref(msg));
                    if (d_use_has_corrections && (has_data->has_status == 1))  // operational mode
                        {
                            d_internal_pvt_solver->store_has_data(*has_data);
                            if (d_enable_rx_clock_correction == true)
                                {
                                    d_user_pvt_solver->store_has_data(*has_data);
                                }
                        }
                    // if (d_has_simple_printer)
                    //     {
                    //         d_has_simple_printer->print_message(has_data.get());
                    //     }
                    // if (d_rtcm_printer && has_data->tow <= 604800)
                    //     {
                    //         d_rtcm_printer->Print_IGM_Messages(*has_data.get());
                    //     }
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            // LOG(WARNING) << "msg_handler_has_data Bad any_cast: " << e.what();
        }
}


std::map<int, Gps_Ephemeris> rtklib_pvt_gs::get_gps_ephemeris_map() const
{
    // return std::make_shared pvt_map_internal = d_internal_pvt_solver->gps_ephemeris_map;
    return d_internal_pvt_solver->gps_ephemeris_map;
}


std::map<int, Gps_Almanac> rtklib_pvt_gs::get_gps_almanac_map() const
{
    return d_internal_pvt_solver->gps_almanac_map;
}


std::map<int, Galileo_Ephemeris> rtklib_pvt_gs::get_galileo_ephemeris_map() const
{
    return d_internal_pvt_solver->galileo_ephemeris_map;
}


std::map<int, Galileo_Almanac> rtklib_pvt_gs::get_galileo_almanac_map() const
{
    return d_internal_pvt_solver->galileo_almanac_map;
}


// std::map<int, Beidou_Dnav_Ephemeris> rtklib_pvt_gs::get_beidou_dnav_ephemeris_map() const
// {
//     return d_internal_pvt_solver->beidou_dnav_ephemeris_map;
// }


std::map<int, Gnss_Synchro> rtklib_pvt_gs::get_observables_map() const
{
    return d_internal_pvt_solver->gnss_observables_map;
}


void rtklib_pvt_gs::clear_ephemeris()
{
    d_internal_pvt_solver->gps_ephemeris_map.clear();
    d_internal_pvt_solver->gps_almanac_map.clear();
    d_internal_pvt_solver->galileo_ephemeris_map.clear();
    d_internal_pvt_solver->galileo_almanac_map.clear();
    d_internal_pvt_solver->beidou_dnav_ephemeris_map.clear();
    d_internal_pvt_solver->beidou_dnav_almanac_map.clear();
    d_internal_pvt_solver->gnss_observables_map.clear();  // Caio
    d_internal_pvt_solver->Gnss_Ephem_map.clear();        // Caio
    if (d_enable_rx_clock_correction == true)
        {
            d_user_pvt_solver->gps_ephemeris_map.clear();
            d_user_pvt_solver->gps_almanac_map.clear();
            d_user_pvt_solver->galileo_ephemeris_map.clear();
            d_user_pvt_solver->galileo_almanac_map.clear();
            d_user_pvt_solver->beidou_dnav_ephemeris_map.clear();
            d_user_pvt_solver->beidou_dnav_almanac_map.clear();
            d_user_pvt_solver->gnss_observables_map.clear();  // Caio
            d_user_pvt_solver->Gnss_Ephem_map.clear();        // Caio
        }
}


bool rtklib_pvt_gs::send_sys_v_ttff_msg(d_ttff_msgbuf ttff) const
{
    if (d_sysv_msqid != -1)
        {
            // Fill Sys V message structures
            int msgsend_size;
            d_ttff_msgbuf msg;
            msg.ttff = ttff.ttff;
            msgsend_size = sizeof(msg.ttff);
            msg.mtype = 1;  // default message ID

            // SEND SOLUTION OVER A MESSAGE QUEUE
            // non-blocking Sys V message send
            msgsnd(d_sysv_msqid, &msg, msgsend_size, IPC_NOWAIT);
            return true;
        }
    return false;
}


bool rtklib_pvt_gs::save_gnss_synchro_map_xml(const std::string& file_name)
{
    if (d_gnss_observables_map.empty() == false)
        {
            std::ofstream ofs;
            try
                {
                    ofs.open(file_name.c_str(), std::ofstream::trunc | std::ofstream::out);
                    boost::archive::xml_oarchive xml(ofs);
                    xml << boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", d_gnss_observables_map);
                    // LOG(INFO) << "Saved gnss_sychro map data";
                }
            catch (const std::exception& e)
                {
                    // LOG(WARNING) << e.what();
                    return false;
                }
            return true;
        }

    // LOG(WARNING) << "Failed to save gnss_synchro, map is empty";
    return false;
}


void rtklib_pvt_gs::log_source_timetag_info(double RX_time_ns, double TAG_time_ns)
{
    if (d_log_timetag_file.is_open())
        {
            try
                {
                    d_log_timetag_file.write(reinterpret_cast<char*>(&RX_time_ns), sizeof(double));
                    d_log_timetag_file.write(reinterpret_cast<char*>(&TAG_time_ns), sizeof(double));
                }
            catch (const std::exception& e)
                {
                    std::cerr << "Problem writing at the log PVT timetag metadata file: " << e.what() << '\n';
                }
        }
}


bool rtklib_pvt_gs::load_gnss_synchro_map_xml(const std::string& file_name)
{
    // load from xml (boost serialize)
    std::ifstream ifs;
    try
        {
            ifs.open(file_name.c_str(), std::ifstream::binary | std::ifstream::in);
            boost::archive::xml_iarchive xml(ifs);
            d_gnss_observables_map.clear();
            xml >> boost::serialization::make_nvp("GNSS-SDR_gnss_synchro_map", d_gnss_observables_map);
            // // std::cout << "Loaded gnss_synchro map data with " << gnss_synchro_map.size() << " pseudoranges\n";
        }
    catch (const std::exception& e)
        {
            // std::cout << e.what() << "File: " << file_name;
            return false;
        }
    return true;
}


std::vector<std::string> rtklib_pvt_gs::split_string(const std::string& s, char delim) const
{
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delim))
        {
            *(std::back_inserter(v)++) = item;
        }

    return v;
}


bool rtklib_pvt_gs::get_latest_PVT(double* longitude_deg,
    double* latitude_deg,
    double* height_m,
    double* ground_speed_kmh,
    double* course_over_ground_deg,
    time_t* UTC_time) const
{
    if (d_enable_rx_clock_correction == true)
        {
            if (d_user_pvt_solver->is_valid_position())
                {
                    *latitude_deg = d_user_pvt_solver->get_latitude();
                    *longitude_deg = d_user_pvt_solver->get_longitude();
                    *height_m = d_user_pvt_solver->get_height();
                    *ground_speed_kmh = d_user_pvt_solver->get_speed_over_ground() * 3600.0 / 1000.0;
                    *course_over_ground_deg = d_user_pvt_solver->get_course_over_ground();
                    *UTC_time = convert_to_time_t(d_user_pvt_solver->get_position_UTC_time());

                    return true;
                }
        }
    else
        {
            if (d_internal_pvt_solver->is_valid_position())
                {
                    *latitude_deg = d_internal_pvt_solver->get_latitude();
                    *longitude_deg = d_internal_pvt_solver->get_longitude();
                    *height_m = d_internal_pvt_solver->get_height();
                    *ground_speed_kmh = d_internal_pvt_solver->get_speed_over_ground() * 3600.0 / 1000.0;
                    *course_over_ground_deg = d_internal_pvt_solver->get_course_over_ground();
                    *UTC_time = convert_to_time_t(d_internal_pvt_solver->get_position_UTC_time());

                    return true;
                }
        }


    return false;
}


bool rtklib_pvt_gs::get_latest_PVT_(double* longitude_deg,
    double* latitude_deg,
    double* height_m,
    time_t* UTC_time,
    double* gps_time_offset,
    double* rx_posX,
    double* rx_posY,
    double* rx_posZ,
    double* rx_velX,
    double* rx_velY,
    double* rx_velZ)
{
    if (d_enable_rx_clock_correction == true)
        {
            if (d_user_pvt_solver->is_valid_position())
                {
                    *latitude_deg = d_user_pvt_solver->get_latitude();
                    *longitude_deg = d_user_pvt_solver->get_longitude();
                    *height_m = d_user_pvt_solver->get_height();
                    *UTC_time = convert_to_time_t(d_user_pvt_solver->get_position_UTC_time());
                    // rtklib_ptr = std::make_shared<Rtklib_Solver> (d_user_pvt_solver);

                    *gps_time_offset = d_user_pvt_solver->get_time_offset_s();
                    *rx_posX = d_user_pvt_solver->get_rx_pos()[0];
                    *rx_posY = d_user_pvt_solver->get_rx_pos()[1];
                    *rx_posZ = d_user_pvt_solver->get_rx_pos()[2];
                    *rx_velX = d_user_pvt_solver->get_rx_vel()[0];
                    *rx_velY = d_user_pvt_solver->get_rx_vel()[1];
                    *rx_velZ = d_user_pvt_solver->get_rx_vel()[2];
                    return true;
                }
        }
    else
        {
            if (d_internal_pvt_solver->is_valid_position())
                {
                    *latitude_deg = d_internal_pvt_solver->get_latitude();
                    *longitude_deg = d_internal_pvt_solver->get_longitude();
                    *height_m = d_internal_pvt_solver->get_height();
                    *UTC_time = convert_to_time_t(d_internal_pvt_solver->get_position_UTC_time());
                    // rtklib_ptr = std::make_shared<Rtklib_Solver> (d_user_pvt_solver);

                    *gps_time_offset = d_user_pvt_solver->get_time_offset_s();
                    *rx_posX = d_user_pvt_solver->get_rx_pos()[0];
                    *rx_posY = d_user_pvt_solver->get_rx_pos()[1];
                    *rx_posZ = d_user_pvt_solver->get_rx_pos()[2];
                    *rx_velX = d_user_pvt_solver->get_rx_vel()[0];
                    *rx_velY = d_user_pvt_solver->get_rx_vel()[1];
                    *rx_velZ = d_user_pvt_solver->get_rx_vel()[2];
                    return true;
                }
        }

    return false;
}

bool rtklib_pvt_gs::get_latest_PVT_2(double* rx_posX,
    double* rx_posY,
    double* rx_posZ,
    double* rx_velX,
    double* rx_velY,
    double* rx_velZ,
    time_t* UTC_time,
    double* gdop,
    double* hdop,
    double* vdop,
    double* pdop)
{
    if (d_enable_rx_clock_correction == true)
        {
            if (d_user_pvt_solver->is_valid_position())
                {
                    *rx_posX = d_user_pvt_solver->get_rx_pos()[0];
                    *rx_posY = d_user_pvt_solver->get_rx_pos()[1];
                    *rx_posZ = d_user_pvt_solver->get_rx_pos()[2];
                    *rx_velX = d_user_pvt_solver->get_rx_vel()[0];
                    *rx_velY = d_user_pvt_solver->get_rx_vel()[1];
                    *rx_velZ = d_user_pvt_solver->get_rx_vel()[2];
                    *UTC_time = convert_to_time_t(d_user_pvt_solver->get_position_UTC_time());
                    // rtklib_ptr = std::make_shared<Rtklib_Solver> (d_user_pvt_solver);

                    *gdop = d_user_pvt_solver->get_gdop();
                    *hdop = d_user_pvt_solver->get_hdop();
                    *vdop = d_user_pvt_solver->get_vdop();
                    *pdop = d_user_pvt_solver->get_pdop();
                    return true;
                }
        }
    else
        {
            if (d_internal_pvt_solver->is_valid_position())
                {
                    *rx_posX = d_internal_pvt_solver->get_rx_pos()[0];
                    *rx_posY = d_internal_pvt_solver->get_rx_pos()[1];
                    *rx_posZ = d_internal_pvt_solver->get_rx_pos()[2];
                    *rx_velX = d_internal_pvt_solver->get_rx_vel()[0];
                    *rx_velY = d_internal_pvt_solver->get_rx_vel()[1];
                    *rx_velZ = d_internal_pvt_solver->get_rx_vel()[2];
                    *UTC_time = convert_to_time_t(d_internal_pvt_solver->get_position_UTC_time());
                    // rtklib_ptr = std::make_shared<Rtklib_Solver> (d_user_pvt_solver);

                    *gdop = d_internal_pvt_solver->get_gdop();
                    *hdop = d_internal_pvt_solver->get_hdop();
                    *vdop = d_internal_pvt_solver->get_vdop();
                    *pdop = d_internal_pvt_solver->get_pdop();
                    return true;
                }
        }
    return false;
}

// caio
int rtklib_pvt_gs::get_num_sat_observ(void)
{
    return d_user_pvt_solver->get_num_valid_observations();
}
bool rtklib_pvt_gs::got_first_fix(void)
{
    return first_fix;
}
//

void rtklib_pvt_gs::apply_rx_clock_offset(std::map<int, Gnss_Synchro>& observables_map,
    double rx_clock_offset_s)
{
    // apply corrections according to Rinex 3.04, Table 1: Observation Corrections for Receiver Clock Offset
    std::map<int, Gnss_Synchro>::iterator observables_iter;

    for (observables_iter = observables_map.begin(); observables_iter != observables_map.end(); observables_iter++)
        {
            // all observables in the map are valid
            observables_iter->second.RX_time -= rx_clock_offset_s;
            observables_iter->second.Pseudorange_m -= rx_clock_offset_s * SPEED_OF_LIGHT_M_S;
            const auto it_freq_map = SIGNAL_FREQ_MAP.find(std::string(observables_iter->second.Signal, 2));
            if (it_freq_map != SIGNAL_FREQ_MAP.cend())
                {
                    observables_iter->second.Carrier_phase_rads -= rx_clock_offset_s * it_freq_map->second * TWO_PI;
                }
        }
}


std::map<int, Gnss_Synchro> rtklib_pvt_gs::interpolate_observables(const std::map<int, Gnss_Synchro>& observables_map_t0,
    const std::map<int, Gnss_Synchro>& observables_map_t1,
    double rx_time_s)
{
    std::map<int, Gnss_Synchro> interp_observables_map;
    // Linear interpolation: y(t) = y(t0) + (y(t1) - y(t0)) * (t - t0) / (t1 - t0)

    // check TOW rollover
    double time_factor;
    if ((observables_map_t1.cbegin()->second.RX_time -
            observables_map_t0.cbegin()->second.RX_time) > 0)
        {
            time_factor = (rx_time_s - observables_map_t0.cbegin()->second.RX_time) /
                          (observables_map_t1.cbegin()->second.RX_time -
                              observables_map_t0.cbegin()->second.RX_time);
        }
    else
        {
            // TOW rollover situation
            time_factor = (604800000.0 + rx_time_s - observables_map_t0.cbegin()->second.RX_time) /
                          (604800000.0 + observables_map_t1.cbegin()->second.RX_time -
                              observables_map_t0.cbegin()->second.RX_time);
        }

    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    for (observables_iter = observables_map_t0.cbegin(); observables_iter != observables_map_t0.cend(); observables_iter++)
        {
            // 1. Check if the observable exist in t0 and t1
            // the map key is the channel ID (see work())
            try
                {
                    if (observables_map_t1.at(observables_iter->first).PRN == observables_iter->second.PRN)
                        {
                            interp_observables_map.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                            interp_observables_map.at(observables_iter->first).RX_time = rx_time_s;  // interpolation point
                            interp_observables_map.at(observables_iter->first).Pseudorange_m += (observables_map_t1.at(observables_iter->first).Pseudorange_m - observables_iter->second.Pseudorange_m) * time_factor;
                            interp_observables_map.at(observables_iter->first).Carrier_phase_rads += (observables_map_t1.at(observables_iter->first).Carrier_phase_rads - observables_iter->second.Carrier_phase_rads) * time_factor;
                            interp_observables_map.at(observables_iter->first).Carrier_Doppler_hz += (observables_map_t1.at(observables_iter->first).Carrier_Doppler_hz - observables_iter->second.Carrier_Doppler_hz) * time_factor;
                        }
                }
            catch (const std::out_of_range& oor)
                {
                    // observable does not exist in t1
                }
        }
    return interp_observables_map;
}


void rtklib_pvt_gs::initialize_and_apply_carrier_phase_offset()
{
    // we have a valid PVT. First check if we need to reset the initial carrier phase offsets to match their pseudoranges
    std::map<int, Gnss_Synchro>::iterator observables_iter;
    d_internal_pvt_solver->gnss_observables_map = d_gnss_observables_map;
    // d_internal_pvt_solver = std::make_shared<Rtklib_Solver>
    for (observables_iter = d_gnss_observables_map.begin(); observables_iter != d_gnss_observables_map.end(); observables_iter++)
        {
            // check if an initialization is required (new satellite or loss of lock)
            // it is set to false by the work function if the gnss_synchro is not valid
            if (d_channel_initialized.at(observables_iter->second.Channel_ID) == false)
                {
                    double wavelength_m = 1.0;
                    const auto it_freq_map = SIGNAL_FREQ_MAP.find(std::string(observables_iter->second.Signal, 2));
                    if (it_freq_map != SIGNAL_FREQ_MAP.cend())
                        {
                            wavelength_m = SPEED_OF_LIGHT_M_S / it_freq_map->second;
                        }
                    const double wrap_carrier_phase_rad = fmod(observables_iter->second.Carrier_phase_rads, TWO_PI);
                    d_initial_carrier_phase_offset_estimation_rads.at(observables_iter->second.Channel_ID) = TWO_PI * round(observables_iter->second.Pseudorange_m / wavelength_m) - observables_iter->second.Carrier_phase_rads + wrap_carrier_phase_rad;
                    d_channel_initialized.at(observables_iter->second.Channel_ID) = true;
                    // DLOG(INFO) << "initialized carrier phase at channel " << observables_iter->second.Channel_ID;
                }
            // apply the carrier phase offset to this satellite
            observables_iter->second.Carrier_phase_rads = observables_iter->second.Carrier_phase_rads + d_initial_carrier_phase_offset_estimation_rads.at(observables_iter->second.Channel_ID);
        }
}


void rtklib_pvt_gs::update_HAS_corrections()
{
    this->d_internal_pvt_solver->update_has_corrections(this->d_gnss_observables_map);
    if (d_enable_rx_clock_correction == true)
        {
            this->d_user_pvt_solver->update_has_corrections(this->d_gnss_observables_map);
        }
}


int rtklib_pvt_gs::work(int noutput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{

    // Caio
    // double timepvt;
    // std::ofstream fileRx("RxX_sampled_EPHEM_.txt", std::ios::out | std::ios::app);
    // std::ofstream filePosRecp("RxX_sampled_PVT.txt", std::ios::out | std::ios::app);
    // std::ofstream fileRxx("Rx_sampled_EPHEM_transmit_time.txt", std::ios::out | std::ios::app);
    // *************** time tags ****************
    if (d_enable_rx_clock_correction == false)  // todo: currently only works if clock correction is disabled
        {
            std::vector<gr::tag_t> tags_vec;
            // time tag from obs to pvt is always propagated in channel 0
            this->get_tags_in_range(tags_vec, 0, this->nitems_read(0), this->nitems_read(0) + noutput_items);
            for (const auto& it : tags_vec)
                {
                    try
                        {
                            if (pmt::any_ref(it.value).type().hash_code() == typeid(const std::shared_ptr<GnssTime>).hash_code())
                                {
                                    const auto timetag = wht::any_cast<const std::shared_ptr<GnssTime>>(pmt::any_ref(it.value));
                                    // // std::cout << "PVT timetag: " << timetag->rx_time << '\n';
                                    d_TimeChannelTagTimestamps.push(*timetag);
                                }
                            else
                                {
                                    // std::cout << "hash code not match\n";
                                }
                        }
                    catch (const wht::bad_any_cast& e)
                        {
                            // std::cout << "msg Bad any_cast: " << e.what();
                        }
                }
        }
    // ************ end time tags **************
    // fileRx.close();
    for (int32_t epoch = 0; epoch < noutput_items; epoch++)
        {
            bool flag_display_pvt = false;
            bool flag_compute_pvt_output = false;
            // bool flag_write_RTCM_1019_output = false;
            // bool flag_write_RTCM_1020_output = false;
            // bool flag_write_RTCM_1045_output = false;
            // bool flag_write_RTCM_MSM_output = false;
            // bool flag_write_RINEX_obs_output = false;
            d_local_counter_ms += static_cast<uint64_t>(d_observable_interval_ms);
            d_gnss_observables_map.clear();
            const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);  // Get the input buffer pointer
            // ############ 1. READ PSEUDORANGES ####
            for (uint32_t i = 0; i < d_nchannels; i++)
                {
                    if (in[i][epoch].Flag_valid_pseudorange)
                        {
                            const auto tmp_eph_iter_gps = d_internal_pvt_solver->gps_ephemeris_map.find(in[i][epoch].PRN);
                            const auto tmp_eph_iter_gal = d_internal_pvt_solver->galileo_ephemeris_map.find(in[i][epoch].PRN);
                            const auto tmp_eph_iter_cnav = d_internal_pvt_solver->gps_cnav_ephemeris_map.find(in[i][epoch].PRN);
                            const auto tmp_eph_iter_glo_gnav = d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN);
                            const auto tmp_eph_iter_bds_dnav = d_internal_pvt_solver->beidou_dnav_ephemeris_map.find(in[i][epoch].PRN);

                            bool store_valid_observable = false;

                            if (tmp_eph_iter_gps != d_internal_pvt_solver->gps_ephemeris_map.cend())
                                {
                                    const uint32_t prn_aux = tmp_eph_iter_gps->second.PRN;
                                    if ((prn_aux == in[i][epoch].PRN) && (std::string(in[i][epoch].Signal, 2) == std::string("1C")) && (d_use_unhealthy_sats || (tmp_eph_iter_gps->second.SV_health == 0)))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_gal != d_internal_pvt_solver->galileo_ephemeris_map.cend())
                                {
                                    const uint32_t prn_aux = tmp_eph_iter_gal->second.PRN;
                                    if ((prn_aux == in[i][epoch].PRN) &&
                                        (((std::string(in[i][epoch].Signal, 2) == std::string("1B")) && (d_use_unhealthy_sats || ((tmp_eph_iter_gal->second.E1B_DVS == false) && (tmp_eph_iter_gal->second.E1B_HS == 0)))) ||
                                            ((std::string(in[i][epoch].Signal, 2) == std::string("5X")) && (d_use_unhealthy_sats || ((tmp_eph_iter_gal->second.E5a_DVS == false) && (tmp_eph_iter_gal->second.E5a_HS == 0)))) ||
                                            ((std::string(in[i][epoch].Signal, 2) == std::string("7X")) && (d_use_unhealthy_sats || ((tmp_eph_iter_gal->second.E5b_DVS == false) && (tmp_eph_iter_gal->second.E5b_HS == 0))))))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_cnav != d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                                {
                                    const uint32_t prn_aux = tmp_eph_iter_cnav->second.PRN;
                                    if ((prn_aux == in[i][epoch].PRN) && (((std::string(in[i][epoch].Signal, 2) == std::string("2S")) || (std::string(in[i][epoch].Signal, 2) == std::string("L5")))))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_glo_gnav != d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
                                {
                                    const uint32_t prn_aux = tmp_eph_iter_glo_gnav->second.PRN;
                                    if ((prn_aux == in[i][epoch].PRN) && ((std::string(in[i][epoch].Signal, 2) == std::string("1G")) || (std::string(in[i][epoch].Signal, 2) == std::string("2G"))))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (tmp_eph_iter_bds_dnav != d_internal_pvt_solver->beidou_dnav_ephemeris_map.cend())
                                {
                                    const uint32_t prn_aux = tmp_eph_iter_bds_dnav->second.PRN;
                                    if ((prn_aux == in[i][epoch].PRN) && (((std::string(in[i][epoch].Signal, 2) == std::string("B1")) || (std::string(in[i][epoch].Signal, 2) == std::string("B3"))) && (d_use_unhealthy_sats || (tmp_eph_iter_bds_dnav->second.SV_health == 0))))
                                        {
                                            store_valid_observable = true;
                                        }
                                }
                            if (std::string(in[i][epoch].Signal, 2) == std::string("E6"))
                                {
                                    store_valid_observable = true;
                                }

                            if (store_valid_observable)
                                {
                                    // store valid observables in a map.
                                    d_gnss_observables_map.insert(std::pair<int, Gnss_Synchro>(i, in[i][epoch]));
                                }

                            // if (d_rtcm_enabled)
                            //     {
                            //         try
                            //             {
                            //                 if (d_internal_pvt_solver->gps_ephemeris_map.empty() == false)
                            //                     {
                            //                         if (tmp_eph_iter_gps != d_internal_pvt_solver->gps_ephemeris_map.cend())
                            //                             {
                            //                                 d_rtcm_printer->lock_time(d_internal_pvt_solver->gps_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                            //                             }
                            //                     }
                            //                 if (d_internal_pvt_solver->galileo_ephemeris_map.empty() == false)
                            //                     {
                            //                         if (tmp_eph_iter_gal != d_internal_pvt_solver->galileo_ephemeris_map.cend())
                            //                             {
                            //                                 d_rtcm_printer->lock_time(d_internal_pvt_solver->galileo_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                            //                             }
                            //                     }
                            //                 if (d_internal_pvt_solver->gps_cnav_ephemeris_map.empty() == false)
                            //                     {
                            //                         if (tmp_eph_iter_cnav != d_internal_pvt_solver->gps_cnav_ephemeris_map.cend())
                            //                             {
                            //                                 d_rtcm_printer->lock_time(d_internal_pvt_solver->gps_cnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                            //                             }
                            //                     }
                            //                 if (d_internal_pvt_solver->glonass_gnav_ephemeris_map.empty() == false)
                            //                     {
                            //                         if (tmp_eph_iter_glo_gnav != d_internal_pvt_solver->glonass_gnav_ephemeris_map.cend())
                            //                             {
                            //                                 d_rtcm_printer->lock_time(d_internal_pvt_solver->glonass_gnav_ephemeris_map.find(in[i][epoch].PRN)->second, in[i][epoch].RX_time, in[i][epoch]);  // keep track of locking time
                            //                             }
                            //                     }
                            //             }
                            //         catch (const boost::exception& ex)
                            //             {
                            //                 // std::cout << "RTCM boost exception: " << boost::diagnostic_information(ex) << '\n';
                            //                 // LOG(ERROR) << "RTCM boost exception: " << boost::diagnostic_information(ex);
                            //             }
                            //         catch (const std::exception& ex)
                            //             {
                            //                 // std::cout << "RTCM std exception: " << ex.what() << '\n';
                            //                 // LOG(ERROR) << "RTCM std exception: " << ex.what();
                            //             }
                            //     }
                        }
                    else
                        {
                            d_channel_initialized.at(i) = false;  // the current channel is not reporting valid observable
                        }
                }

            // ############ 2. APPLY HAS CORRECTIONS IF AVAILABLE ####
            if (d_use_has_corrections && !d_gnss_observables_map.empty())
                {
                    this->update_HAS_corrections();
                }

            // ############ 2 COMPUTE THE PVT ################################
            bool flag_pvt_valid = false;
            flag_new_pvt_data = flag_pvt_valid;
            if (d_gnss_observables_map.empty() == false)
                {
                    //// LOG(INFO) << "diff raw obs time: " << d_gnss_observables_map.cbegin()->second.RX_time * 1000.0 - old_time_debug;
                    // old_time_debug = d_gnss_observables_map.cbegin()->second.RX_time * 1000.0;
                    // uint32_t current_RX_time_ms = 0;
                    // current_RX_time_ms = 0;
                    // #### solve PVT and store the corrected observable set
                    if (d_internal_pvt_solver->get_PVT(d_gnss_observables_map, false))
                        {
                            d_pvt_errors_counter = 0;  // Reset consecutive PVT error counter
                            const double Rx_clock_offset_s = d_internal_pvt_solver->get_time_offset_s();

                            // **************** time tags ****************
                            if (d_enable_rx_clock_correction == false)  // todo: currently only works if clock correction is disabled (computed clock offset is applied here)
                                {
                                    // ************ Source TimeTag comparison with GNSS computed TOW *************

                                    if (!d_TimeChannelTagTimestamps.empty())
                                        {
                                            double delta_rxtime_to_tag_ms;
                                            GnssTime current_tag;
                                            // 1. Find the nearest timetag to the current rx_time (it is relative to the receiver's start operation)
                                            do
                                                {
                                                    current_tag = d_TimeChannelTagTimestamps.front();
                                                    delta_rxtime_to_tag_ms = d_rx_time * 1000.0 - current_tag.rx_time;
                                                    d_TimeChannelTagTimestamps.pop();
                                                }
                                            while (fabs(delta_rxtime_to_tag_ms) >= 100 and !d_TimeChannelTagTimestamps.empty());

                                            // 2. If both timestamps (relative to the receiver's start) are closer than 100 ms (the granularituy of the PVT)
                                            if (fabs(delta_rxtime_to_tag_ms) <= 100)  // [ms]
                                                {
                                                    // std::cout << "GNSS-SDR RX TIME: " << d_rx_time << " TAG RX TIME: " << current_tag.rx_time / 1000.0 << " [s]\n";
                                                    if (d_log_timetag == true)
                                                        {
                                                            double current_corrected_RX_clock_ns = (d_rx_time - Rx_clock_offset_s) * 1e9;
                                                            double TAG_time_ns = (static_cast<double>(current_tag.tow_ms) + current_tag.tow_ms_fraction + delta_rxtime_to_tag_ms) * 1e6;
                                                            log_source_timetag_info(current_corrected_RX_clock_ns, TAG_time_ns);
                                                            double tow_error_ns = current_corrected_RX_clock_ns - TAG_time_ns;
                                                            // std::cout << "[Time ch] RX TimeTag Week: " << current_tag.week
                                                            //   << ", TOW: " << current_tag.tow_ms
                                                            //   << " [ms], TOW fraction: " << current_tag.tow_ms_fraction
                                                            //   << " [ms], GNSS-SDR OBS CORRECTED TOW - EXTERNAL TIMETAG TOW: " << tow_error_ns << " [ns] \n";
                                                        }
                                                }
                                        }
                                }
                            // **********************************************

                            if (fabs(Rx_clock_offset_s) * 1000.0 > d_max_obs_block_rx_clock_offset_ms)
                                {
                                    // check if the message was just sent to not duplicate it while it is being applied
                                    if ((d_local_counter_ms - d_timestamp_rx_clock_offset_correction_msg_ms) > 1000)
                                        {
                                            this->message_port_pub(pmt::mp("pvt_to_observables"), pmt::make_any(Rx_clock_offset_s));
                                            d_timestamp_rx_clock_offset_correction_msg_ms = d_local_counter_ms;
                                            // LOG(INFO) << "PVT: Sent clock offset correction to observables: " << Rx_clock_offset_s << "[s]";
                                        }
                                }
                            else
                                {
                                    if (d_enable_rx_clock_correction == true)
                                        {
                                            d_gnss_observables_map_t0 = d_gnss_observables_map_t1;
                                            apply_rx_clock_offset(d_gnss_observables_map, Rx_clock_offset_s);
                                            d_gnss_observables_map_t1 = d_gnss_observables_map;

                                            // ### select the rx_time and interpolate observables at that time
                                            if (!d_gnss_observables_map_t0.empty())
                                                {
                                                    const auto t0_int_ms = static_cast<uint32_t>(d_gnss_observables_map_t0.cbegin()->second.RX_time * 1000.0);
                                                    const uint32_t adjust_next_obs_interval_ms = d_observable_interval_ms - t0_int_ms % d_observable_interval_ms;
                                                    current_RX_time_ms = t0_int_ms + adjust_next_obs_interval_ms;

                                                    if (current_RX_time_ms % d_output_rate_ms == 0)
                                                        {
                                                            d_rx_time = static_cast<double>(current_RX_time_ms) / 1000.0;
                                                            // // std::cout << " obs time t0: " << d_gnss_observables_map_t0.cbegin()->second.RX_time
                                                            //           << " t1: " << d_gnss_observables_map_t1.cbegin()->second.RX_time
                                                            //           << " interp time: " << d_rx_time << '\n';
                                                            d_gnss_observables_map = interpolate_observables(d_gnss_observables_map_t0,
                                                                d_gnss_observables_map_t1,
                                                                d_rx_time);
                                                            flag_compute_pvt_output = true;
                                                            // d_rx_time = current_RX_time;
                                                            // // std::cout.precision(17);
                                                            // // std::cout << "current_RX_time: " << current_RX_time << " map time: " << d_gnss_observables_map.begin()->second.RX_time << '\n';
                                                        }
                                                }
                                        }
                                    else
                                        {
                                            d_rx_time = d_gnss_observables_map.begin()->second.RX_time;
                                            current_RX_time_ms = static_cast<uint32_t>(d_rx_time * 1000.0);
                                            if (current_RX_time_ms % d_output_rate_ms == 0)
                                                {
                                                    flag_compute_pvt_output = true;
                                                    // // std::cout.precision(17);
                                                    // // std::cout << "current_RX_time: " << current_RX_time_ms << " map time: " << d_gnss_observables_map.begin()->second.RX_time << '\n';
                                                }
                                            flag_pvt_valid = true;
                                            flag_new_pvt_data = flag_pvt_valid;
                                        }
                                }
                        }
                    else
                        {
                            // sanity check: If the PVT solver is getting 100 consecutive errors, send a reset command to observables block
                            d_pvt_errors_counter++;
                            if (d_pvt_errors_counter >= 100)
                                {
                                    int command = 1;
                                    this->message_port_pub(pmt::mp("pvt_to_observables"), pmt::make_any(command));
                                    // LOG(INFO) << "PVT: Number of consecutive position solver error reached, Sent reset to observables.";
                                    d_pvt_errors_counter = 0;
                                }
                        }

                    // compute on the fly PVT solution
                    if (flag_compute_pvt_output == true)
                        {
                            flag_pvt_valid = d_user_pvt_solver->get_PVT(d_gnss_observables_map, false);
                        }

                    if (flag_pvt_valid == true)
                        {
                            flag_new_pvt_data = true;
                            // experimental VTL tests
                            // send tracking command
                            //                            const std::shared_ptr<TrackingCmd> trk_cmd_test = std::make_shared<TrackingCmd>(TrackingCmd());
                            //                            trk_cmd_test->carrier_freq_hz = 12345.4;
                            //                            trk_cmd_test->sample_counter = d_gnss_observables_map.begin()->second.Tracking_sample_counter;
                            //                            this->message_port_pub(pmt::mp("pvt_to_trk"), pmt::make_any(trk_cmd_test));

                            // initialize (if needed) the accumulated phase offset and apply it to the active channels
                            // required to report accumulated phase cycles comparable to pseudoranges
                            initialize_and_apply_carrier_phase_offset();

                            const double Rx_clock_offset_s = d_user_pvt_solver->get_time_offset_s();
                            if (d_enable_rx_clock_correction == true and fabs(Rx_clock_offset_s) > 0.000001)  // 1us !!
                                {
                                    // LOG(INFO) << "Warning: Rx clock offset at interpolated RX time: " << Rx_clock_offset_s * 1000.0 << "[ms]"
                                    //   << " at RX time: " << static_cast<uint32_t>(d_rx_time * 1000.0) << " [ms]";
                                }
                            else
                                {
                                    // DLOG(INFO) << "Rx clock offset at interpolated RX time: " << Rx_clock_offset_s * 1000.0 << "[s]"
                                    //    << " at RX time: " << static_cast<uint32_t>(d_rx_time * 1000.0) << " [ms]";
                                    // Optional debug code: export observables snapshot for rtklib unit testing
                                    // // std::cout << "step 1: save gnss_synchro map\n";
                                    // save_gnss_synchro_map_xml("./gnss_synchro_map.xml");
                                    // getchar(); // stop the execution
                                    // end debug
                                    if (d_display_rate_ms != 0)
                                        {
                                            if (current_RX_time_ms % d_display_rate_ms == 0)
                                                {
                                                    flag_display_pvt = true;

                                                    // Caio Mod: Talvez inserir aqui o Acionamento e Ctrl de PWM
                                                }
                                        }
                                    // if (d_rtcm_MT1019_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                    //     {
                                    //         if (current_RX_time_ms % d_rtcm_MT1019_rate_ms == 0)
                                    //             {
                                    //                 flag_write_RTCM_1019_output = true;
                                    //             }
                                    //     }
                                    // if (d_rtcm_MT1020_rate_ms != 0)  // allows deactivating messages by setting rate = 0
                                    //     {
                                    //         if (current_RX_time_ms % d_rtcm_MT1020_rate_ms == 0)
                                    //             {
                                    //                 flag_write_RTCM_1020_output = true;
                                    //             }
                                    //     }
                                    // if (d_rtcm_MT1045_rate_ms != 0)
                                    //     {
                                    //         if (current_RX_time_ms % d_rtcm_MT1045_rate_ms == 0)
                                    //             {
                                    //                 flag_write_RTCM_1045_output = true;
                                    //             }
                                    //     }
                                    // TODO: RTCM 1077, 1087 and 1097 are not used, so, disable the output rates
                                    // if (current_RX_time_ms % d_rtcm_MT1077_rate_ms==0 && d_rtcm_MT1077_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1077_output_time = current_RX_time;
                                    //     }
                                    // if (current_RX_time_ms % d_rtcm_MT1087_rate_ms==0 && d_rtcm_MT1087_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1087_output_time = current_RX_time;
                                    //     }
                                    // if (current_RX_time_ms % d_rtcm_MT1097_rate_ms==0 && d_rtcm_MT1097_rate_ms != 0)
                                    //     {
                                    //         last_RTCM_1097_output_time = current_RX_time;
                                    //     }
                                    // if (d_rtcm_MSM_rate_ms != 0)
                                    //     {
                                    //         if (current_RX_time_ms % d_rtcm_MSM_rate_ms == 0)
                                    //             {
                                    //                 flag_write_RTCM_MSM_output = true;
                                    //             }
                                    //     }
                                    // if (d_rinexobs_rate_ms != 0)
                                    //     {
                                    //         if (current_RX_time_ms % static_cast<uint32_t>(d_rinexobs_rate_ms) == 0)
                                    //             {
                                    //                 flag_write_RINEX_obs_output = true;
                                    //             }
                                    //     }

                                    if (d_first_fix == true)
                                        {
                                            first_fix = true;
                                            // uint8_t bf[] = {0xDF, 0xDF};
                                            // int siso = 2;
                                            // int byyt;
                                            // byyt = HEserial_envio(&comms, &bf[0], &siso);

                                            if (d_show_local_time_zone)
                                                {
                                                    const boost::posix_time::ptime time_first_solution = d_user_pvt_solver->get_position_UTC_time() + d_utc_diff_time;
                                                    // std::cout << TEXT_BOLD_YELLOW << "First position fix at " << time_first_solution << d_local_time_str;
                                                }
                                            else
                                                {
                                                    // std::cout << TEXT_BOLD_YELLOW << "First position fix at " << d_user_pvt_solver->get_position_UTC_time() << " UTC";
                                                }
                                            // std::cout << " is Lat = " << d_user_pvt_solver->get_latitude() << " [deg], Long = " << d_user_pvt_solver->get_longitude()
                                            //           << " [deg], Height= " << d_user_pvt_solver->get_height() << " [m]\n"
                                            //           << TEXT_RESET;
                                            d_ttff_msgbuf ttff;
                                            ttff.mtype = 1;
                                            d_end = std::chrono::system_clock::now();
                                            std::chrono::duration<double> elapsed_seconds = d_end - d_start;
                                            ttff.ttff = elapsed_seconds.count();
                                            send_sys_v_ttff_msg(ttff);
                                            d_first_fix = false;
                                        }
                                }
                        }
                        d_internal_pvt_solver->gnss_observables_map = d_gnss_observables_map;
                        // num_sat = d_user_pvt_solver->get_num_valid_observations();
                        if (d_user_pvt_solver->is_valid_position()&&flag_pvt_valid)
                            {
                                jdex = 0;
                                rx_pos = d_user_pvt_solver->get_rx_pos();
                                // timepvt = d_user_pvt_solver->d_pvt_sol_time.time;
                                rx_vel = d_user_pvt_solver->get_rx_vel();
                                double rx_clk_deslize{0};
                                gps_ephem = d_internal_pvt_solver->gps_ephemeris_map;
                                sync_map = get_observables_map();
                                last_RX_time = sync_map.begin()->second.RX_time;
                                // if (!flag_interrupt_serial)
                                //     {
                                // std::map<int, Gps_CNAV_Ephemeris> gps_cnav = d_internal_pvt_solver->gps_cnav_ephemeris_map;
                                uint8_t checks{0};
                                msgVec[0] = 0xd4;
                                msgVec[1] = 0x4f;
                                index = 2;
                                // const double _conv = 0.017453292519943;          // (pi/180)
                                // // const double _c = 299792458.0;                   // velocidade da luz
                                // const double _OmegaDot = 7.292115146700000e-05;  // Taxa de velocidade de rotacao da Terra
                                // for(const auto &z:gps_cnav)
                                // {
                                //     std::cout<<TEXT_BOLD_RED<<"PRN: "<<z.second.PRN<<" ISCL1: "<<z.second.ISCL1<<TEXT_RESET<<"\n";
                                // }
                                for (const auto& y : sync_map)
                                    {
                                        for (const auto& x : gps_ephem)
                                            {
                                                if (y.second.System == 'G')
                                                    {
                                                        if (y.second.PRN == x.second.PRN)
                                                            {
                                                                if ((x.second.SV_health == 0))  // && (y.second.CN0_dB_hz<50.0))
                                                                    {
                                                                        // auto tStartSteady = std::chrono::high_resolution_clock::now();
                                                                        // auto StartTime = tStartSteady;
                                                                        // // std::cout<<x.second.PRN<<" "<<x.second.SV_health<<"\n";
                                                                        // // std::cout<<x.first<<"\n";
                                                                        // double variavel = x.second.tow - y.second.Pseudorange_m/SPEED_OF_LIGHT_M_S;
                                                                        // double variavel = (y.second.interp_TOW_ms / 1000.0) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;
                                                                        // double variavel = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S - x.second.af0;
                                                                        // double variavel = (y.second.RX_time) - (y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S);
                                                                        // double variavel = (y.second.TOW_at_current_symbol_ms) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S - x.second.af0;
                                                                        // y.second.TOW_at_current_symbol_ms    

                                                                        // // #########################   Geometrical Approuch on Transmit time:  #################################

                                                                        double tempoo = y.second.RX_time;  // this->current_RX_time_ms = y.second.TOW_at_current_symbol_ms;
                                                                        double diffSATREC, diffSATSAT = 1000;
                                                                        double delta_tempo;
                                                                        double nvotempo;
                                                                        double limiar = 0.2;
                                                                        // int iter = 0;
                                                                        std::vector<double> LastSatPos(3);
                                                                        // Polêmico
                                                                        double linha_visadas_ECEF[3];
                                                                        double pos_sat_rot[3];
                                                                        double PP[2][2]{0};
                                                                        //
                                                                        // gps_ephem.at(x.first).satellitePosition(y.second.RX_time);
                                                                        // while(limiar<diffSATSAT)
                                                                        gps_ephem.at(x.first).satellitePosition(tempoo);
                                                                        //  Step 1
                                                                        LastSatPos[0] = x.second.satpos_X;
                                                                        LastSatPos[1] = x.second.satpos_Y;
                                                                        LastSatPos[2] = x.second.satpos_Z;

                                                                        while (diffSATSAT > limiar)
                                                                            {
                                                                                //  Step 2
                                                                                diffSATREC = sqrt(
                                                                                    // (pow(d_internal_pvt_solver->get_rx_pos()[0] - LastSatPos[0], 2)) + (pow(d_internal_pvt_solver->get_rx_pos()[1] - LastSatPos[1], 2)) + (pow(d_internal_pvt_solver->get_rx_pos()[2] - LastSatPos[2], 2)));
                                                                                    pow(rx_pos[0] - LastSatPos[0], 2) + pow(rx_pos[1] - LastSatPos[1], 2) + pow(rx_pos[2] - LastSatPos[2], 2));
                                                                                delta_tempo = diffSATREC / SPEED_OF_LIGHT_M_S;
                                                                                // Step 3
                                                                                nvotempo = tempoo - delta_tempo;
                                                                                gps_ephem.at(x.first).satellitePosition(nvotempo);
                                                                                diffSATSAT = sqrt(
                                                                                    (pow(LastSatPos[0] - x.second.satpos_X, 2)) + (pow(LastSatPos[1] - x.second.satpos_Y, 2)) + (pow(LastSatPos[2] - x.second.satpos_Z, 2)));
                                                                                // iter++;
                                                                                // // std::cout << "Iter: " << iter <<"\n";
                                                                                LastSatPos[0] = x.second.satpos_X;
                                                                                LastSatPos[1] = x.second.satpos_Y;
                                                                                LastSatPos[2] = x.second.satpos_Z;
                                                                            }
                                                                        // Step 4
                                                                        variavelTempo = nvotempo - d_user_pvt_solver->get_time_offset_s();
                                                                        gps_ephem.at(x.first).satellitePosition(variavelTempo);


                                                                        PRN = (uint8_t)x.second.PRN;
                                                                        Carrier_Doppler_Hz = y.second.Carrier_Doppler_hz;
                                                                        satPosX = x.second.satpos_X;
                                                                        satPosY = x.second.satpos_Y;
                                                                        satPosZ = x.second.satpos_Z;
                                                                        satVelX = x.second.satvel_X;
                                                                        satVelY = x.second.satvel_Y;
                                                                        satVelZ = x.second.satvel_Z;
                                                                        // last_RX_time = y.second.TOW_at_current_symbol_ms * 0.001;
                                                                        // last_RX_time = y.second.RX_time;
                                                                        double m1, m2, m3;

                                                                        // DELTA PSEUDORANGE
                                                                        // Verificar a unid. do af1
                                                                        deltaprange = -SPEED_OF_LIGHT_M_S * (Carrier_Doppler_Hz / 1575420000) - ((d_user_pvt_solver->get_clock_drift_ppm()* 1e-6)-x.second.af1) * SPEED_OF_LIGHT_M_S ;

                                                                        // double prange = sqrt(
                                                                        //     (x.second.satpos_X - d_user_pvt_solver->get_rx_pos()[0]) * (x.second.satpos_X - d_user_pvt_solver->get_rx_pos()[0]) +
                                                                        //     (x.second.satpos_Y - d_user_pvt_solver->get_rx_pos()[1]) * (x.second.satpos_Y - d_user_pvt_solver->get_rx_pos()[1]) +
                                                                        //     (x.second.satpos_Z - d_user_pvt_solver->get_rx_pos()[2]) * (x.second.satpos_Z - d_user_pvt_solver->get_rx_pos()[2]));
                                                                        m1 = (satPosX - rx_pos[0]) * (satPosX - rx_pos[0]);
                                                                        m2 = (satPosY - rx_pos[1]) * (satPosY - rx_pos[1]);
                                                                        m3 = (satPosZ - rx_pos[2]) * (satPosZ - rx_pos[2]);
                                                                        double prange = sqrt((m1 + m2 + m3));
                                                                        
                                                                        // Polêmico
                                                                        double deltatempo = prange/SPEED_OF_LIGHT_M_S;
                                                                        PP[0][0] = cos(GNSS_OMEGA_EARTH_DOT*delta_tempo);
                                                                        PP[0][1] = sin(GNSS_OMEGA_EARTH_DOT*delta_tempo);
                                                                        PP[1][0] = -sin(GNSS_OMEGA_EARTH_DOT*delta_tempo);
                                                                        PP[1][1] = cos(GNSS_OMEGA_EARTH_DOT*delta_tempo);
                                                                        pos_sat_rot[0] = PP[0][0]*satPosX + PP[0][1]*satPosY;
                                                                        pos_sat_rot[1] = PP[1][0]*satPosX + PP[1][1]*satPosY;
                                                                        pos_sat_rot[2] = satPosZ;
                                                                        linha_visadas_ECEF[0] = pos_sat_rot[0]-rx_pos[0];
                                                                        linha_visadas_ECEF[1] = pos_sat_rot[1]-rx_pos[1];
                                                                        linha_visadas_ECEF[2] = pos_sat_rot[2]-rx_pos[2];
                                                                        prange = sqrt(linha_visadas_ECEF[0]*linha_visadas_ECEF[0]
                                                                        +linha_visadas_ECEF[1]*linha_visadas_ECEF[1]
                                                                        +linha_visadas_ECEF[2]*linha_visadas_ECEF[2]);
                                                                        //


                                                                        /*
                                                                        mtx.lock();
                                                                        // ######## Gravar na Struct #########
                                                                        StorageSat[jdex].PRN = PRN;
                                                                        StorageSat[jdex].prang = prange;
                                                                        StorageSat[jdex].deltaprange = deltaprange;
                                                                        StorageSat[jdex].satPosX = satPosX;
                                                                        StorageSat[jdex].satPosY = satPosY;
                                                                        StorageSat[jdex].satPosZ = satPosZ;
                                                                        StorageSat[jdex].satVelX = satVelX;
                                                                        StorageSat[jdex].satVelY = satVelY;
                                                                        StorageSat[jdex].satVelZ = satVelZ;
                                                                        StoragePVT.last_RX_time = last_RX_time*0.001; // Quando TOW_current_symbol_ms
                                                                        StoragePVT.gps_time_offset = y.second.RX_time; //Temporariamente coloquei o tempo do receptor.
                                                                        jdex++;
                                                                        // ###################################
                                                                        mtx.unlock();
                                                                         */

                                                                        // Teste de Conversão p/ hexadecimal
                                                                        // uint8_t vec[8];
                                                                        // double ExVec;
                                                                        // Double2Hex(&vec[0],&temp);
                                                                        // Hex2Double(&ExVec,&vec[0]);
                                                                        // Double2Hex(&test[0] , &temp);
                                                                        //
                                                                        mtx.lock();
                                                                        msgVec[index + 0] = PRN;
                                                                        Double2HexAlt(&msgVec[index + 1], &prange);
                                                                        Double2HexAlt(&msgVec[index + 9], &deltaprange);
                                                                        Double2HexAlt(&msgVec[index + 17], &satPosX);
                                                                        Double2HexAlt(&msgVec[index + 25], &satPosY);
                                                                        Double2HexAlt(&msgVec[index + 33], &satPosZ);
                                                                        Double2HexAlt(&msgVec[index + 41], &satVelX);
                                                                        Double2HexAlt(&msgVec[index + 49], &satVelY);
                                                                        Double2HexAlt(&msgVec[index + 57], &satVelZ);
                                                                        index = index + 65;
                                                                        mtx.unlock();
                                                                        // ###############################
                                                                        // auto tEndSteady = std::chrono::high_resolution_clock::now();
                                                                        // std::chrono::duration<double, std::milli> tempo_ligado_ms = tEndSteady - tStartSteady;
                                                                        // std::cout << tempo_ligado_ms.count() << "\n";
                                                                    }
                                                            }
                                                    }
                                                else
                                                    {
                                                        if (y.second.PRN == x.second.PRN)
                                                            {
                                                                if ((x.second.SV_health == 0))  // && (y.second.CN0_dB_hz<50.0))
                                                                    {
                                                                        // auto tStartSteady = std::chrono::high_resolution_clock::now();
                                                                        // auto StartTime = tStartSteady;
                                                                        // // std::cout<<x.second.PRN<<" "<<x.second.SV_health<<"\n";
                                                                        // // std::cout<<x.first<<"\n";
                                                                        // double variavel = x.second.tow - y.second.Pseudorange_m/SPEED_OF_LIGHT_M_S;
                                                                        // double variavel = (y.second.interp_TOW_ms / 1000.0) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S;
                                                                        // double variavel = (y.second.RX_time) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S - x.second.af0;
                                                                        // double variavel = (y.second.RX_time) - (y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S);
                                                                        // double variavel = (y.second.TOW_at_current_symbol_ms) - y.second.Pseudorange_m / SPEED_OF_LIGHT_M_S - x.second.af0;
                                                                        // y.second.TOW_at_current_symbol_ms

                                                                        // // #########################   Geometrical Approuch on Transmit time:  #################################

                                                                        double tempoo = y.second.RX_time;  // this->current_RX_time_ms = y.second.TOW_at_current_symbol_ms;
                                                                        double diffSATREC, diffSATSAT = 1000;
                                                                        double delta_tempo;
                                                                        double nvotempo;
                                                                        double limiar = 0.2;
                                                                        // int iter = 0;
                                                                        std::vector<double> LastSatPos(3);
                                                                        // gps_ephem.at(x.first).satellitePosition(y.second.RX_time);
                                                                        // while(limiar<diffSATSAT)
                                                                        gps_ephem.at(x.first).satellitePosition(tempoo);
                                                                        //  Step 1
                                                                        LastSatPos[0] = x.second.satpos_X;
                                                                        LastSatPos[1] = x.second.satpos_Y;
                                                                        LastSatPos[2] = x.second.satpos_Z;

                                                                        while (diffSATSAT > limiar)
                                                                            {
                                                                                //  Step 2
                                                                                diffSATREC = sqrt(
                                                                                    // (pow(d_internal_pvt_solver->get_rx_pos()[0] - LastSatPos[0], 2)) + (pow(d_internal_pvt_solver->get_rx_pos()[1] - LastSatPos[1], 2)) + (pow(d_internal_pvt_solver->get_rx_pos()[2] - LastSatPos[2], 2)));
                                                                                    pow(rx_pos[0] - LastSatPos[0], 2) + pow(rx_pos[1] - LastSatPos[1], 2) + pow(rx_pos[2] - LastSatPos[2], 2));
                                                                                delta_tempo = diffSATREC / SPEED_OF_LIGHT_M_S;
                                                                                // Step 3
                                                                                nvotempo = tempoo - delta_tempo;
                                                                                gps_ephem.at(x.first).satellitePosition(nvotempo);
                                                                                diffSATSAT = sqrt(
                                                                                    (pow(LastSatPos[0] - x.second.satpos_X, 2)) + (pow(LastSatPos[1] - x.second.satpos_Y, 2)) + (pow(LastSatPos[2] - x.second.satpos_Z, 2)));
                                                                                // iter++;
                                                                                // // std::cout << "Iter: " << iter <<"\n";
                                                                                LastSatPos[0] = x.second.satpos_X;
                                                                                LastSatPos[1] = x.second.satpos_Y;
                                                                                LastSatPos[2] = x.second.satpos_Z;
                                                                            }
                                                                        // Step 4
                                                                        variavelTempo = nvotempo - d_user_pvt_solver->get_time_offset_s();
                                                                        gps_ephem.at(x.first).satellitePosition(variavelTempo);


                                                                        PRN = (uint8_t)x.second.PRN;
                                                                        Carrier_Doppler_Hz = y.second.Carrier_Doppler_hz;
                                                                        satPosX = x.second.satpos_X;
                                                                        satPosY = x.second.satpos_Y;
                                                                        satPosZ = x.second.satpos_Z;
                                                                        satVelX = x.second.satvel_X;
                                                                        satVelY = x.second.satvel_Y;
                                                                        satVelZ = x.second.satvel_Z;
                                                                        
                                                                        // last_RX_time = y.second.RX_time;
                                                                        double m1, m2, m3;

                                                                        deltaprange = -SPEED_OF_LIGHT_M_S * (Carrier_Doppler_Hz / 1575420000) - ((d_user_pvt_solver->get_clock_drift_ppm()* 1e-6)-x.second.af1) * SPEED_OF_LIGHT_M_S ;

                                                                        // double prange = sqrt(
                                                                        //     (x.second.satpos_X - d_user_pvt_solver->get_rx_pos()[0]) * (x.second.satpos_X - d_user_pvt_solver->get_rx_pos()[0]) +
                                                                        //     (x.second.satpos_Y - d_user_pvt_solver->get_rx_pos()[1]) * (x.second.satpos_Y - d_user_pvt_solver->get_rx_pos()[1]) +
                                                                        //     (x.second.satpos_Z - d_user_pvt_solver->get_rx_pos()[2]) * (x.second.satpos_Z - d_user_pvt_solver->get_rx_pos()[2]));
                                                                        m1 = (satPosX - rx_pos[0]) * (satPosX - rx_pos[0]);
                                                                        m2 = (satPosY - rx_pos[1]) * (satPosY - rx_pos[1]);
                                                                        m3 = (satPosZ - rx_pos[2]) * (satPosZ - rx_pos[2]);
                                                                        double prange = sqrt((m1 + m2 + m3));

                                                                        // Polêmico
                                                                        double linha_visadas_ECEF[3];
                                                                        double pos_sat_rot[3];
                                                                        double PP[2][2]{0};
                                                                        double deltatempo = prange / SPEED_OF_LIGHT_M_S;
                                                                        PP[0][0] = cos(GNSS_OMEGA_EARTH_DOT * delta_tempo);
                                                                        PP[0][1] = sin(GNSS_OMEGA_EARTH_DOT * delta_tempo);
                                                                        PP[1][0] = -sin(GNSS_OMEGA_EARTH_DOT * delta_tempo);
                                                                        PP[1][1] = cos(GNSS_OMEGA_EARTH_DOT * delta_tempo);
                                                                        pos_sat_rot[0] = PP[0][0] * satPosX + PP[0][1] * satPosY;
                                                                        pos_sat_rot[1] = PP[1][0] * satPosX + PP[1][1] * satPosY;
                                                                        pos_sat_rot[2] = satPosZ;
                                                                        linha_visadas_ECEF[0] = pos_sat_rot[0] - rx_pos[0];
                                                                        linha_visadas_ECEF[1] = pos_sat_rot[1] - rx_pos[1];
                                                                        linha_visadas_ECEF[2] = pos_sat_rot[2] - rx_pos[2];
                                                                        prange = sqrt(linha_visadas_ECEF[0] * linha_visadas_ECEF[0]
                                                                         + linha_visadas_ECEF[1] * linha_visadas_ECEF[1] 
                                                                         + linha_visadas_ECEF[2] * linha_visadas_ECEF[2]);
                                                                        //

                                                                        /*
                                                                        mtx.lock();
                                                                        // ######## Gravar na Struct #########
                                                                        StorageSat[jdex].PRN = PRN;
                                                                        StorageSat[jdex].prang = prange;
                                                                        StorageSat[jdex].deltaprange = deltaprange;
                                                                        StorageSat[jdex].satPosX = satPosX;
                                                                        StorageSat[jdex].satPosY = satPosY;
                                                                        StorageSat[jdex].satPosZ = satPosZ;
                                                                        StorageSat[jdex].satVelX = satVelX;
                                                                        StorageSat[jdex].satVelY = satVelY;
                                                                        StorageSat[jdex].satVelZ = satVelZ;
                                                                        StoragePVT.last_RX_time = last_RX_time*0.001; // Quando TOW_current_symbol_ms
                                                                        StoragePVT.gps_time_offset = y.second.RX_time; //Temporariamente coloquei o tempo do receptor.
                                                                        jdex++;
                                                                        // ###################################
                                                                        mtx.unlock();
                                                                         */

                                                                        // Teste de Conversão p/ hexadecimal
                                                                        // uint8_t vec[8];
                                                                        // double ExVec;
                                                                        // Double2Hex(&vec[0],&temp);
                                                                        // Hex2Double(&ExVec,&vec[0]);
                                                                        // Double2Hex(&test[0] , &temp);
                                                                        //
                                                                        mtx.lock();
                                                                        msgVec[index + 0] = PRN + 32;
                                                                        Double2HexAlt(&msgVec[index + 1], &prange);
                                                                        Double2HexAlt(&msgVec[index + 9], &deltaprange);
                                                                        Double2HexAlt(&msgVec[index + 17], &satPosX);
                                                                        Double2HexAlt(&msgVec[index + 25], &satPosY);
                                                                        Double2HexAlt(&msgVec[index + 33], &satPosZ);
                                                                        Double2HexAlt(&msgVec[index + 41], &satVelX);
                                                                        Double2HexAlt(&msgVec[index + 49], &satVelY);
                                                                        Double2HexAlt(&msgVec[index + 57], &satVelZ);
                                                                        index = index + 65;
                                                                        mtx.unlock();
                                                                        // ###############################
                                                                        // auto tEndSteady = std::chrono::high_resolution_clock::now();
                                                                        // std::chrono::duration<double, std::milli> tempo_ligado_ms = tEndSteady - tStartSteady;
                                                                        // std::cout << tempo_ligado_ms.count() << "\n";
                                                                    }
                                                            }
                                                    }
                                            }
                                    }
                                mtx.lock();
                                for (int i = 0; i < index; i++)
                                    {
                                        // msgVec[index + 1] = msgVec[index + 1] ^ msgVec[i];  // CRC
                                        checks ^= msgVec[i];
                                        // if (msgVec[i])
                                        //     {
                                        //     }
                                    }
                                msgVec[index] = checks;

                                index++;
                                // int sendedEphem = write(comms.fd, &msgVec[0], tam + 3);
                                // int sended = serial4send(&msgVec[0], &tam);
                                // double auxxx = (uint32_t)d_rx_time;
                                // last_RX_time = round(last_RX_time);
                                msgVec[index + 0] = 0xd4;
                                msgVec[index + 1] = 0x4f;
                                Double2HexAlt(&msgVec[index + 2], &rx_pos[0]);
                                Double2HexAlt(&msgVec[index + 10], &rx_pos[1]);
                                Double2HexAlt(&msgVec[index + 18], &rx_pos[2]);
                                Double2HexAlt(&msgVec[index + 26], &rx_vel[0]);
                                Double2HexAlt(&msgVec[index + 34], &rx_vel[1]);
                                Double2HexAlt(&msgVec[index + 42], &rx_vel[2]);
                                Double2HexAlt(&msgVec[index + 50], &last_RX_time);
                                // Double2HexAlt(&msgVec[index+50], &auxxx);
                                checks = 0;
                                for (int i = index + 0; i < index + 58; i++)
                                    {
                                        checks ^= msgVec[i];
                                    }
                                msgVec[index + 58] = checks;
                                jdex = index + 58 + 1;
                                num_sat = d_user_pvt_solver->get_num_valid_observations();
                                mtx.unlock();

                                /*
                                mtx.lock();
                                StoragePVT.rx_pos[0] = rx_pos[0];
                                StoragePVT.rx_pos[1] = rx_pos[1];
                                StoragePVT.rx_pos[2] = rx_pos[2];
                                StoragePVT.rx_vel[0] = rx_vel[0];
                                StoragePVT.rx_vel[1] = rx_vel[1];
                                StoragePVT.rx_vel[2] = rx_vel[2];
                                // StoragePVT.last_RX_time = last_RX_time;//*0.001;
                                mtx.unlock();
                                */

                                msgReady = true;
                                // gtime_t gg = d_user_pvt_solver->d_pvt_sol_time; gg.time-=10800;
                                // std::cout<<TEXT_BOLD_CYAN<<"Local Time : "<<std::put_time(std::gmtime(&gg.time), "%c %Z")<<" GPST: "<<gg.time<<TEXT_RESET<<"\n";
                            }
                        // else
                        //     {
                        //         if (msgVec[0] != 0)
                        //             {
                        //                 msgReady = false;
                        //                 mtx.lock();
                        //                 for (int i = 0; i < (tam + 6 + 56); i += 10)
                        //                     {
                        //                         // msgVec[i]=0;
                        //                         msgVec[i + 0] = 0;
                        //                         msgVec[i + 1] = 0;
                        //                         msgVec[i + 2] = 0;
                        //                         msgVec[i + 3] = 0;
                        //                         msgVec[i + 4] = 0;
                        //                         msgVec[i + 5] = 0;
                        //                         msgVec[i + 6] = 0;
                        //                         msgVec[i + 7] = 0;
                        //                         msgVec[i + 8] = 0;
                        //                         msgVec[i + 9] = 0;
                        //                     }
                        //                 mtx.unlock();
                        //             }
                        //     }

                    // DEBUG MESSAGE: Display position in console output
                    // if (d_user_pvt_solver->is_valid_position() && flag_display_pvt)
                    //     {
                    //         boost::posix_time::ptime time_solution;
                    //         std::string UTC_solution_str;
                    //         if (d_show_local_time_zone)
                    //             {
                    //                 time_solution = d_user_pvt_solver->get_position_UTC_time() + d_utc_diff_time;
                    //                 UTC_solution_str = d_local_time_str;
                    //             }
                    //         else
                    //             {
                    //                 time_solution = d_user_pvt_solver->get_position_UTC_time();
                    //                 UTC_solution_str = " UTC";
                    //             }
                            // std::streamsize ss = // std::cout.precision();  // save current precision
                            // std::cout.setf(std::ios::fixed, std::ios::floatfield);
                            auto* facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
                            // std::cout.imbue(std::locale(// std::cout.getloc(), facet));


                            // // PVT MONITOR
                            // if (d_user_pvt_solver->is_valid_position())
                            //     {
                            //         const std::shared_ptr<Monitor_Pvt> monitor_pvt = std::make_shared<Monitor_Pvt>(d_user_pvt_solver->get_monitor_pvt());

                            //         // publish new position to the gnss_flowgraph channel status monitor
                            //         if (current_RX_time_ms % d_report_rate_ms == 0)
                            //             {
                            //                 this->message_port_pub(pmt::mp("status"), pmt::make_any(monitor_pvt));
                            //             }
                            //         if (d_flag_monitor_pvt_enabled)
                            //             {
                            //                 d_udp_sink_ptr->write_monitor_pvt(monitor_pvt.get());
                            //             }
                            //     }
                            // }

                            flag_new_pvt_data = flag_pvt_valid;
                }

            return noutput_items;
        }
}

bool rtklib_pvt_gs::set_flag_serial_interp(bool status)
{
    flag_interrupt_serial = status;
}



void rtklib_pvt_gs::serialcmd_(void)
{
    // bool flag_fix_first = false;
    // std::ofstream filele("ArqTest.bin",std::ios::out|std::ios::app|std::ios::binary);
    auto tStartSteady = std::chrono::high_resolution_clock::now();
    auto tEndSteady = std::chrono::high_resolution_clock::now();
    mtx.lock();
    double ult_tempo = sync_map.begin()->second.TOW_at_current_symbol_ms * 0.001;  //*1000;
    mtx.unlock();
    // double nvo_tempo;
    double nvoo_tempo;
    double tttt;
    int ccontmsg = 0;
    double deltinha = 0.0f;
    while (!flag_interrupt_serial)
        {
            tEndSteady = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> tempo_ligado_ms = tEndSteady - tStartSteady;

            tttt = tempo_ligado_ms.count()-deltinha;
            mtx.lock();
            nvoo_tempo = sync_map.begin()->second.TOW_at_current_symbol_ms * 0.001;
            mtx.unlock();
            // nvo_tempo = d_rx_time;

            // std::map<int, Gps_CNAV_Ephemeris> gps_cnav = d_internal_pvt_solver->gps_cnav_ephemeris_map;
            if ((first_fix) && (tttt >= 1000) && (round(nvoo_tempo - ult_tempo) >= 1.0))
            // if ((first_fix) && (((current_RX_time_ms % 1000) == 0) && ((nvo_tempo - ult_tempo) >= 1.0)))//&& (tttt>1000))
                {
                    int sended_PVT = write(comms.fd, &msgVec[0], jdex);
                    ccontmsg++;
                    deltinha = tttt - 1000.0;
                    tStartSteady = std::chrono::high_resolution_clock::now();
                    ult_tempo = nvoo_tempo;
                    std::cout << TEXT_BOLD_GREEN <<" N_Sat: " << num_sat << " Bytes: " << sended_PVT <<" Contador: "<<ccontmsg<<" Time: " << tttt 
                    << " last_Rx_time: " << nvoo_tempo <<" d_rx_time: "<<d_rx_time<< TEXT_RESET << "\n";
                    // for(const auto &x:gps_cnav)
                    // {
                    //     std::cout<<"PRN: "<<x.second.PRN<<" ISCL1: "<<x.second.ISCL1<<"\n";
                    // }
                }
            else if (tttt>=1100)
                {
                    tStartSteady = std::chrono::system_clock::now();
                }
                else{continue;}
        }

}
