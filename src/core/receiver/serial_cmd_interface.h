/**
 * Função feita em semelhança a class tcpcmdinterface
 * Tem a função de receber e enviar comandos via serial.
*/



#ifndef _SERIAL_CMD_INTERFACE_H_
#define _SERIAL_CMD_INTERFACE_H_

#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <array>
#include <cstdint>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gnss_flowgraph.h"

#include "HEtechSerial.h"

#include "channel.h"
#include "channel_interface.h"


class PvtInterface;
class AcquisitionInterface;
class TrackingInterface;
class Rtklib_Solver;
class rtklib_pvt_gs;
class Gnss_Ephemeris;
class Gnss_Synchro;
class ChannelInterface;
class Channel;
class GNSSFlowgraph;

class SerialCmdInterface
{
public:
    SerialCmdInterface();
    ~SerialCmdInterface() = default;

    typedef struct
    {
        // Satellite and Signals Info
        char Signal[3]{};
        uint32_t PRN{};
        int32_t Channel_ID{};
        // Acquisition
        double Acq_delay_samples{};
        double Acq_doppler_hz{};
        uint64_t Acq_samplestamp_samples{};
        uint32_t Acq_doppler_step{};
        // Tracking
        int64_t fs{};
        double Prompt_I{};
        double Prompt_Q{};
        double CN0_db_hz{};
        double Carrier_Doppler_hz{};
        double Carrier_phase_rads{};
        double Code_phase_samples{};
        uint64_t Tracking_sample_counter{};
        int32_t correlation_length_ms{};
        // Telemetry Decoder
        uint32_t TOW_at_current_symbol_ms{};
        // Observables
        double Pseudorange_m{};
        double RX_time{};
        double interp_TOW_ms{};
        // Flags
        bool Flag_valid_acquisition{};
        bool Flag_valid_symbol_output{};
        bool Flag_valid_word{};
        bool Flag_valid_pseudorange{};
        bool Flag_PLL_180_deg_phase_locked{};
    } internals;


    /**
     * void run_cmd_device(std::string dev);
     * void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);
     */

    void set_pvt(std::shared_ptr<PvtInterface> PVT_sptr);
    void set_Acq(std::shared_ptr<AcquisitionInterface> Acq_sptr);
    void set_Trk(std::shared_ptr<TrackingInterface> Trk_sptr);
    void set_Sync(std::shared_ptr<Gnss_Synchro> Sync_sptr_);
    void set_GPS_Ephemeris(std::shared_ptr<Gnss_Ephemeris> GPS_ephem_sptr);
    void set_GNSS_Observ(std::shared_ptr<std::vector<int, Gnss_Synchro>> gnss_observables_sptr);
    void set_RTK_solver(std::shared_ptr<Rtklib_Solver> RtkSolver_sptr);
    void set_channels(std::shared_ptr<std::vector<std::shared_ptr<ChannelInterface>>> channels_sptr);
    void set_serial_ptr_init(std::shared_ptr<SerialCmdInterface> serial_ptr_init);
    // void get_SerialCmd(std::shared_ptr<GNSSFlowgraph>);
    // void set_gnss_synchro_monitor(std::vector<std::shared_ptr<gnss_synchro_monitor>> monitor_ptr);

    void serial_get_GPS_ephemeris(std::shared_ptr<Rtklib_Solver> Rtklib_solver_sptr_);
    void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);
    void run_serial_listener(char *, char *, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr);
    //  void run_serial_listener(char*, char*);
    int DersoProtocol(void);
    void CospeEphemeris(void);
    void CmdParser(char *cmd);
    void serial_get_pvt(void);
    void serial_status(void);
    void serial_reset(void);
    void serial_get_synchro(void);
    //    void applicar(int);
    // void serial_cmd_init(void);

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue_;
    //    std::shared_ptr<GNSSFlowgraph> flowgraph_;
    std::shared_ptr<std::vector<std::shared_ptr<ChannelInterface>>> channels_sptr_;
    std::vector<Gnss_Synchro> Update_Synchro_Internals(const std::vector<Gnss_Synchro> &stocks, int n_channels);


    std::vector<internals> satelites[16];  // Máximo de 16 satélites
    std::vector<Gnss_Synchro> sats;
    int sat_iter = 0;
    int num_channel = 0;
    // std::vector<Gnss_Synchro> sats = std::make_shared<std::vector<Gnss_Synchro>>(sate);

private:
    void register_functions();

    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string> &)>>
        functions_;

    std::string serial_status(const std::vector<std::string> &commandLine);
    std::string reset(const std::vector<std::string> &commandLine);
    std::string standby(const std::vector<std::string> &commandLine);
    std::string hotstart(const std::vector<std::string> &commandLine);
    std::string warmstart(const std::vector<std::string> &commandLine);
    std::string coldstart(const std::vector<std::string> &commandLine);
    std::string set_ch_satellite(const std::vector<std::string> &commandLine);


    // std::string serial_get_pvt(const std::vector<std::string> &commandLine);
    // std::string serial_status(void);
    std::shared_ptr<PvtInterface> PVT_sptr_;
    std::shared_ptr<AcquisitionInterface> Acq_sptr_;
    std::shared_ptr<TrackingInterface> Trk_sptr_;
    std::shared_ptr<Gnss_Synchro> Sync_sptr_;
    std::shared_ptr<Gnss_Ephemeris> GPS_ephem_sptr_;
    std::shared_ptr<Rtklib_Solver> Rtklib_solver_sptr_;
    std::shared_ptr<std::vector<int, Gnss_Synchro>> gnss_observables_sptr_;
    //    std::shared_ptr<std::vector<std::shared_ptr<ChannelInterface>>> channels_sptr_;
    std::shared_ptr<SerialCmdInterface> serial_cmd_intern_ptr_;
    // std::shared_ptr<std::vector<std::shared_ptr<gnss_synchro_monitor>>> gnss_synchro_monitor_ptr_;

    channel_status_msg_receiver_sptr channels_status_;

    bool keep_running_;
};

#endif