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

class PvtInterface;
class AcquisitionInterface;
class TrackingInterface;
class Rtklib_Solver;
class rtklib_pvt_gs;
class Gnss_Ephemeris;
class Gnss_Synchro;

class GNSSFlowgraph;

class SerialCmdInterface
{
    public:
        SerialCmdInterface();
        ~SerialCmdInterface() = default;

        /**
         * void run_cmd_device(std::string dev);
         * void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);
        */

       void set_pvt(std::shared_ptr<PvtInterface> PVT_sptr);
       void set_Acq(std::shared_ptr<AcquisitionInterface> Acq_sptr);
       void set_Trk(std::shared_ptr<TrackingInterface> Trk_sptr);
       void set_Sync(std::shared_ptr<Gnss_Synchro> Sync_sptr_);
       void set_GPS_Ephemeris(std::shared_ptr<Gnss_Ephemeris> GPS_ephem_sptr);
       void set_RTK_solver(std::shared_ptr<Rtklib_Solver> RtkSolver_sptr);
       
    //    void get_SerialCmd(std::shared_ptr<GNSSFlowgraph>);

       void serial_get_ephemeris(void);
       void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);
       void run_serial_listener(char*, char*, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr);
      //  void run_serial_listener(char*, char*);
       void DersoProtocol(void);
       void CmdParser(char* cmd);
       std::string serial_get_pvt(void);
       void serial_status(void);
       void serial_reset(void);
    //    void applicar(int);

       std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue_;
    //    std::shared_ptr<GNSSFlowgraph> flowgraph_;

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


       bool keep_running_;
};

#endif