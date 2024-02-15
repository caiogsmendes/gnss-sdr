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

#include "HEtechSerial.h"

class PvtInterface;

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
       void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);
       void run_serial_listener(char*);
       void CmdParser(char* cmd);
       void serial_get_pvt(void);
       void serial_status(void);
       void serial_reset(void);

       std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue_;

   private:
       void register_functions();

       std::unordered_map<std::string, std::function<std::string(const std::vector<std::string> &)>>
           functions_;
    //    std::string serial_get_pvt(const std::vector<std::string> &commandLine);
       // std::string serial_status(void);
       std::shared_ptr<PvtInterface> PVT_sptr_;

       bool keep_running_;
};

#endif