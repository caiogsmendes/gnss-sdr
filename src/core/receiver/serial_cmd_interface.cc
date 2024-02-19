
#include "serial_cmd_interface.h"

#include "command_event.h"
#include "pvt_interface.h"

#include "HEtechSerial.h"

SerialCmdInterface::SerialCmdInterface() : keep_running_(true)
{
    register_functions();
    control_queue_ = nullptr;
}

void SerialCmdInterface::set_pvt(std::shared_ptr<PvtInterface> PVT_sptr)
{
    // std::move() indica a passagem de acesso aos recursos de um ponteiro
    // para outro ponteiro.
    PVT_sptr_ = std::move(PVT_sptr);
}

void SerialCmdInterface::run_serial_listener(char* input)
{
    while (keep_running_ == true)
        {
            serial4readByte(input);  // Vai esperar um cmd pela porta serial.
            // CmdParser(input); // Melhor remover essa função e apply action lá na control_thread
            if (control_queue_ != nullptr)
            {
                const command_event_sptr new_evnt = command_event_make(300,(int)(*input)); 
                control_queue_->push(pmt::make_any(new_evnt));
                // flowgraph->apply_action((int*)input);
            }
        }
}

// std::string TcpCmdInterface::reset(const std::vector<std::string> &commandLine __attribute__((unused)))
// {
//     std::string response;
//     if (control_queue_ != nullptr)
//         {
//             const command_event_sptr new_evnt = command_event_make(200, 1);  // send the restart message (who=200,what=1)
//             control_queue_->push(pmt::make_any(new_evnt));
//             response = "OK\n";
//         }
//     else
//         {
//             response = "ERROR\n";
//         }
//     return response;
// }


void SerialCmdInterface::register_functions(void)
{
    // functions_["serial_get_pvt"] = [&](auto &s) { return SerialCmdInterface::serial_get_pvt(s); };
}

void SerialCmdInterface::CmdParser(char* cmd)
{
    switch ((int)(*cmd)) //Gambiarra, tem que ajeitar isso aqui.
    {
        case 0x01:
            serial_get_pvt();
            break;
        case 0x02:
            serial_reset();
            break;
        default:
            char buff = 0xFF;
            serial4send(&buff);
    }
}

void SerialCmdInterface::serial_get_pvt(void)
{
    double longitude_deg;
    double latitude_deg;
    double height_m;
    double ground_speed_kmh;
    double course_over_ground_deg;
    time_t UTC_time;
    if (PVT_sptr_->get_latest_PVT(&longitude_deg,
            &latitude_deg,
            &height_m,
            &ground_speed_kmh,
            &course_over_ground_deg,
            &UTC_time) == true)
        {
            char buff[200];
            memset(buff,'\0',sizeof(buff));
            sprintf(buff, "1=%lf|2=%lf|3=%lf|4=%lf|5=%lf|6=%lf", latitude_deg, longitude_deg, height_m, ground_speed_kmh, course_over_ground_deg, UTC_time);
            serial4send(&buff[0]);
        }
}

void SerialCmdInterface::serial_reset(void)

{
    const command_event_sptr new_evnt = command_event_make(200, 1);  // send the restart message (who=200,what=1)
    control_queue_->push(pmt::make_any(new_evnt));
    std::cout << "OK. Receptor GNSS reiniciando ...\n";
}


void SerialCmdInterface::serial_status(void)
{
    char buff[100];
    memset(buff,'\0',sizeof(buff));
    sprintf(buff, "Tá Rodando ...", NULL) ;
    serial4send(&buff[0]);
}

void SerialCmdInterface::set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue)
{
    control_queue_ = std::move(control_queue);
}