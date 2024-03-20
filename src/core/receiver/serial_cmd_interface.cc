
#include "serial_cmd_interface.h"

#include "command_event.h"
#include "pvt_interface.h"
#include "HEtechSerial.h"
#include "rtklib_solver.h"



/**
 * Com o PVT_sptr_ 'e possivel acessar o mapping das ephemeris, 
 * Verificar tambem como puxar para essa classe, o acesso aos recursos do ponteiro d_user_pvt_solver do rtklib_pvt_gs.cc
 * Puxar para ca os ponteiros referentes ao GNSS Synchro
*/

SerialCmdInterface::SerialCmdInterface() : keep_running_(true)
{
    register_functions();
    control_queue_ = nullptr;
}

// SerialCmdInterface::SerialCmdInterface(){};

void SerialCmdInterface::set_pvt(std::shared_ptr<PvtInterface> PVT_sptr)
{
    // std::move() indica a passagem de acesso aos recursos de um ponteiro
    // para outro ponteiro.
    PVT_sptr_ = std::move(PVT_sptr);
}
void SerialCmdInterface::set_Acq(std::shared_ptr<AcquisitionInterface> Acq_sptr)
{
    // std::move() indica a passagem de acesso aos recursos de um ponteiro
    // para outro ponteiro.
    Acq_sptr_ = std::move(Acq_sptr);
}
void SerialCmdInterface::set_Trk(std::shared_ptr<TrackingInterface> Trk_sptr)
{
    // std::move() indica a passagem de acesso aos recursos de um ponteiro
    // para outro ponteiro.
    Trk_sptr_ = std::move(Trk_sptr);
}
void SerialCmdInterface::set_Sync(std::shared_ptr<Gnss_Synchro> Sync_sptr)
{
    // std::move() indica a passagem de acesso aos recursos de um ponteiro
    // para outro ponteiro.
    Sync_sptr_ = std::move(Sync_sptr);
}
void SerialCmdInterface::set_GPS_Ephemeris(std::shared_ptr<Gnss_Ephemeris> GPS_ephem_sptr)
{
    GPS_ephem_sptr_=std::move(GPS_ephem_sptr);
}
void SerialCmdInterface::set_RTK_solver(std::shared_ptr<Rtklib_Solver> rtklib_solver_sptr)
{
    Rtklib_solver_sptr_=std::move(rtklib_solver_sptr);
}
// void SerialCmdInterface::get_SerialCmd(std::shared_ptr<GNSSFlowgraph>)
// {

// }


// void SerialCmdInterface::run_serial_listener(char* device, char* buffer, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr)
void SerialCmdInterface::run_serial_listener(char* device, char* buffer, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr)
{
    // const char *cstr = input.c_str();
    while (keep_running_ == true)
        {
            // int msg;
            std::cout<<"run_serial_listener: Esperando Comando"<<"\n";
            serial4readByte(buffer);  // Vai esperar um cmd pela porta serial.
            // CmdParser(input); // Melhor remover essa função e apply action lá na control_thread
            if (control_queue_ != nullptr)
                {
                    // const command_event_sptr new_evnt = command_event_make(300,(int)(*buffer));
                    // control_queue_->push(pmt::make_any(new_evnt));
                    // flowgraph->apply_action((int*)input);
                    // flowgraph_sptr->apply_action(300,(int)(*buffer));
                    // const command_event_sptr new_evnt = command_event_make(200, 1);  // send the restart message (who=200,what=1)
                    // control_queue_->push(pmt::make_any(new_evnt));
                    // std::cout<<"Evento de leitura"<<"\n";
                    // flowgraph_sptr->apply_action(300, (int)(*buffer));
                    CmdParser(buffer);
                }
        }
}

void SerialCmdInterface::register_functions(void)
{
    // functions_["serial_get_pvt"] = [&](auto &s) { return SerialCmdInterface::serial_get_pvt(s); };
    // functions_["serial_reset"] = [&](auto &s) {return SerialCmdInterface::serial_reset(s);};
    // functions_[""]
}

void SerialCmdInterface::CmdParser(char* cmd)
{
    switch ((int)(*cmd)) //Gambiarra, tem que ajeitar isso aqui.
    {
        case 0x01:
            // serial_standby();
            break;
        case 0x02:
            // serial_reset();
            break;
        case 0x03:
            // serial_get_pvt();
            break;
        case 0x04:
            // serial_get_synchro();
            break;
        case 0x05:
            // serial_get_Ephemeris();
            break;
        case 0x06:
            // serial_hotstart();
            break;
        case 0x07:
            {
                std::cout << "Protocolo Anderson: "
                          << "\n";
                DersoProtocol();
            }
        default:
            char buff = 0xFF;
            serial4send(&buff);
    }
}

std::string SerialCmdInterface::serial_get_pvt(void)
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
            // Sync_sptr_->Pseudorange_m;
            // Sync_sptr_->
            // Rtklib_solver_sptr_->
        }
}

void SerialCmdInterface::serial_reset(void)
{
    const command_event_sptr new_evnt = command_event_make(200, 1);  // send the restart message (who=200,what=1)
    control_queue_->push(pmt::make_any(new_evnt));
    std::cout << "OK. Receptor GNSS reiniciando ...\n";
}

std::string SerialCmdInterface::serial_status(const std::vector<std::string> &commandLine __attribute__((unused)))
{
    char buff[100];
    memset(buff,'\0',sizeof(buff));
    sprintf(buff, "Tá Rodando ...", NULL) ;
    serial4send(&buff[0]);
}

void SerialCmdInterface::serial_status(void)
{
    char buff[100];
    memset(buff,'\0',sizeof(buff));
    sprintf(buff, "Tá Rodando ...", NULL) ;
    serial4send(&buff[0]);
}

void SerialCmdInterface::serial_get_ephemeris(void)
{
    // Precisa percorrer os canais

}

void SerialCmdInterface::DersoProtocol(void)
{
    //Função de passar toda a estrutura do Anderson
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
            sprintf(buff, "%d%d%lf%lf%lf%lf%lf%d\n",0x4d,
                                                 UTC_time,
                                                 0x02,
                                                 0x00,
                                                 Rtklib_solver_sptr_->get_gdop(),
                                                 Rtklib_solver_sptr_->get_hdop(),
                                                 Rtklib_solver_sptr_->get_vdop(),
                                                 Rtklib_solver_sptr_->get_pdop(),
                                                 latitude_deg,
                                                 longitude_deg,
                                                 height_m,
                                                 0xFF
                                                 );
            serial4send(&buff[0]);
        }
}
void SerialCmdInterface::set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue)
{
    control_queue_ = std::move(control_queue);
}