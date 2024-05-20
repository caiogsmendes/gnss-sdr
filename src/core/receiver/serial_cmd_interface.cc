
#include "serial_cmd_interface.h"

#include "command_event.h"
#include "pvt_interface.h"
#include "HEtechSerial.h"
#include "rtklib_solver.h"
#include "display.h"
#include "gnss_synchro_monitor.h"
#include "rtklib_pvt_gs.h"
// #include <ncurses.h>


// #include <map>


/**
 * Com o PVT_sptr_ 'e possivel acessar o mapping das ephemeris, 
 * Verificar tambem como puxar para essa classe, o acesso aos recursos do ponteiro d_user_pvt_solver do rtklib_pvt_gs.cc
 * Puxar para ca os ponteiros referentes ao GNSS Synchro
*/

SerialCmdInterface::SerialCmdInterface() : keep_running_(true)
{
    register_functions();
    control_queue_ = nullptr;
    // std::vector<Gnss_Synchro> vect_sat[16];
    channels_status_ = channel_status_msg_receiver_make();
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
void SerialCmdInterface::set_GNSS_Observ(std::shared_ptr<std::vector<int, Gnss_Synchro>> gnss_observables_sptr)
{
    gnss_observables_sptr_=std::move(gnss_observables_sptr);
}

// void SerialCmdInterface::set_serial_ptr_init(std::shared_ptr<SerialCmdInterface> serial_ptr_init)
// {
//     serial_cmd_intern_ptr_=std::move(serial_ptr_init);
// }

void SerialCmdInterface::set_channels(std::shared_ptr<std::vector<std::shared_ptr<ChannelInterface>>> channels_sptr)
{
    channels_sptr_= std::move(channels_sptr);
}

// void SerialCmdInterface::set_channels(std::shared_ptr<std::vector<std::shared_ptr<ChannelInterface>>> channels_sptr)
// {
//     channels_sptr_ = std::move(channels_sptr);
// }

// void SerialCmdInterface::set_gnss_synchro_monitor(std::vector<std::shared_ptr<gnss_synchro_monitor>> gnss_synchro_monitor_ptr)
// {
//     gnss_synchro_monitor_ptr_ = std::move(gnss_synchro_monitor_ptr);
// }

// void SerialCmdInterface::set_gnss_synchro_monitor(std::shared_ptr<std::vector<std::shared_ptr<Gnss_Synchro>>> gnss_synchro_monitor_sptr)
// {
//     gnss_synchro_monitor_sptr_ = std::move(gnss_synchro_monitor_sptr);
// }

// void SerialCmdInterface::run_serial_listener(char* device, char* buffer, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr)
void SerialCmdInterface::run_serial_listener(char* device, char* buffer, std::shared_ptr<GNSSFlowgraph> flowgraph_sptr)
{
    // PVT_sptr_->get_latest_PVT()
    // = PVT_sptr_->get_gps_ephemeris();
    while (keep_running_ == true)
        {

            // printf("Channel ID: %d\n",Sync_sptr_->Channel_ID);
            // int n_ch = static_cast<int>(channels_sptr_->size());
            // for (int n = 0; n < n_ch; n++)
            //     {
            //         std::shared_ptr<Channel> ch_sptr = std::dynamic_pointer_cast<Channel>(channels_sptr_->at(0));
            //         std::cout << TEXT_BOLD_BLUE
            //                   << "System: "
            //                   << ch_sptr->get_signal().get_satellite().get_system()
            //                   << " PRN: "
            //                   << ch_sptr->get_signal().get_satellite().get_PRN()
            //                   << TEXT_RESET << "\n";
            //     }
            // CospeEphemeris();
            // set_Sync();
            // std::cout<<"Channel_ID: "<<sats[0].Channel_ID<<"\n";
            // flowgraph_sptr->get_Acq();
            // std::map<int, std::shared_ptr<Gnss_Synchro>> current_channels_status = channels_status_->get_current_status_map();
            // for(const auto&x:current_channels_status)
            // {
            //     std::cout<<"Channel_ID: "<<x.second->Channel_ID<<"  "<<"PRN: "<<x.second->PRN<<"  "<<"\n";
            // }

            // // Gnss_Synchro - Correto
            // // Gnss_Ephemeris - Correto
            // // Gnss_PVT - Correto
            // std::map<int, Gps_Ephemeris> gps_map_ephem = PVT_sptr_->;
            // std::map<int, Gps_Ephemeris> gps_map_ephem = PVT_sptr_->get_gps_ephemeris();      // ERRADO!!!! DEFEITO!!
            // std::map<int, Gnss_Synchro> gnss_observ = flowgraph_sptr->get_pvt()->get_gnss_observables();
            // if(flowgraph_sptr->get_pvt()->get_gnss_observables()[0].Flag_valid_pseudorange == true){
            
            
            // system("clear");}
            // system("clear");
            // clear();
            // for (const auto& y : gps_map_ephem)
            //     {
                    // for (const auto& x : gnss_observ)
                    //     {
            //                 if (y.second.PRN == x.second.PRN)
            //                     {
            //                         if (x.second.fs != 0)
            //                             {
                                            // std::cout
                                            //     << TEXT_BOLD_RED
                                            //     << "Channel_ID: "
                                            //     << x.second.Channel_ID
                                            //     << " System: "
                                            //     << x.second.System
                                            //     << " Signal: "
                                            //     << x.second.Signal
                                            //     << " PRN: "
                                            //     << x.second.PRN
                                            //     << " PLL Lock: "
                                            //     << x.second.Flag_PLL_180_deg_phase_locked
                                            //     << " Valid PseudoRange: "
                                            //     << x.second.Flag_valid_pseudorange
                                            //     << " PseudoRange: "
                                            //     << x.second.Pseudorange_m
                                            //     << " CN0_dB_Hz: "
                                            //     << x.second.CN0_dB_hz
                                            //     // << " PosX: "
                                            //     // << y.second.satpos_X
                                            //     << " fs: "
                                            //     << x.second.fs
                                            //     << TEXT_RESET
                                            //     << "\r\n";
            //                             }
            //                     }
                        // }
            //     }
            // refresh();

            // for(const auto &x:gps_map_ephem)
            // {
            //     std::cout<<TEXT_GREEN
            //     << " sqrtA: "
            //     << x.second.sqrtA
            //     << " ecc: "
            //     << x.second.ecc
            //     << " inc: "
            //     << x.second.i_0
            //     << " OMEGA: "
            //     << x.second.OMEGA_0
            //     << " omega: "
            //     << x.second.omega
            //     << " M_0: "
            //     << x.second.M_0
            //     <<TEXT_RESET<<"\n";
            // }

            /**
             * Gravação de Arquivos
             */
            // std::fstream fileArq("ArqRx.txt", std::ios::in | std::ios::app);

            // for (const auto& x : flowgraph_sptr->get_pvt()->get_gnss_observables())
            //     {
            // for (const auto& y : PVT_sptr_->get_gps_ephemeris())
            //     {
            //         // if (x.second.PRN == y.second.PRN)
            //         //     {
            //         std::cout
            //             // << x.second.PRN
            //             // << x.second.Pseudorange_m
            //             // << x.second.Pseudorange_m
            //             << y.second.satpos_X
            //             << y.second.satpos_Y
            //             << y.second.satpos_Z
            //             << y.second.satvel_X
            //             << y.second.satvel_Y
            //             << y.second.satvel_Z
            //             << "\n";
            //     }
        }
// 
//         }
// }
// std::cout<< std::dynamic_pointer_cast<Channel> (channels_sptr_->at(0));
// std::shared_ptr<Channel> ch_sptr = std::dynamic_pointer_cast<Channel>(channels_sptr_->at(0));
// std::cout<<TEXT_GREEN<<ch_sptr->get_signal().get_satellite().get_PRN()<<TEXT_RESET<<"\n";
}

void SerialCmdInterface::register_functions(void)
{
    // functions_["serial_get_pvt"] = [&](auto &s) { return SerialCmdInterface::serial_get_pvt(s); };
    // functions_["serial_reset"] = [&](auto &s) {return SerialCmdInterface::serial_reset(s);};
    // functions_[""]
}

void SerialCmdInterface::CmdParser(char* cmd)
{
    char buffer[40];
    memset(buffer,'\0',sizeof(buffer));
    switch ((int)(*cmd))
    {
        case 0x01:
            // serial_standby();
            break;
        case 0x02:
            // serial_reset();
            break;
        case 0x03:
            serial_get_pvt();
            break;
        case 0x04:
            serial_get_synchro();
            // Sync_sptr_->Channel_ID
            // std::map<int, Gps_Ephemeris> variavel_test = PVT_sptr_->get_gps_ephemeris();
            // channels_sptr_->at(1);
            {
                int n_ch = static_cast<int>(channels_sptr_->size());
                for (int n = 0; n < n_ch; n++)
                    {
                        std::shared_ptr<Channel> ch_sptr = std::dynamic_pointer_cast<Channel>(channels_sptr_->at(0));
                        // std::cout << TEXT_BOLD_BLUE
                        //           << "System: "
                        //           << ch_sptr->get_signal().get_satellite().get_system()
                        //           << " PRN: "
                        //           << ch_sptr->get_signal().get_satellite().get_PRN()
                        //           << TEXT_RESET << "\n";
                        
                    }
            }break;
        case (0x05):{
            // serial_get_Ephemeris();
            CospeEphemeris();
        }break;
        case (0x06):
            // serial_hotstart();
            break;
        case (0x07):
            {
                std::cout << "Protocolo Anderson: "
                          << "\n";
                DersoProtocol();
            }break;
        default:
            char buff = 0xFF;
            // int bytes = serial4send(&buff);
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
    // std::map<int, Gps_Ephemeris> gps_map_ephem = PVT_sptr_->get_gps_ephemeris();

    // std::cout<<TEXT_RED<<gps_map_ephem[13].PRN <<TEXT_RESET<<"\n";
    // PVT_sptr_->get_gps_ephemeris
    // if (PVT_sptr_->get_latest_PVT(&longitude_deg,
    //         &latitude_deg,
    //         &height_m,
    //         &ground_speed_kmh,
    //         &course_over_ground_deg,
    //         &UTC_time) == true)
    //     {
    //         char buff[200];
    //         memset(buff,'\0',sizeof(buff));
    //         sprintf(buff, "1=%lf|2=%lf|3=%lf|4=%lf|5=%lf|6=%lf", latitude_deg, longitude_deg, height_m, ground_speed_kmh, course_over_ground_deg, UTC_time);
    //         serial4send(&buff[0]);
    //         // Sync_sptr_->Pseudorange_m;
    //         // Sync_sptr_->
    //         // Rtklib_solver_sptr_->
    //     }
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
    // serial4send(&buff[0]);
}

// void SerialCmdInterface::serial_status(void)
// {
//     char buff[100];
//     memset(buff,'\0',sizeof(buff));
//     sprintf(buff, "Tá Rodando ...", NULL) ;
//     serial4send(&buff[0]);
// }

// void SerialCmdInterface::serial_get_GPS_ephemeris(std::shared_ptr<Rtklib_Solver> Rtklib_solver_sptr_)
// {
//     // Precisa percorrer os canais
//     std::map<int, Gps_Ephemeris>::const_iterator GPS_received;
//     GPS_received = Rtklib_solver_sptr_->gps_ephemeris_map.find(gnss_observables_iter->second.PRN); //Defeito!!
// }

int SerialCmdInterface::DersoProtocol(void)
{
    //Função de passar toda a estrutura do Anderson
    double longitude_deg;
    double latitude_deg;
    double height_m;
    double ground_speed_kmh;
    double course_over_ground_deg;
    time_t UTC_time;
    int bytes;
    if (PVT_sptr_->get_latest_PVT(&longitude_deg,
            &latitude_deg,
            &height_m,
            &ground_speed_kmh,
            &course_over_ground_deg,
            &UTC_time) == true)
        {
            char buff[400];
            memset(buff, '\0', sizeof(buff));
            sprintf(buff, "%d,%lf,%d,%d,%lf%lf%lf%lf,%lf%lf%lf,%d\n", 0x4d,
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
                0xFF);
            // bytes = serial4send(&buff[0]);
        }
        // else{char buff_err = 0xE0; bytes = serial4send(&buff_err);}

    return bytes;
}

void SerialCmdInterface::set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue)
{
    control_queue_ = std::move(control_queue);
    
}

std::vector<Gnss_Synchro> SerialCmdInterface::Update_Synchro_Internals(const std::vector<Gnss_Synchro>& stocks, int n_channels)
{
    return stocks;    
    // internals[0].Channel_ID=stocks[0].Channel_ID;
    // const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);
    // internals.push_back(stocks);
    // d_nchannels;
    // internals[0].Channel_ID;
    // extern internals GNSS;
    // GNSS.Channel_ID;
    // int jj=0;
    // if (jj <= n_channels)
    //     {
    //         do
    //             {
    //                 (*satelites)[jj].Channel_ID = stocks[jj].Channel_ID;
    //                 (*satelites)[jj].PRN = stocks[jj].PRN;
    //                 jj++;
    //             }
    //         while (jj <= 16);
    //     }
    // else
    //     {
    //         jj = 0;
    //     }
    // for(int i=0; i<n_channels;i++)
    // {
    //     std::cout<<stocks[i].PRN<<"\n";
    // }
}

void SerialCmdInterface::serial_get_synchro()
{
    // std::map<int, Gnss_Synchro> gnss_observ = flowgraph_sptr->get_pvt()->get_gnss_observables();
    // system("clear");
    // for (const auto& x : gnss_observ)
    //     {
    //         std::cout
    //             << TEXT_BOLD_MAGENTA
    //             << "Channel_ID: "
    //             << x.second.Channel_ID
    //             << " PRN: "
    //             << x.second.PRN
    //             << " PLL Lock: "
    //             << x.second.Flag_PLL_180_deg_phase_locked
    //             << " Valid PseudoRange: "
    //             << x.second.Flag_valid_pseudorange
    //             << " PseudoRange: "
    //             << x.second.Pseudorange_m
    //             << " CN0_dB_Hz: "
    //             << x.second.CN0_dB_hz
    //             << TEXT_RESET
    //             << "\n";
    //     }
}

void SerialCmdInterface::CospeEphemeris(void)
{
    char buffTX[400];
    char tmpbuff[100];
    memset(&buffTX,'\0',sizeof(buffTX));
    // char *char_ptr;
    // system("clear");
    // std::map<int, Gps_Ephemeris> gps_map_ephem = PVT_sptr_->get_gps_ephemeris();
    // std::cout<<TEXT_RED<<"System: "<<gps_map_ephem[13].System<<"\n"
    //                     <<"PRN: "<<gps_map_ephem[13].PRN<<"\n"
    //                     <<"delta_n: "<<gps_map_ephem[13].delta_n<<"\n"
    //                     <<"semiEixo: "<<gps_map_ephem[13].sqrtA<<"\n"
    //                     <<"Excent: "<<gps_map_ephem[13].ecc<<"\n"
    //                     <<"Inclin: "<<gps_map_ephem[13].i_0<<"\n"
    //                     <<"Asc.Reta: "<<gps_map_ephem[13].OMEGA_0<<"\n"
    //                     <<"Arg.Perigeo: "<<gps_map_ephem[13].omega<<"\n"
    //                     <<"Anom. Media: "<<gps_map_ephem[13].M_0<<"\n"
    //                     <<"Something: "<<"\n"
    //                     <<TEXT_RESET<<"\n\n";
    for (const auto& y : PVT_sptr_->get_gnss_observables()) // defeito
        {
            for (const auto& x : PVT_sptr_->get_gps_ephemeris())
                {
                    // if (y.second.PRN == x.second.PRN)
                        {
                            std::cout << TEXT_BOLD_RED
                                      << "PRN: " << x.second.PRN << "\n"
                                      << "delta_n: " << x.second.delta_n << "\n"
                                      << "SemiEixo: " << x.second.sqrtA << "\n"
                                      << "Excent.: " << x.second.ecc << "\n"
                                      << "Inclin.: " << x.second.i_0 << "\n"
                                      << "Asc.Reta: " << x.second.OMEGA_0 << "\n"
                                      << "Arg.Perigeo: " << x.second.omega << "\n"
                                      << "Anom. Media: " << x.second.M_0 << "\n"
                                      << TEXT_RESET << "\n";
                        }
                }
        }
    // system("clear");
    // system("clear");
    // std::map<int,Gnss_Synchro> gnss_sync = PVT_sptr_->get_gnss_observables();
    // for (auto const& y : gnss_sync){
    // for (auto const& x : PVT_sptr_->get_gps_ephemeris())
    //     {
    //         sprintf(tmpbuff,
    //         "%d%lf%lf%lf%lf%lf%lf%lf",
    //         x.second.PRN,
    //         y.second.Pseudorange_m,
    //         x.second.sqrtA,
    //         x.second.ecc,
    //         x.second.i_0,
    //         x.second.OMEGA_0,
    //         x.second.omega,
    //         x.second.M_0);
    //         strcat(buffTX, tmpbuff);
    //     }
    // }
    //     serial4send(&buffTX[0]);
}