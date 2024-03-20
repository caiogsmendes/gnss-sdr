// Ainda não esta pronto para produção.

/**
 * TODO: 
 * -> Colocar uma struct com o estado do file da UART
 * -> adicionar polling para gerenciamento da UART
 * -> Separar a struct de config do termios
 * 
 * */ 

/**
 * Bug: A função write() da biblioteca unistd.h está sendo ignorada e o compilador está pegando uma outra função
 * da biblioteca boost. Não é possível overload uma função em C.
*/


// #include <iostream>
//#include <string>
//#include ""
//#include "hetech_protocol.pb.h"


#define BUFF_SIZE 2048
#define POLL_TIMEOUT 200


extern "C"
{
#include "HEtechSerial.h"
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
// #include "intercept.h"


// How do i override this thing?
// #define write (int __fd, const void *__buf, size_t __n)

    // ######### Experimental #########

    // ################################

    // struct pollfd ufds;

    // void HEserial_connect();
    // void HEserial_send();
    // void HEserial_put();
    // void HEserial_get();
    // void HEserial_set();
    // void HEserial_available();
    // void HEserial_close();

    // // Antigas Funções

    int enviaar(char *msg)
    {
        // char buf_rx[100];
        char buf_tx[100];
        const char *device = "/dev/colibri-uartc";
        struct termios tty;
        int fd;
        int flags = O_RDWR | O_NOCTTY | O_NDELAY; /* O_RDWR Read/write access to the serial port */
                                                  /* O_NOCTTY No terminal will control the process */
                                                  /* O_NDELAY Use non-blocking I/O */

        /*------------------------------- Opening the Serial Port -------------------------------*/
        fd = open(device, flags);

        if (fd == -1)
        {
            printf("\n Failed to open port! ");
            return -1;
        }

        /*---------- Serial port settings using the termios structure --------- */
        /* Settings (9600/8N1):
        Baud rate: 9600 baud
        Data bits: 8
        Parity bit: No
        Stop bit: 1
        Hardware Flow Control: No
        */

        tcgetattr(fd, &tty); /* Get the current attributes of the Serial port */

        cfsetispeed(&tty, B115200); /* Set read speed as 9600 baud                       */
        cfsetospeed(&tty, B115200); /* Set write speed as 9600 baud                      */
        // cfmakeraw(&tty);
        tty.c_cflag &= ~PARENB;  /* Disables the Parity Enable bit(PARENB)  */
        tty.c_cflag &= ~CSTOPB;  /* Clear CSTOPB, configuring 1 stop bit    */
        tty.c_cflag &= ~CSIZE;   /* Using mask to clear data size setting   */
        tty.c_cflag |= CS8;      /* Set 8 data bits                         */
        tty.c_cflag &= ~CRTSCTS; /* Disable Hardware Flow Control           */
        tty.c_cflag |= CREAD;// | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        if ((tcsetattr(fd, TCSANOW, &tty)) != 0)
        { /* Write the configuration to the termios structure*/
            printf("Error! Can't set attributes.\n");
            return -1;
        }
        else
        {
            printf("All set! \n");
        }

        tcflush(fd, TCIFLUSH);

        strncpy(buf_tx, msg, sizeof(buf_tx));

        int result = write(fd, &msg[0], strlen(msg));
        if (result == -1)
        {
            printf("Error: %s\n", strerror(errno));
            return -1;
        }
        else
        {
            printf("%d bytes sent\n", result);
        }
        close(fd);
    }
    
        void serial_envio(const char *device, int flags, char *msg)
        {
            /**
             * Função para envio de um valor apenas
             */
            char buf_tx[1000];
            memset(buf_tx, '\0', sizeof(buf_tx)); // Limpa o buffer.
            struct termios tty;

            // int res = 0, err = 0;
            // struct pollfd ufds;

            int fd;
            fd = open(device, flags);
            if (fd == -1)
            {
                printf("Falha em abrir port UART\n");
            }

            tcgetattr(fd, &tty);


            tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
            tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
            tty.c_cflag |= CS8;            // 8 bits per byte (most common)
            tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO;                                                        // Disable echo
            tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
            tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
            tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

            // tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            // tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);

            if ((tcsetattr(fd, TCSANOW, &tty)) != 0)
            {
                printf("Erro em setar atributos\n");
            }

            tcflush(fd, TCIFLUSH);
            // strncpy(buf_tx, /*(char *)*/ msg, sizeof(buf_tx));
             memcpy(buf_tx, msg, strlen(msg));
            int result = write(fd, &buf_tx, strlen(buf_tx));
            if (result == -1)
            {
                printf("Erro: %s\n", strerror(errno));
            }
            close(fd);
        }

        void serial_envioByte(const char* device, int flags, double *msg)
        {
            struct termios tty;
            int fd3;
            fd3 = open(device, flags);
            if(fd3 == -1)
            {
                printf("Falha em abrir port UART\n");
            }
            tcgetattr(fd3, &tty);

            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CRTSCTS;

            cfsetispeed(&tty, B921600);
            cfsetospeed(&tty, B921600);

            if ((tcsetattr(fd3, TCSANOW, &tty)) != 0)
            {
                printf("Erro em setar atributos\n");
            }
            else
            {
                printf("UART set\n");
            }
            tcflush(fd3, TCIFLUSH);
            int result = write(fd3, &msg, sizeof(msg));
            if (result == -1)
            {
                printf("Erro: %s\n", strerror(errno));
            }
            close(fd3);
        }

    int serial4send(char *data)
    {
        // Configs de Escrita UART
        // const char *device = "/dev/colibri-uartc";
        const char *device = "/dev/ttyUSB0";
        int flags = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK; //Tirei o NonBlock pro poll bloquear a escrita
        //
        struct serial_s comm = HEserial_connect(device, flags);
        //
        int bytes = HEserial_envio(&comm, data);
        //
        printf("%d bytes enviados\n",bytes);
        HEserial_disconnect(&comm);
        return bytes;
    }

    int serial4read(char *data)
    {
        // Configs de Leitura UART
        const char *device = "/dev/colibri-uartc";
        int flags = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK;

        struct serial_s comm = HEserial_connect(device, flags);
        
        int bytes = HEserial_leitura(&comm, data);
        
        HEserial_disconnect(&comm);
        return bytes;
    }

    void serial4readByte(char *data)
    {
        // Configs de Leitura UART
        const char *device = "/dev/ttyUSB0";
        int flags = O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK;
        struct serial_s comm = HEserial_connect(device, flags);
        *data = HEserial_leitura_byte(&comm, data);
        HEserial_disconnect(&comm);
        // return byte;
    }

    // ####### New Functions ##########

    // struct serial_s HEserial_init(const char *device, int flags) // Não usar
    // {
    //     struct serial_s comm;
    //     comm.fd = *device;
    //     comm.flags = flags;
    //     return comm;
    // }

    struct serial_s HEserial_connect(const char *device, int flags)
    {
        struct serial_s comm;
        comm.fd = open(device, flags); // cria file para IO
        tcgetattr(comm.fd, &comm.tty); // pega atributos e cria struct termios(termios.h)
        cfsetispeed(&comm.tty, B115200);
        cfsetospeed(&comm.tty, B115200);
        comm.tty.c_cflag &= ~PARENB;   // Disable geração/check de Bit de pariedade
        comm.tty.c_cflag &= ~CSTOPB;   // set 1 Stop Bit
        comm.tty.c_cflag |= CREAD | CLOCAL;
        comm.tty.c_cflag &= ~CSIZE;
        comm.tty.c_cflag |= CS8;       //Character size mask
        comm.tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS (hardware) flow control
        comm.tty.c_lflag &= ~ICANON ;   // Disable canonical mode
        comm.tty.c_lflag &= ~ECHO;                                                        // Disable echo
        comm.tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        comm.tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        comm.tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        comm.tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        comm.tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
        // comm.tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        // comm.tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        
        // comm.tty.c_cc[VTIME] = 0;
        // comm.tty.c_cc[VMIN] = 232; // É bom revisar esses números, n sei se leitura e escrita usariam os mesmos parâmetros.
        

        
        tcsetattr(comm.fd, TCSANOW, &comm.tty);
        
        tcflush(comm.fd, TCIFLUSH);
        comm.ufds.fd = comm.fd;
        return comm;
    }

    int HEserial_envio(serial_s* comm, char* msg)
    {
        /**
         * Precisa abrir um buff aqui. Do contrário a função write, envia apenas 1 byte.
         * Como estava sendo anteriormente, os parametros da função write estavam com ponteiros e o sizeof() estava retornando 
         * o valor do tamanho do ponteiro e não do vetor char.
         * Então esse memcpy() vai ter que ficar.
        */

        char buff[1000];

        // Limpar o buffer de caracteres espúrios
        memset(&buff, '\0', sizeof(buff));
        memcpy(&buff,msg,sizeof(buff));
        int result = 0;
        comm->ufds.events = POLLOUT;
        if (poll(&comm->ufds, 1, -1) > 0)
        {
            if (comm->ufds.revents & POLLOUT)
            {
                result = write(comm->fd, &buff[0], strlen(buff));
            }
        }
        return result;
    }

    int HEserial_leitura(serial_s *comm, char *msg)
    {
        // Limpar o buffer se characteres espúrios
        memset(comm->rxbuff, '\0', sizeof(comm->rxbuff));
        int result = 0;
        comm->ufds.events = POLLIN;
        if (poll(&comm->ufds, 1, -1) > 0)
        {
            if (comm->ufds.revents & POLLIN)
            {
                result = read(comm->fd, &comm->rxbuff, sizeof(comm->rxbuff));
                memcpy(msg, &comm->rxbuff, sizeof(comm->rxbuff)); // Está Redundante ??
            }
        }
        return result;
    }

    char HEserial_leitura_byte(serial_s *comm, char *msg)
    {
        // Limpar o buffer se characteres espúrios
        // memset(comm->rxbuff, '\0', sizeof(comm->rxbuff));
        comm->ufds.events = POLLIN;
        if (poll(&comm->ufds, 1, -1) > 0)
        {
            if (comm->ufds.revents & POLLIN)
            {
                printf("Alguma coisa foi lida na HEserial_leitura_byte\n");
                int result = read(comm->fd, &comm->buffrx, sizeof(comm->buffrx));
                memcpy(msg, &comm->buffrx, sizeof(comm->buffrx)); // Está Redundante ??
            }
        }
        return comm->buffrx;
    }

    void HEserial_disconnect(serial_s *comm)
    {
        close(comm->fd);
    }

    // void doubletoHex(double* input, uint8_t* output)
    // {

    // }
}
