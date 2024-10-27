#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <chrono>
#include <thread>


using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals;


// *Structure of an SBUS Msg
struct Sbusmsg
{
    static constexpr int NofChannels = 16;
    // declare channels, each will have 11 bits
    uint16_t channels[NofChannels];

    // CH 17 AND 18
    bool digi_channel_1 = false;
    bool digi_channel_2 = false;

    // FLAGS
    bool frame_lost;
    bool failsafe;
};

class SBusSerialPort
{
private:
    static constexpr int SbusFrameLength_ = 25;
    static constexpr uint8_t SbusHeaderByte_ = 0x0F;
    static constexpr uint8_t SbusFooterByte_ = 0x00;

    int pwmchannels[9];

    bool configureSerialPortForSBus() const;

    int serial_port_fd_;
    int fd;
    Sbusmsg sm;
    // bool flag = false;
public:
    SBusSerialPort();
    bool setUpSBusSerialPort(const std::string &port);
    bool connectSerialPort(const std::string &port);
    void disconnectSerialPort();
    void TXSerialSBusMsg() const;
    void PWM_interpreter(int channels[9]);
    void read_joy();
    int joy_setup();

    ~SBusSerialPort();
};

SBusSerialPort::SBusSerialPort()
{
    std::string port = "/dev/ttyUSB1";
    // configure both serial ports
    setUpSBusSerialPort(port);
    joy_setup();
    for (int i = 0; i < 16; i++) {
    sm.channels[i] = 992;
    }
}

SBusSerialPort::~SBusSerialPort()
{
    disconnectSerialPort();
}

bool SBusSerialPort::configureSerialPortForSBus() const {
  // clear config
  fcntl(serial_port_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_port_fd_, F_SETFL, FNDELAY);

  struct termios2 uart_config;
  /* Fill the struct for the new configuration */
  ioctl(serial_port_fd_, TCGETS2, &uart_config);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
//   uart_config.c_iflag &=~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  //
  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  //
//   uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // Turn off odd parity
  uart_config.c_cflag &= ~(CSIZE | PARODD);

  // Enable parity generation on output and parity checking for input.
  uart_config.c_cflag |= PARENB;
  // Set two stop bits, rather than one.
  uart_config.c_cflag |= CSTOPB;
  // No output processing, force 8 bit input
  uart_config.c_cflag |= CS8;
  // Enable a non standard baud rate
//   uart_config.c_cflag |= BOTHER;

  // * Custom baud rate of 100'000 bits/s necessary for sbus
  // ! updated to 115200 to support pMDDL 2450 data links
  const speed_t spd = 115200;
  uart_config.c_ispeed = spd;
  uart_config.c_ospeed = spd;

  if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
    std::cerr<<"Serial Port issue";
    return false;
  }

  return true;
}

bool SBusSerialPort::connectSerialPort(const std::string& port) {
  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_port_fd_ == -1) {
    return false;
  }

  if (!configureSerialPortForSBus()) {
    close(serial_port_fd_);
    std::cerr<<"Serial Port Connection Issue.";
    return false;
  }

  return true;
}

void SBusSerialPort::disconnectSerialPort() {
  close(serial_port_fd_);
}

bool SBusSerialPort::setUpSBusSerialPort(const std::string& port) {
  if (!connectSerialPort(port)) {
    return false;
  }

  return true;
}

// *fxn to create the sbusmsg and write it to the serial port. 
void SBusSerialPort::TXSerialSBusMsg() const
{
    // for(int i=0; i<16;i++){
    //     std::cout<<sm.channels[i]<<" ";
    // }
    // std::cout<<std::endl;
    while (1)
    {
        static uint8_t buffer[SbusFrameLength_];

        // header
        buffer[0] = SbusHeaderByte_;

        // footer
        buffer[24] = SbusFooterByte_;

        // start channels
        //  16 channels of 11 bit data
        buffer[1] = (uint8_t)((sm.channels[0] & 0x07FF));
        buffer[2] = (uint8_t)((sm.channels[0] & 0x07FF) >> 8 |
                              (sm.channels[1] & 0x07FF) << 3);
        buffer[3] = (uint8_t)((sm.channels[1] & 0x07FF) >> 5 |
                              (sm.channels[2] & 0x07FF) << 6);
        buffer[4] = (uint8_t)((sm.channels[2] & 0x07FF) >> 2);
        buffer[5] = (uint8_t)((sm.channels[2] & 0x07FF) >> 10 |
                              (sm.channels[3] & 0x07FF) << 1);
        buffer[6] = (uint8_t)((sm.channels[3] & 0x07FF) >> 7 |
                              (sm.channels[4] & 0x07FF) << 4);
        buffer[7] = (uint8_t)((sm.channels[4] & 0x07FF) >> 4 |
                              (sm.channels[5] & 0x07FF) << 7);
        buffer[8] = (uint8_t)((sm.channels[5] & 0x07FF) >> 1);
        buffer[9] = (uint8_t)((sm.channels[5] & 0x07FF) >> 9 |
                              (sm.channels[6] & 0x07FF) << 2);
        buffer[10] = (uint8_t)((sm.channels[6] & 0x07FF) >> 6 |
                               (sm.channels[7] & 0x07FF) << 5);
        buffer[11] = (uint8_t)((sm.channels[7] & 0x07FF) >> 3);
        buffer[12] = (uint8_t)((sm.channels[8] & 0x07FF));
        buffer[13] = (uint8_t)((sm.channels[8] & 0x07FF) >> 8 |
                               (sm.channels[9] & 0x07FF) << 3);
        buffer[14] = (uint8_t)((sm.channels[9] & 0x07FF) >> 5 |
                               (sm.channels[10] & 0x07FF) << 6);
        buffer[15] = (uint8_t)((sm.channels[10] & 0x07FF) >> 2);
        buffer[16] = (uint8_t)((sm.channels[10] & 0x07FF) >> 10 |
                               (sm.channels[11] & 0x07FF) << 1);
        buffer[17] = (uint8_t)((sm.channels[11] & 0x07FF) >> 7 |
                               (sm.channels[12] & 0x07FF) << 4);
        buffer[18] = (uint8_t)((sm.channels[12] & 0x07FF) >> 4 |
                               (sm.channels[13] & 0x07FF) << 7);
        buffer[19] = (uint8_t)((sm.channels[13] & 0x07FF) >> 1);
        buffer[20] = (uint8_t)((sm.channels[13] & 0x07FF) >> 9 |
                               (sm.channels[14] & 0x07FF) << 2);
        buffer[21] = (uint8_t)((sm.channels[14] & 0x07FF) >> 6 |
                               (sm.channels[15] & 0x07FF) << 5);
        buffer[22] = (uint8_t)((sm.channels[15] & 0x07FF) >> 3);

        // SBUS flags1
        // (bit0 = least significant bit)
        // bit0 = ch17 = digital channel (0x01)
        // bit1 = ch18 = digital channel (0x02)
        // bit2 = Frame lost, equivalent red LED on receiver (0x04)
        // bit3 = Failsafe activated (0x08)
        // bit4 = n/a
        // bit5 = n/a
        // bit6 = n/a
        // bit7 = n/a
        buffer[23] = 0x00;
        // TODO digi channel logic
        if (sm.digi_channel_1)
        {
            buffer[23] |= 0x01;
        }
        if (sm.digi_channel_2)
        {
            buffer[23] |= 0x02;
        }
        if (sm.frame_lost)
        {
            buffer[23] |= 0x04;
        }
        if (sm.failsafe)
        {
            buffer[23] |= 0x08;
        }

        // for(int i=0; i<25; i++){
        //     write(serial_port_fd_, &buffer[i], 1);
        // }
        const int written = write(serial_port_fd_, &buffer, SbusFrameLength_); // prime
        sleep_for(3ms);
    }
    // std::cout<<"tx done"; //debug msgs
}

// *fxn that converts the joystick values to SBUS standard PWM values. 
// *   Range = 172 - 1812. (in usec for T-on out of T-period 3msec)
void SBusSerialPort::PWM_interpreter(int channels[9]){

    // std::cout<<channels[0]<<std::endl;
    sm.channels[0] = 992;           // set roll to neutral
    sm.channels[2]=(channels[0]*1640)/242 + 172;    //throttle
    sm.channels[3]=(channels[1]*1640)/255 + 172;    //yaw
    sm.channels[1]=(channels[2]*1640)/255 + 172;    //pitch
    sm.channels[6]= (channels[3]*1640)+172;         //down
    sm.channels[7]= (channels[4]*1640)+172;         //left
    sm.channels[8]= (channels[5]*1640)+172;         //up
    sm.channels[9]= (channels[6]*1640)+172;         //right
    sm.channels[10]= (channels[7])*1640 + 172;      //downpress aux fxn
    // cout<<"pwm\n";
}

int SBusSerialPort::joy_setup(){
    
    std::string joyPort = "/dev/ttyUSB0"; //Joy Device Address
    fd = open(joyPort.c_str(), O_RDWR | O_NOCTTY );
    if (fd < 0) {
        perror("problem in serial port");
        return -1;
    }

    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) == -1)
    {
        perror("Error getting serial port attributes");
        close(fd);
        return -1;
    }
    // Set baud rate
    tio.c_ispeed = 115200;
    tio.c_ospeed = 115200;

    // 8 data bits, no parity, 1 stop bit
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;

    if (ioctl(fd, TCSETS2, &tio) == -1)
    {
        perror("Error setting serial port attributes");
        close(fd);
        return -1;
    }

    //opening port complete
    return fd;

}


// *fxn to read joy in blocks of 8 bytes after detecting new line character over serial. This will pass to PWM Interpreter fxn.
void SBusSerialPort::read_joy()
{
    while (1)
    {
        unsigned char buffer[9];
        ssize_t bytesRead;
        bool flag = false;

        while (!flag)
        {
            bytesRead = read(fd, &buffer, 1); // Read one byte at a time
            // std::cout<<buffer[0]<<std::endl;
            if (bytesRead == 1 && buffer[0] == 0xFF)
            {
                flag = true;
                // std::cout<<"flag set"<<std::endl;
            }
        }

        bytesRead = read(fd, buffer + 1, 8); // Read remaining bytes

        if (bytesRead < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No data available, wait or perform other tasks
                sleep_for(20ms); // Sleep for 10 milliseconds
                // continue;
            }
            else
            {
                perror("Error reading from serial port");
                // break;
            }
        }
        else
        {
            // std::cout << "Read " << bytesRead + 1 << " bytes: ";
            for (int i = 0; i < 9; i++)
            {
                // std::cout << static_cast<int>(buffer[i]) << " ";
                pwmchannels[i] = int(buffer[i + 1]);
                // std::cout<<i<<"_"<<pwmchannels[i]<<" ";
            }
            // std::cout << std::endl;
            PWM_interpreter(pwmchannels);
        }
    }
    // for(int i=0; i<9; i++){
    //     pwmchannels[i]=0;
    // }
    // PWM_interpreter(sm, pwmchannels);
}

int main(){


    Sbusmsg sm;
    SBusSerialPort SBP;

    std::thread thread1(&SBusSerialPort::read_joy, &SBP);
    
    std::thread thread2(&SBusSerialPort::TXSerialSBusMsg, &SBP);

    // while(1){
    // // read serial here
    //     // SBP.read_joy();

    //     // SBP.TXSerialSBusMsg();
    // // tx thru callback chain
    // }
    thread1.join();
    thread2.join();
  // An SBUS message takes 3ms to be transmitted by the serial port so provide appropriate delay of 50ms
    return 0;
}