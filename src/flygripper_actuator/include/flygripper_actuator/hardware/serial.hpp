#pragma once
#include <string>
#include <cstring>
#include <iostream>
#include <memory>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace flygripper_actuator {
/**
 * @brief serial class
 */
class Serial {
 public:
  using Ptr = std::shared_ptr<Serial>;
  /**
   * @brief Constructor of serial
   * @param port_name port name, i.e. /dev/ttyUSB0
   * @param baudrate serial baudrate
   */
  Serial(std::string port_name,
         int baudrate, int data_bits = 8, char parity_bits = 'N', int stop_bits = 1) :
      port_name_(port_name),
      baudrate_(baudrate),
      data_bits_(data_bits),
      parity_bits_(parity_bits),
      stop_bits_(stop_bits) {};
  /**
   * @brief Destructor of serial to close the device
   */
  ~Serial() {
    this->Close();
  }
  /**
   * @brief Initialization of serial device to config and open the device
   * @return True if success
   */
  bool Init() {
    std::cout << "Attempting to open device " << port_name_ << " with baudrate " << baudrate_ << std::endl;
    if (port_name_.c_str() == nullptr) {
      port_name_ = "/dev/ttyUSB0";
    }
    if (this->Open() && this->Config()) {
      FD_ZERO(&serial_fd_set_);
      FD_SET(serial_fd_, &serial_fd_set_);
      std::cout << "...Serial started successfully." << std::endl;
      return true;
    } else {
      std::cerr << "...Failed to start serial " << port_name_ << std::endl;
      this->Close();
      return false;
    }
  };
  /**
   * @brief Serial read function
   * @param buf Given buffer to be updated by reading
   * @param len Read data length
   * @return -1 if failed, else the read length
   */
  int Read(uint8_t *buf, int len) {
    int ret = -1;

    if (NULL == buf) {
      return -1;
    } else {
      ret = read(serial_fd_, buf, len);
//    std::cout << "Read once length: " << ret << std::endl;
      while (ret == 0) {
        std::cerr << "Connection closed, try to reconnect." << std::endl;
        while (!Init()) {
          usleep(500000);
        }
        std::cout << "Reconnect Success." << std::endl;
        ret = read(serial_fd_, buf, len);
      }
      return ret;
    }
  };
  /**
   * @brief Write the buffer data into device to send the data
   * @param buf Given buffer to be sent
   * @param len Send data length
   * @return < 0 if failed, else the send length
   */
  int Write(const uint8_t *buf, int len) {
    return write(serial_fd_, buf, len);
  }

 private:
  /**
   * @brief Open the serial
   * @return True if open successfully
   */
  bool Open() {

#ifdef __arm__
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif

    if (serial_fd_ < 0) {
      std::cerr << "cannot open device " << serial_fd_ << " " << port_name_ << std::endl;
      return false;
    }

    return true;
  }
  /**
   * @brief Close the serial
   * @return True if close successfully
   */
  bool Close() {
    close(serial_fd_);
    serial_fd_ = -1;
    return true;
  }

  /**
   * @brief Configure the serial
   * @return True if configure successfully
   */
  bool Config() {
    int st_baud[] = {B4800, B9600, B19200, B38400,
                     B57600, B115200, B230400, B921600, B1000000, B2000000};
    int std_rate[] = {4800, 9600, 19200, 38400,
                      57600, 115200, 230400, 921600, 1000000, 2000000};
    /* save current port parameter */
    if (tcgetattr(serial_fd_, &old_termios_) != 0) {
      std::cerr << "fail to save current port" << std::endl;
      return false;
    }
    memset(&new_termios_, 0, sizeof(new_termios_));

    /* config the size of char */
    new_termios_.c_cflag |= CLOCAL | CREAD;
    new_termios_.c_cflag &= ~CSIZE;

    /* config data bit */
    switch (data_bits_) {
      case 7:new_termios_.c_cflag |= CS7;
        break;
      case 8:new_termios_.c_cflag |= CS8;
        break;
      default:new_termios_.c_cflag |= CS8;
        break; //8N1 default config
    }
    /* config the parity bit */
    switch (parity_bits_) {
      /* odd */
      case 'O':
      case 'o':new_termios_.c_cflag |= PARENB;
        new_termios_.c_cflag |= PARODD;
        break;
        /* even */
      case 'E':
      case 'e':new_termios_.c_cflag |= PARENB;
        new_termios_.c_cflag &= ~PARODD;
        break;
        /* none */
      case 'N':
      case 'n':new_termios_.c_cflag &= ~PARENB;
        break;
      default:new_termios_.c_cflag &= ~PARENB;
        break; //8N1 default config
    }
    /* config baudrate */
    for (int i = 0; i < sizeof(std_rate); ++i) {
      if (std_rate[i] == baudrate_) {
        /* set standard baudrate */
        cfsetispeed(&new_termios_, st_baud[i]);
        cfsetospeed(&new_termios_, st_baud[i]);
        break;
      }
    }
    /* config stop bit */
    if (stop_bits_ == 1)
      new_termios_.c_cflag &= ~CSTOPB;
    else if (stop_bits_ == 2)
      new_termios_.c_cflag |= CSTOPB;
    else
      new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

/* config waiting time & min number of char */
    new_termios_.c_cc[VTIME] = 1;
    new_termios_.c_cc[VMIN] = 18;

    /* using the raw data mode */
    new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_termios_.c_oflag &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd_, TCIFLUSH);

    /* activite the configuration */
    if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
      std::cerr << "failed to activate serial configuration" << std::endl;
      return false;
    }
    return true;

  }

  //! port name of the serial
  std::string port_name_;
  //! baudrate of the serial
  int baudrate_;
  //! stop bits of the serial, as default
  int stop_bits_;
  //! data bits of the serial, as default
  int data_bits_;
  //! parity bits of the serial, as default
  char parity_bits_;
  //! serial handler
  int serial_fd_;
  //! set flag of serial handler
  fd_set serial_fd_set_;
  //! termios config for serial handler
  struct termios new_termios_, old_termios_;
};
}