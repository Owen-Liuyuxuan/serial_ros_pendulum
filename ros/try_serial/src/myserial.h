#ifndef MY_SERIAL_H_
#define MY_SERIAL_H_

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <vector>

class MySerial
{
  private:
    bool init_port(const std::string port, const unsigned char char_size);

  public:
    MySerial(const std::string &port_name);
    ~MySerial();

    void write_chars(const std::vector<char> data);
    void write_chars(const char* data, size_t length);

    void write_to_serial(const std::string data);

    void read_from_serial();

    void handle_read(boost::system::error_code ec, std::size_t byte_transformed);

    void call_handle();

    int call_handle(double seconds);
    
    void flush();

  private:
    boost::asio::io_service io_s;

    boost::asio::serial_port *pSerialPort;

    std::string port;

    boost::system::error_code ec;

  public:
    bool is_finished_read;
    char my_buffer[4];
};

namespace serial_helper
{
enum flush_type
{
    flush_receive = TCIFLUSH,
    flush_send = TCOFLUSH,
    flush_both = TCIOFLUSH
};

void flush_serial_port(
    boost::asio::serial_port &serial_port,
    flush_type what,
    boost::system::error_code &error)
{
    if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what))
    {
        error = boost::system::error_code();
    }
    else
    {
        error = boost::system::error_code(errno,
                                          boost::asio::error::get_system_category());
    }
}
} // namespace serial_helper

#endif // !MY_SERIAL_H_
