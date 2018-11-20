#include <string>
#include <vector>
#include "myserial.h"
#include <ctime>

using namespace std;
using namespace boost::asio;
using namespace serial_helper;
MySerial::MySerial(const std::string &port_name)
{
    pSerialPort = new serial_port(io_s);
    if (pSerialPort)
    {
        init_port(port_name, 8);
    }
}

MySerial::~MySerial()
{
    if (pSerialPort)
    {
        delete pSerialPort;
    }
}

bool MySerial::init_port(const string port_name, const unsigned char char_size)
{
    if (!pSerialPort)
    {
        return false;
    }

    pSerialPort->open(port_name, ec);

    pSerialPort->set_option(serial_port::baud_rate(115200), ec); //比特率

    pSerialPort->set_option(serial_port::flow_control(serial_port::flow_control::none), ec); //流量控制

    pSerialPort->set_option(serial_port::parity(serial_port::parity::none), ec); //奇偶校验

    pSerialPort->set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec); //停止位

    pSerialPort->set_option(serial_port::character_size(char_size), ec); //数据位

    return true;
}

void MySerial::write_chars(const std::vector<char> data)
{
    std::string str(data.data(), data.size());
    write(*pSerialPort, buffer(str), ec);
}

void MySerial::write_chars(const char* data, size_t length)
{
    std::string str(data, length);
    write(*pSerialPort, buffer(str), ec);
}

void MySerial::write_to_serial(const string data)
{
    size_t len = write(*pSerialPort, buffer(data), ec);
    //cout << len << "\t" << data << "\n";
}

void MySerial::handle_read(boost::system::error_code ec, size_t bytes_transferred)
{
    if (ec)
    {
        std::cout<<"asio read failed, reason:"<<ec.message()<<std::endl;
        return;
    }
    /*
    cout << "\nhandle_read: ";
    for (int i = 0; i < bytes_transferred; i++)
    {
        printf("%c", my_buffer[i]);
    }
    cout<<endl;
    */
    is_finished_read = true;
    //cout.write(buf, bytes_transferred);
}

void MySerial::read_from_serial()
{
    async_read(*pSerialPort, buffer(my_buffer), boost::bind(&MySerial::handle_read, this,  _1, _2));
}



void MySerial::call_handle()
{
    //There can use deadline_timer to cancle serial_port read data
    //Wait for call callback function
    is_finished_read = false;
    io_s.run();
    io_s.reset();
}

int MySerial::call_handle(double seconds)
{
    clock_t start;
    start = std::clock();
    double duration;
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    is_finished_read = false;
    while(!is_finished_read && duration < seconds)
    {
        io_s.poll_one();
        if(is_finished_read)
        {
            io_s.reset();
            return sizeof(my_buffer);
        }
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    }
    std::cout<<"overtime"<<endl;
    return -1;
}



void MySerial::flush()
{
    flush_serial_port(*pSerialPort, flush_receive, ec);
}
