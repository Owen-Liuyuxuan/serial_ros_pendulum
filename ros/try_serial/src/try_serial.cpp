#include <string>

#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "myserial.h"
#include <random>

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <inverted_pendulum/pendulum.h>

using namespace boost::asio;
using namespace std;

std::random_device rd;
std::mt19937 gen{rd()};

size_t iters = 0;

ros::Publisher pub;
MySerial my_Sp("/dev/ttyUSB0");

void transform_state(const vector<float> state, char *buf)
{
    int buf_len = 4 * state.size(); //4 * 4 + 3 = 19
    for (int i = 0; i < state.size(); i++)
    {
        *(float *)(&buf[4 * i]) = state[i];
    }
}

void receive_from_char(const char *buf, vector<float> &control_output)
{
    for (int i = 0; i < control_output.size(); i++)
    {
        control_output[i] = *(float *)(&buf[4 * i]);
    }
}
template <typename T>
void print_vector(const vector<T> vec)
{
    cout << "[";
    for (int i = 0; i < vec.size(); i++)
    {
        cout << vec[i];
        if (i != vec.size() - 1)
        {
            cout << ",";
        }
        else
        {
            cout << "]" << endl;
        }
    }
}

void produce_test_case(vector<float> &output_vector, bool is_shown)
{
    std::normal_distribution<> distribution_x(0, 1);
    for (int i = 0; i < output_vector.size(); i++)
    {
        output_vector[i] = (float)(distribution_x(gen));
    }
    float K_CON[4] = {0.77, 1, 6.3, 1};
    float correct_output = 0;
    for (int i = 0; i < 4; i++)
    {
        correct_output += K_CON[i] * output_vector[i];
    }
    if (is_shown)
    {
        cout << "original vector:" << endl;
        print_vector(output_vector);
        cout << "correct control output:" << endl;
        cout << correct_output << endl;
    }
}


void get_state(const inverted_pendulum::pendulum::ConstPtr &msg)
{
    vector<float> state(4);
    vector<float> output_vector(1);
    char buf[16];

    state[0] = float(msg->x_position), state[1] = float(msg->x_dot);
    state[2] = float(msg->theta), state[3] = float(msg->theta_dot);

    transform_state(state, buf);
    my_Sp.flush();
    my_Sp.write_chars(buf, 16);
    my_Sp.read_from_serial();
    int result = my_Sp.call_handle(0.2);
    if (result > 0)
    {
        receive_from_char(my_Sp.my_buffer, output_vector);
        std_msgs::Float64 temp;
        temp.data = output_vector[0];
        pub.publish(temp);
    }
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "learn_serial");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/pendulum_output", 1, get_state);

    pub = n.advertise<std_msgs::Float64>("/control_input", 1);

    vector<float> state(4);

    ros::spin();

}

