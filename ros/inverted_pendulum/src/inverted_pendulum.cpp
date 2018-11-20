#include "ros/ros.h"
#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <inverted_pendulum/pendulum.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace boost::numeric::odeint;

const double M = 0.01;
const double m = 0.109;
const double L = 0.25;
const double g = 9.8;


typedef boost ::array<double, 4> state_type;
state_type x_current = {{ 0.1,0 , 3.14/12 , 0 }}; // initial conditions
//state_type x_current = {{ 0.8 ,0 , 0 , 0 }}; // initial conditions
double control_input = 0;
bool is_controller_connected = false;
void get_control(const std_msgs::Float64::ConstPtr &msg)
{
    control_input = msg->data;
    is_controller_connected = true;
}
void inv_pen(const state_type &x, state_type &dxdt, double t)
{
    double x0 = x[0];
    double x0_dot = x[1];
    double theta = x[2];
    double theta_dot = x[3];

    dxdt[0] = x0_dot;
    dxdt[2] = theta_dot;

    dxdt[1] = (-m * g * cos(theta) * sin(theta) +
               theta_dot * theta_dot * m * L * sin(theta) + control_input) /
              (M + m * sin(theta) * sin(theta));
    dxdt[3] = ((M + m) * g * sin(theta) -
               theta_dot * theta_dot * m * L * sin(theta) * cos(theta) -
               control_input * cos(theta)) /
              (M + m * sin(theta) * sin(theta)) / L;
    cout<<dxdt[3]<<endl;
    if (! is_controller_connected)
    {
        for(int i = 0; i<4 ;i++)
        {
            dxdt[i] = 0;
        }
    }
}
void output(const state_type &x , const double t)
{
    for(int i=0;i<4;i++)
    {
        x_current[i] = x[i];
    }
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"pendulum_model");
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub = n.subscribe("control_input",1,get_control);
    pub = n.advertise<inverted_pendulum::pendulum>("pendulum_output",1);
    
    double t0 = 0.0,dt = 0.005;
    ros::Rate loop_rate(100); 
    while(ros::ok())
    {
        ros::spinOnce();
        double end_time = t0 + dt;
        integrate( inv_pen , x_current , t0 , end_time , 0.001 , output );
        inverted_pendulum::pendulum msg;
        msg.x_position = x_current[0];
	    msg.x_dot = x_current[1];
        msg.theta = x_current[2];
	    msg.theta_dot = x_current[3];
        pub.publish(msg);
        t0 = end_time;
        loop_rate.sleep();
    }
        
    return 0;
}
