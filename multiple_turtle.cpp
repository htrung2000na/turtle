#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std;

ros::Publisher pub;
const float PI = 3.14159265;
float rate = 5;
turtlesim::Pose current_pose;
geometry_msgs::Twist msg;
const double tolerance = 1e-2;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

class PoseCallback {
public:
    int turtle_idx;
    ros::Subscriber sub;
    turtlesim::Pose current_pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        cout << "turtle " << turtle_idx+1 << ": x = " << left << setw(10) << msg->x << " | y = " << setw(10) << msg->y << " | theta = " << setw(10) << msg->theta << endl;
        current_pose = *msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    ros::Publisher pub[10];
    PoseCallback sub[10];
    ros::Rate loopRate(rate);

    int n_turtle = atoi(argv[1]);
    cout << "n_turtle = " << n_turtle << endl;

    double x0[n_turtle], y0[n_turtle];

    for (int i = 0; i < n_turtle; i++)
    {
        stringstream s;
        s << "turtle" << i+1;
        string name = s.str();

        pub[i] = h.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);
        sub[i].turtle_idx = i;
        sub[i].sub = h.subscribe(name+"/pose", 1000, &PoseCallback::callback, &sub[i]);
        cout << "subcribe turtle " << i+1 << " to " << name << "/pose" << endl;
    }

    for (int i = 2; i < argc; i+=2*n_turtle)
    {
        if (i + 2*n_turtle >= argc)
        {
            n_turtle = (argc - i)/2;
        }

        for (int f = 0; f < n_turtle; f++)
        {

                x0[f] = atof(argv[i+2*f]);
                y0[f] = atof(argv[i+2*f+1]);
        }

        int r = n_turtle;
        int goal[n_turtle];
        for (int f = 0; f < n_turtle; f++)
            goal[f] = 1;

            while (ros::ok())
            {

                loopRate.sleep();
                ros::spinOnce();

                for (int idx = 0; idx < n_turtle; idx++)
                {
                    if (goal[idx] == 1)
                    {

                    if (sub[idx].current_pose.theta > PI) sub[idx].current_pose.theta = sub[idx].current_pose.theta - 2*PI;
                    if (sub[idx].current_pose.theta < -PI) sub[idx].current_pose.theta = sub[idx].current_pose.theta + 2*PI;

                    double distance = sqrt( pow(x0[idx]-sub[idx].current_pose.x, 2) + pow(y0[idx]-sub[idx].current_pose.y, 2) );
                    if (distance < tolerance)
                    {
                            pub[idx].publish(getMessage(0,0));
                            r--;
                            goal[idx] = 0;
                    }

                    double dx = x0[idx] - sub[idx].current_pose.x, dy = y0[idx] - sub[idx].current_pose.y, theta = sub[idx].current_pose.theta;

                    double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);

                    if (sub[idx].current_pose.x > x0[idx])
                    {
                        if (sub[idx].current_pose.theta > -PI/2 && sub[idx].current_pose.theta < PI/2)
                            msg = getMessage(
                            -min(5*distance, 5.0),
                            -6*dalpha
                            );
                        else msg = getMessage(
                            min(5*distance, 5.0),
                            6*dalpha
                            );
                    }
                    else if(sub[idx].current_pose.x < x0[idx])
                    {
                        if (sub[idx].current_pose.theta > PI/2 || sub[idx].current_pose.theta < -PI/2)
                            msg = getMessage(
                            -min(5*distance, 5.0),
                            -6*dalpha
                            );
                        else msg = getMessage(
                            min(5*distance, 5.0),
                            6*dalpha
                            );

                    }
                    else if (sub[idx].current_pose.x <= x0[idx] && sub[idx].current_pose.x >= x0[idx] )
                    {
                        if (sub[idx].current_pose.y >= y0[idx])
                        {
                            if (sub[idx].current_pose.theta >= 0)
                                msg = getMessage(
                                -min(5*distance, 5.0),
                                -6*dalpha
                                );
                            else msg = getMessage(
                                min(5*distance, 5.0),
                                6*dalpha
                                );
                        }
                        else
                        {
                            if (sub[idx].current_pose.theta < 0)
                                msg = getMessage(
                                -min(5*distance, 5.0),
                                -6*dalpha
                                );
                            else msg = getMessage(
                                min(5*distance, 5.0),
                                6*dalpha
                                );
                        }
                    }
                    pub[idx].publish(msg);
                    }
                }
                if (r == 0) break;
            }
    }


    return 0;
}
