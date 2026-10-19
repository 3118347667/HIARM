#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    std::string odomFrame;
    std::vector<double> boxMin;
    std::vector<double> boxMax;
    int round;
    double diagWidth;
    double sideWidth;
    double maxAccRate;
    double maxVelRate;
    double weightT;
    std::vector<double> chiVec;
    double relCostTol;
    bool c2Diffeo;
    int headingMode;

    static void loadParameters(Config &conf, const ros::NodeHandle &nh_priv)
    {
        nh_priv.param("OdomFrame", conf.odomFrame, std::string("world"));
        nh_priv.param("BoxMin", conf.boxMin, {-3.0, -2.0, 1.0});
        nh_priv.param("BoxMax", conf.boxMax, {3.0, 2.0, 1.2});
        nh_priv.param("Round", conf.round, 1);
        nh_priv.param("DiagWidth", conf.diagWidth, 1.0);
        nh_priv.param("SideWidth", conf.sideWidth, 1.0);
        nh_priv.param("MaxAccRate", conf.maxAccRate, 2.0);
        nh_priv.param("MaxVelRate", conf.maxVelRate, 1.0);
        nh_priv.param("WeightT", conf.weightT, 1000.0);
        nh_priv.param("ChiVec", conf.chiVec, {1024.0, 1024.0, 1024.0});
        nh_priv.param("RelCostTol", conf.relCostTol, 1.0e-4);
        nh_priv.param("C2Diffeo", conf.c2Diffeo, true);
        nh_priv.param("HeadingMode", conf.headingMode, 0);
    }
};

#endif