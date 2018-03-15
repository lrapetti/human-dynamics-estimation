#ifndef HUMANROBOTPOSE_H
#define HUMANROBOTPOSE_H

#include <memory>
#include <yarp/os/RFModule.h>

class HumanRobotPose : public yarp::os::RFModule
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanRobotPose();
    virtual ~HumanRobotPose();

    double getPeriod() override;
    bool configure(yarp::os::ResourceFinder& rf) override;
    bool updateModule() override;
    bool close() override;
};

#endif
