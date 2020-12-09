/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANDYNAMICSESTIMATOR
#define HDE_DEVICES_HUMANDYNAMICSESTIMATOR

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/visualization_msgs/Marker.h>

#include <iDynTree/KinDynComputations.h>

#include "IHumanWrench.h"
#include "IHumanDynamics.h"

#include <memory>

namespace hde {
    namespace devices {
        class HumanDynamicsEstimator;
    } // namespace devices
} // namespace hde

class hde::devices::HumanDynamicsEstimator final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public yarp::dev::IAnalogSensor
    , public hde::interfaces::IHumanWrench
    , public hde::interfaces::IHumanDynamics
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    yarp::os::Node rosNode;
    yarp::rosmsg::visualization_msgs::Marker estimatedObjectMassMsg;
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::Marker> estimatedObjectMassMarkerMsgPub;

public:
    HumanDynamicsEstimator();
    ~HumanDynamicsEstimator() override;

    // Wrench smoothing
    void wrenchSmoothing(std::vector<double>& inputWrench, std::vector<double>& outputWrench);

    // Method to express wrenches in different frame
    void expressWrenchInDifferentFrames(const std::vector<double>& wrenchInLinkFrame,
                                        hde::interfaces::IHumanWrench::TaskType taskType,
                                        hde::interfaces::IHumanWrench::WrenchType wrenchType,
                                        iDynTree::KinDynComputations& kinDyn);
    // Publish ros marker message
    void publishRosMarkerMsg(const double& mass);

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // IAnalogSensor interface
    int read(yarp::sig::Vector& out) override;
    int getState(int ch) override;
    int getChannels() override;
    int calibrateSensor() override;
    int calibrateSensor(const yarp::sig::Vector& value) override;
    int calibrateChannel(int ch) override;
    int calibrateChannel(int ch, double value) override;

    // IHumanWrench
    std::vector<std::pair<std::string, WrenchSourceType>> getWrenchSourceNameAndType() const override;
    std::vector<std::string> getWrenchSourceNames() const override;
    size_t getNumberOfWrenchSources() const override;
    std::vector<double> getWrenches() const override;
    std::vector<double> getWrenchesInFrame(TaskType, WrenchType, WrenchReferenceFrame) const override;

    // IHumanDynamics
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::vector<double> getJointTorques() const override;
    std::vector<std::array<double,6>> getInternalWrenches() const override;
};

#endif // HDE_DEVICES_HUMANDYNAMICSESTIMATOR
