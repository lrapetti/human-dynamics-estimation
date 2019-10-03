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
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include "IHumanDynamics.h"

#include <memory>
#ifdef ENABLE_LOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

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
    , public hde::interfaces::IHumanDynamics
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    HumanDynamicsEstimator();
    ~HumanDynamicsEstimator() override;

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

    // IHumanDynamics
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::vector<double> getJointTorques() const override;
};

#ifdef ENABLE_LOGGER
inline std::string getTimeDateMatExtension()
{
    // this code snippet is taken from
    // https://stackoverflow.com/questions/17223096/outputting-date-and-time-in-c-using-stdchrono
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string timedate;
    std::strftime(&timedate[0], timedate.size(), "%Y-%m-%d%H:%M:%Slog.mat", std::localtime(&now));
    return timedate;
}
#endif

#endif // HDE_DEVICES_HUMANDYNAMICSESTIMATOR
