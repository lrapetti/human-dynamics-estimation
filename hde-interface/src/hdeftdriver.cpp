/*
 * Copyright (c) 2018, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "hdeftdriver.h"

using namespace yarp::dev;

const unsigned ForceTorqueChannelsNumber = 6;

HDEFTDriver::HDEFTDriver()
{

}

bool HDEFTDriver::open(yarp::os::Searchable& config)
{
    number_of_sensors = config.find("number_of_sensors").asInt();
    number_of_channels = number_of_sensors*ForceTorqueChannelsNumber;
    
    force_torque_vector.resize(number_of_channels);
    
    yarp::os::Bottle frames = config.findGroup("FT_FRAMES");
    if(!frames.check("FT_FRAMES"))
    {
        yError() << "HDEFTDriver: Failed to get FT FRAMES";
        return false;
    }
    else
    {
        ft_frame_names.resize(number_of_sensors);
        for(int i=0; i < number_of_sensors; i++)
        {
            ft_frame_names.at(i) = frames.get(i+1).asString();
        }
    }
    
    return true;
}

int HDEFTDriver::read(yarp::sig::Vector& out)
{
    if(force_torque_vector.size() != number_of_channels)
    {
        yError() << "HDEFTDriver: ForceTorqueVector size is not the same as the total number of channels";
        return AS_ERROR;
    }
    
    if(out.size() != number_of_channels)
    {
        out.resize(number_of_channels);
    }
    
    data_mutex.wait();
    out = force_torque_vector;
    data_mutex.post();
    
    return AS_OK;
}

bool HDEFTDriver::close()
{
    return yarp::dev::DeviceDriver::close();
}

int HDEFTDriver::getChannels()
{
    return number_of_channels;
}

int HDEFTDriver::getState(int)
{
    return AS_OK;
}

int HDEFTDriver::calibrateSensor()
{
    return AS_OK;
}

int HDEFTDriver::calibrateSensor(const yarp::sig::Vector&)
{
    return AS_OK;
}

int HDEFTDriver::calibrateChannel(int)
{
    return AS_OK;
}

int HDEFTDriver::calibrateChannel(int, double)
{
    return AS_OK;
}

int HDEFTDriver::getTotalSensorsSize()
{
    return number_of_sensors;
};
    
std::string HDEFTDriver::getFTFrameName(int& i)
{ 
    return ft_frame_names.at(i);
};
    
void HDEFTDriver::setFTValues(double u,int pos)
{
    force_torque_vector[pos] = u;
};

HDEFTDriver::~HDEFTDriver()
{

}

