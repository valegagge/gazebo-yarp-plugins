/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <cstdio>
#include "DoubleLaserDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Events.hh>


#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


GazeboYarpDoubleLaserDriver::GazeboYarpDoubleLaserDriver() : m_deviceName("") 
{
    yWarning () <<"....GazeboYarpDoubleLaserDriver::sono nel costruttore...";
}

GazeboYarpDoubleLaserDriver::~GazeboYarpDoubleLaserDriver() {}


void GazeboYarpDoubleLaserDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
   yWarning () <<"....GazeboYarpDoubleLaserDriver::onUpdate...";
}

void GazeboYarpDoubleLaserDriver::onReset()
{
}


bool GazeboYarpDoubleLaserDriver::attachAll(const PolyDriverList &p)
{;}
bool GazeboYarpDoubleLaserDriver::detachAll()
{;}

bool GazeboYarpDoubleLaserDriver::open(yarp::os::Searchable& config)
{
    yarp::os::LockGuard guard(m_mutex);

    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpLaserSensorScopedName.c_str()).asString().c_str());

    yError() << "GazeboYarpDoubleLaserDriver: sto cercando il sensore chiamato" <<  sensorScopedName ;
    
    m_parentSensor = dynamic_cast<gazebo::sensors::RaySensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
    m_started = true;
    
    if (!m_parentSensor)
    {
        yError() << "Error, sensor" <<  sensorScopedName << "was not found" ;
        return  false ;
    }
    
    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpDoubleLaserDriver::onUpdate, this, _1));
    
    //DEVO AGGIUNGERLO???
    // Connect the onReset method to the WorldReset event callback
    this->m_resetConnection =
    gazebo::event::Events::ConnectWorldReset(boost::bind(&GazeboYarpDoubleLaserDriver::onReset, this));
    
    //AGGIUNGI QUI ROBA PER LEGGERE LA CONFIGURAZIONE DEI LASER!

}

bool GazeboYarpDoubleLaserDriver::close()
{
    this->m_updateConnection.reset();
    return true;
}



bool GazeboYarpDoubleLaserDriver::getDistanceRange(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::setDistanceRange(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::getScanLimits(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::setScanLimits(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setScanLimits not yet implemented");
    return true;
}

bool GazeboYarpDoubleLaserDriver::getHorizontalResolution(double& step)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::setHorizontalResolution(double step)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::getScanRate(double& rate)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("getScanRate not yet implemented");
    return true;
}

bool GazeboYarpDoubleLaserDriver::setScanRate(double rate)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setScanRate not yet implemented");
    return false;
}


bool GazeboYarpDoubleLaserDriver::getRawData(yarp::sig::Vector &out)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}
bool GazeboYarpDoubleLaserDriver::getDeviceStatus(Device_status &status)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

bool GazeboYarpDoubleLaserDriver::getDeviceInfo (std::string &device_info)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

