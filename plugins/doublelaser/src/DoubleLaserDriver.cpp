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

    if(!config.check("laserFront"))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laser1 parameter";
        return false;
    }

    if(!config.check("laserBack"))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laser2 parameter";
        return false;
    }
    
    
    std::string laserFront_name= config.find("laserFront").asString();
    std::string laserBack_name= config.find("laserBack").asString();
    yError() << "GazeboYarpDoubleLaserDriver: laserFront=" <<  laserFront_name <<" laserBack=" << laserBack_name ;
    
    
    m_driver_laserFront = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserFront_name);
    if(m_driver_laserFront == nullptr)
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laserFront device";
        return false;
    }
        else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: finded laserFront device. OK";
    }

    
    if(!m_driver_laserFront->view(m_dev_laserFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser front";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser front. OK";
    }
    


    m_driver_laserBack = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserBack_name);
    if(m_driver_laserBack == nullptr)
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laserBack device";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: finded laserBack device. OK";
    }
    
    if(!m_driver_laserBack->view(m_dev_laserBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser Back";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser Back. OK";
    }
    
    
    
//     //MI SERVER IL PARENT SENSOR???
//     m_parentSensor = dynamic_cast<gazebo::sensors::RaySensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
//     m_started = true;
//     
//     if (!m_parentSensor)
//     {
//         yError() << "Error, sensor" <<  sensorScopedName << "was not found" ;
//         return  false ;
//     }
//     
    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpDoubleLaserDriver::onUpdate, this, _1));
    
    //DEVO AGGIUNGERLO???
    // Connect the onReset method to the WorldReset event callback
    this->m_resetConnection =
    gazebo::event::Events::ConnectWorldReset(boost::bind(&GazeboYarpDoubleLaserDriver::onReset, this));
    
    //AGGIUNGI QUI ROBA PER LEGGERE LA CONFIGURAZIONE DEI LASER!
    yError() << "****GazeboYarpDoubleLaserDriver: ora dovrei leggere la configurazione!!!! OK";
    
    double minFront, maxFront, minBack, maxBack;
    if(!m_dev_laserFront->getScanLimits(minFront, maxFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting scan limits for front laser";
        return false;
    }

    if(!m_dev_laserBack->getScanLimits(minBack, maxBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting scan limits for back laser";
        return false;
    }
    
    if( (minFront != minBack) || (maxFront != maxBack) )
    {
        yError() << "GazeboYarpDoubleLaserDriver: front and back laser differ in scan limits";
        return false;
    }
    
    double resolutionFront, resolutionBack;
    if(!m_dev_laserFront->getHorizontalResolution(resolutionFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting resolution for front laser";
        return false;
    }

    if(!m_dev_laserBack->getHorizontalResolution(resolutionBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting resolution for back laser";
        return false;
    }
    
    if(resolutionFront != resolutionBack) 
    {
        yError() << "GazeboYarpDoubleLaserDriver: front and back laser differ in resolution";
        return false;
    }

    //here I'm sure front and back have same config
    m_samples = (maxFront -minFront) /resolutionFront;
    
    m_resolution = resolutionFront;
    //TODO: aggiungi qui le verifiche di uaguale configurazione dei due laser.
    //Per ora, nelle funzioni get ritorno le info del front
    
    yError() << "****GazeboYarpDoubleLaserDriver: samples=" << m_samples;
    return true;
}

bool GazeboYarpDoubleLaserDriver::close()
{
    this->m_updateConnection.reset();
    return true;
}



bool GazeboYarpDoubleLaserDriver::getDistanceRange(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    return m_dev_laserFront->getDistanceRange(min, max);
}

bool GazeboYarpDoubleLaserDriver::setDistanceRange(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setDistanceRange not yet implemented");
    return false;
}

bool GazeboYarpDoubleLaserDriver::getScanLimits(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    return m_dev_laserFront->getScanLimits(min, max);
    return true;
}

bool GazeboYarpDoubleLaserDriver::setScanLimits(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setScanLimits not yet implemented");
    return false;
}

bool GazeboYarpDoubleLaserDriver::getHorizontalResolution(double& step)
{
    yarp::os::LockGuard guard(m_mutex);
    return m_dev_laserFront->getHorizontalResolution(step);
}

bool GazeboYarpDoubleLaserDriver::setHorizontalResolution(double step)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setHorizontalResolution not yet implemented");
    return false;
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
    yarp::sig::Vector dataFront;
    yarp::sig::Vector dataBack;
    if(out.size() != m_samples)
        out.resize(m_samples);
    
    if(!m_dev_laserFront->getRawData(dataFront))
        return false;
    if(!m_dev_laserBack->getRawData(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
        if(dataFront[i]!= INFINITY)
            out[i] = dataFront[i];
        else if(dataBack[i] != INFINITY)
            out[i] = dataBack[i];
        else
            out[i] = INFINITY;
    }
    
    return true;
}

bool GazeboYarpDoubleLaserDriver::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
    yarp::os::LockGuard guard(m_mutex);
    
    std::vector<LaserMeasurementData> dataFront;
    std::vector<LaserMeasurementData> dataBack;
    if(data.size() != m_samples)
        data.resize(m_samples);
    
    if(!m_dev_laserFront->getLaserMeasurement(dataFront))
        return false;
    if(!m_dev_laserBack->getLaserMeasurement(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
        double rhoFront, thetaFront, rhoBack, thetaBack;
        dataFront[i].get_polar(rhoFront, thetaFront);
        dataBack[i].get_polar(rhoBack, thetaBack);
        
        if(rhoFront!= INFINITY)
            data[i].set_polar(rhoFront, thetaFront);
        else  if(rhoBack != INFINITY)
                data[i].set_polar(rhoBack, thetaBack);
        else
            data[i].set_polar(INFINITY, i*m_resolution);
    }

    return true;
}
bool GazeboYarpDoubleLaserDriver::getDeviceStatus(Device_status &status)
{
    yarp::os::LockGuard guard(m_mutex);
    Device_status statusFront, statusBack;
    if(! m_dev_laserFront->getDeviceStatus(statusFront))
        return false;
    if(! m_dev_laserBack->getDeviceStatus(statusBack))
        return false;
    if(statusFront == statusBack)
        status = statusFront;
    else
    {
        //TODO
        yError() << "GazeboYarpDoubleLaserDriver: the status of laser front (" << statusFront << ") differs from the status of laser back (" << statusBack << ")";
        return false;
    }

    return true;
}

bool GazeboYarpDoubleLaserDriver::getDeviceInfo (std::string &device_info)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

