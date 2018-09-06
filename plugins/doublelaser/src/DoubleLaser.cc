/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "DoubleLaser.hh"
#include "DoubleLaserDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

//#include <gazebo/sensors/RaySensor.hh>

#include <gazebo/physics/Model.hh>

#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


using namespace std;
namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(GazeboYarpDoubleLaser)

    GazeboYarpDoubleLaser::GazeboYarpDoubleLaser() : m_iWrap_rangeFinder(0), m_iWrap_doublelaser(0)
    {}

    GazeboYarpDoubleLaser::~GazeboYarpDoubleLaser()
    {
        if (m_iWrap_doublelaser) 
        {
            m_iWrap_doublelaser->detachAll();
            m_iWrap_rangeFinder = 0;
        }
        
        if (m_iWrap_rangeFinder) 
        {
            m_iWrap_rangeFinder->detachAll();
            m_iWrap_rangeFinder = 0;
        }

        if (m_wrapper_rangeFinder.isValid()) 
        {
            m_wrapper_rangeFinder.close();
        }


        for (int n = 0; n < m_lasers.size(); n++) {
            std::string scopedDeviceName = m_sensorName + "::" + m_lasers[n]->key.c_str();
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
        }

        GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
        yarp::os::Network::fini();
    }

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpDoubleLaser::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yError() << "GazeboYarpDoubleLaser : sono nella load";
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            yError() << "GazeboYarpDoubleLaser : yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        if (!_parent) {
            gzerr << "GazeboYarpDoubleLaser plugin requires a parent.\n";
            return;
        }

        
        
        m_sensorName = _parent->ScopedName();
        
        yError() << "GazeboYarpDoubleLaser : my name is " << m_sensorName;

        //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());


        // Add the gazebo_controlboard device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpDoubleLaserDriver>("gazebo_doubleLaser", "Rangefinder2DWrapper", "GazeboYarpDoubleLaserDriver"));

        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;

        ::yarp::os::Property wrapper_properties;
        ::yarp::os::Property driver_properties;
        
        configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,driver_properties);
        
        
         if (!configuration_loaded)
        {
            yError() << "GazeboYarpDoubleLaser::Load error: unabble to load configuration";
            return;
        };

        wrapper_properties = driver_properties;


        driver_properties.put(YarpLaserSensorScopedName.c_str(), m_sensorName.c_str());
        
        //Open the wrapper
        wrapper_properties.put("device","Rangefinder2DWrapper");
        if( m_wrapper_rangeFinder.open(wrapper_properties) ) 
        {
            yError() << "GazeboYarpDoubleLaser : ho aperto il wrapper Rangefinder2DWrapper ";
        } 
        else
        {
            yError()<<"GazeboYarpDoubleLaser failed: error in opening yarp driver wrapper Rangefinder2DWrapper!!!";
            return;
        }

        if (!m_wrapper_rangeFinder.view(m_iWrap_rangeFinder)) 
        {
            yError("GazeboYarpDoubleLaser : wrapper interface not found, load failed. Rangefinder2DWrapper");
            return;
        }
        
        //Open the driver DoubleLaser
        driver_properties.put("device","gazebo_doubleLaser");
        if( m_driver_doublelaser.open(driver_properties) ) 
        {
        
            yError() << "GazeboYarpDoubleLaser : ho aperto il driver doubleLaser ";
        }
        else 
        {
            yError()<<"GazeboYarpDoubleLaser Plugin failed: error in opening yarp driver doubleLaser";
            return;
        }
        
        
        //attach rangefinder wrapper to double laser
        yarp::dev::PolyDriverList listofdoubellaser; //it will contain only double laser
        yarp::dev::PolyDriverDescriptor doublelaser_desc;
        doublelaser_desc.poly = &m_driver_doublelaser;
        
        listofdoubellaser.push(doublelaser_desc);
        
        if(m_iWrap_doublelaser->attachAll(listofdoubellaser))
        {
            yError() << "GazeboYarpDoubleLaser: ho fatto attach di double laser a rangefinder wrapper. OK";
        }
        else
        {
            yError() << "GazeboYarpDoubleLaser: ERRORE mentre facevo attach di double laser a rangefinder wrapper.";
        }
        
        //cerco i due sensori laser
        yarp::dev::PolyDriverDescriptor laser1, laser2;
        std::string scopedDeviceName_laser1 = m_sensorName + "::" + "laser_sensor";
        std::string scopedDeviceName_laser2 = m_sensorName + "::" + "laser_sensor2";
        laser1.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName_laser1);
        if(laser1.poly == nullptr)
        {
            yError()<<"GazeboYarpDoubleLaser: non trovo laser1";
            return;
        }
        laser2.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName_laser2);
        if(laser2.poly == nullptr)
        {
            yError()<<"GazeboYarpDoubleLaser: non trovo laser2";
            return;
        }
        
        
         yError()<<"GazeboYarpDoubleLaser: ho trovato tutti e due i laser!! OK";
         
         GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor", laser1.poly);
         m_lasers.push(laser1);
         
         GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor2", laser2.poly);
         m_lasers.push(laser2);
         
         
         
         if (!m_driver_doublelaser.view(m_iWrap_doublelaser)) 
         {
            yError("GazeboYarpDoubleLaser : doublelaser device's wrapper interface not found, load failed.");
            return;
         }
         
         yError("GazeboYarpDoubleLaser : ho fatto la view doublelaser.");
         
         if(m_iWrap_doublelaser->attachAll(m_lasers))
         {
             yError("GazeboYarpDoubleLaser : ho fatto attach a tutti i laserOK.");
         }
         else
         {
             yError("GazeboYarpDoubleLaser : ERRORE mentre facevo attach a tutti i laser.");
        }
        //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

     }

}
