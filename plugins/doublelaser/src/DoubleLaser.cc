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

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpDoubleLaser)

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

        
        
        m_sensorName = _parent->GetScopedName();
        
        yError() << "GazeboYarpDoubleLaser : my name is " << m_sensorName;

        //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));


        // Add the gazebo_doubleLaser device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpDoubleLaserDriver>("gazebo_doubleLaser", "Rangefinder2DWrapper", "GazeboYarpDoubleLaserDriver"));

        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;
//-------------------------------------------------------------------------
//          ::yarp::os::Property wrapper_properties;
//          ::yarp::os::Property driver_properties;
//         
//         configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,driver_properties);
//         wrapper_properties = driver_properties;
// 
// 
//         driver_properties.put(YarpLaserSensorScopedName.c_str(), m_sensorName.c_str());
//         
//         //Open the wrapper
//         wrapper_properties.put("device","Rangefinder2DWrapper");         
//---------------------------------------------------        
        yarp::os::Bottle wrapper_group;
        yarp::os::Bottle driver_group;
        if (_sdf->HasElement("yarpConfigurationFile")) 
        {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

            yError() << "GazeboYarpDoubleLaser::Load() ini file path=" << ini_file_path;
            yError() << "GazeboYarpDoubleLaser::Load() ini_file_name=" << ini_file_name;    
            
            GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

            bool wipe = false;
            if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
            {
                m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

//                 wrapper_group = m_parameters.findGroup("WRAPPER");
//                 if(wrapper_group.isNull()) 
//                 {
//                     yError("GazeboYarpDoubleLaser : [WRAPPER] group not found in config file\n");
//                     return;
//                 }

                //ho tolto [ROS]
                configuration_loaded = true;
                yError() << "GazeboYarpDoubleLaser::ho caricato la cfg del wrapper rangefinder=" <<  wrapper_group.toString();
            }
            else
            {
                yError() << "GazeboYarpDoubleLaser::Load error: ini file path=" << ini_file_path;
                return;
            }

        }
         if (!configuration_loaded)
        {
            yError() << "GazeboYarpDoubleLaser::Load error: unabble to load configuration";
            return;
        };
        
        
        yError() << "GazeboYarpDoubleLaser PARAMETERS :" << m_parameters.toString();
        //apro il wrapper Rangefinder2DWrapper
//         yarp::os::Bottle ros_group = m_parameters.findGroup("ROS");
//         if(!ros_group.isNull())
//             wrapper_group.append(ros_group);
//         else
//         { 
//              yError() << "GazeboYarpDoubleLaser WRAPPERBOTTLE : non trovo ROS";
//         }
//         
//          yError() << "GazeboYarpDoubleLaser WRAPPERBOTTLE :" << wrapper_group.toString();
        
        //if( m_wrapper_rangeFinder.open(wrapper_properties) ) 
        if( m_wrapper_rangeFinder.open(m_parameters/*wrapper_group*/) )  
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
        else
        {
             yError() << "GazeboYarpDoubleLaser : ho ho fatto la view di Rangefinder2DWrapper ";
        }
        
        
        
         //cerco i due sensori laser tra i driver di yarp
        yarp::dev::PolyDriverDescriptor laser1, laser2;
        std::string scopedDeviceName_laser1 = "base_laser";//default::SIM_CER_ROBOT::mobile_base_body_link::base_laser"; //m_sensorName + "::" + "base_laser";
        std::string scopedDeviceName_laser2 = "base_laser2"; //default::SIM_CER_ROBOT::mobile_base_body_link::base_laser2";
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
         
         //GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor", laser1.poly); sono gia stati aggiunti ..credo..
         m_lasers.push(laser1);
         
         //GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor2", laser2.poly); sono gia stati aggiunti ..credo..
         m_lasers.push(laser2);
        
        
        
        //Open the driver DoubleLaser
         yError() << "GazeboYarpDoubleLaser : sto per aprire il device doublelaser..... ";
        ::yarp::os::Property driver_properties;
        driver_properties.put("device","gazebo_doubleLaser");
        driver_properties.put("laserFront",scopedDeviceName_laser1);
        driver_properties.put("laserBack",scopedDeviceName_laser2);
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
        
        if(m_iWrap_rangeFinder->attachAll(listofdoubellaser))
        {
            yError() << "GazeboYarpDoubleLaser: ho fatto attach di rangeFinder wrapper al double laser. OK";
        }
        else
        {
            yError() << "GazeboYarpDoubleLaser: ERRORE mentre facevo attach di rangeFinder wrapper al double laser.";
        }
        
         
         
         
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
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

     }

}
