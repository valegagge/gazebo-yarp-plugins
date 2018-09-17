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


    bool GazeboYarpDoubleLaser::readConfigurationFromFile(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!_sdf->HasElement("yarpConfigurationFile"))
        {
            yError() << "GazeboYarpDoubleLaser: error: unabble to load configuration";
            return false;
        }

        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path == "")
        {
            yError() << "GazeboYarpDoubleLaser: ini file path is empty";
            return false;
        }

        GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

        bool wipe = false; //in order to not clear m_parameters
        if (! m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
        {
            yError()  << "GazeboYarpDoubleLaser: error reading parameters from config file= " << ini_file_name << "in" <<ini_file_path ;
            return false;
        }

        m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

        return true;
    }



    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpDoubleLaser::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // 1) check params
        if (!_parent)
        {
            gzerr << "GazeboYarpDoubleLaser plugin requires a parent.\n";
            return;
        }

        //2) check yarp network
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpDoubleLaser : yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        m_sensorName = _parent->GetScopedName();
        
        //3) load configuration from sdf
        if(!readConfigurationFromFile( _parent, _sdf))
            return;

        // 4) Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));


        // 5) Add the gazebo_doubleLaser device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpDoubleLaserDriver>("gazebo_doubleLaser", "Rangefinder2DWrapper", "GazeboYarpDoubleLaserDriver"));


        //TODO: I removed the parser of  [ROS] group. Do I need it?


        //6) open wrapper Rangefinder2DWrapper
        if(! m_wrapper_rangeFinder.open(m_parameters))
        {
            yError()<<"GazeboYarpDoubleLaser failed: error in opening yarp driver wrapper Rangefinder2DWrapper!!!";
            return;
        }
//         else
//         {
//             yError() << "GazeboYarpDoubleLaser : ho aperto il wrapper Rangefinder2DWrapper ";
//
//         }

        if (!m_wrapper_rangeFinder.view(m_iWrap_rangeFinder)) 
        {
            yError("GazeboYarpDoubleLaser : wrapper interface not found, load failed. Rangefinder2DWrapper");
            return;
        }
//         else
//         {
//              yError() << "GazeboYarpDoubleLaser : ho ho fatto la view di Rangefinder2DWrapper ";
//         }

         //7) look for the laser front and back in in yarp drivers set
        yarp::dev::PolyDriverDescriptor laser1, laser2;
        std::string scopedDeviceName_laser1 = "base_laser";
        std::string scopedDeviceName_laser2 = "base_laser2";
//         laser1.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName_laser1);
//         if(laser1.poly == nullptr)
//         {
//             yError()<<"GazeboYarpDoubleLaser: non trovo laser1";
//             return;
//         }
//         laser2.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName_laser2);
//         if(laser2.poly == nullptr)
//         {
//             yError()<<"GazeboYarpDoubleLaser: non trovo laser2";
//             return;
//         }
//
//
//          yError()<<"GazeboYarpDoubleLaser: ho trovato tutti e due i laser!! OK";
//
//          //GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor", laser1.poly); sono gia stati aggiunti ..credo..
//          m_lasers.push(laser1);
//
//          //GazeboYarpPlugins::Handler::getHandler()->setDevice("laser_sensor2", laser2.poly); sono gia stati aggiunti ..credo..
//          m_lasers.push(laser2);
        
        
        
        // 8) Open the driver DoubleLaser
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
        

//         if (!m_driver_doublelaser.view(m_iWrap_doublelaser))
//         {
//             yError("GazeboYarpDoubleLaser : doublelaser device's wrapper interface not found, load failed.");
//             return;
//         }

         yError("GazeboYarpDoubleLaser : ho fatto la view doublelaser.");

//          if(m_iWrap_doublelaser->attachAll(m_lasers))
//          {
//              yError("GazeboYarpDoubleLaser : ho fatto attach a tutti i laserOK.");
//          }
//          else
//          {
//              yError("GazeboYarpDoubleLaser : ERRORE mentre facevo attach a tutti i laser.");
//              return;
//          }




        
        // 9 )attach rangefinder wrapper to double laser
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
        
         
         
         
      //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

     }

}
