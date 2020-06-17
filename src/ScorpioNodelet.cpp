#include "scorpio_tester/scorpio_tester_nodelet.hpp"
#include "Configuration.h"
#include "Utils/IO/IOUtilities.hpp"
#include "Utils/Math/MathUtilities.hpp"
#include "Utils/General/Clock.hpp"
#include "ExternalSource/myYaml/include/myYaml/node/node.h"

namespace scorpio_nodelet
{
  ScorpioNodelet::~ScorpioNodelet()
  {
    m_spin_thread->join();
  }

  void ScorpioNodelet::onInit()
  {
    m_spin_thread.reset(new boost::thread(boost::bind(&ScorpioNodelet::spinThread, this)));
  }

  void ScorpioNodelet::spinThread()
  {
    m_sync.reset(new aptk::comm::Synchronizer(true, "scorpio_nodelet"));
    m_rt_nh.reset(new aptk::comm::RTNodeHandle());
    ros::NodeHandle nh("~");

    nh.param("/rt/controller_rt", m_controller_rt, true);
    nh.param("/rt/controller_core", m_controller_core, 3);
    nh.param("/rt/controller_priority", m_controller_priority, 5);
    if(m_controller_rt)
      aptk::comm::enableRT(m_controller_priority, m_controller_core);

    m_sync->connect();

    //initialize slaves & register MISO and MOSI
    m_slave_list = m_sync->getSlaveList();
    initialize();
    preProcess();

    //clear faults
    for(std::size_t i = 0; i < m_slave_list.size(); ++i)
    {
        m_sync->clearFaults(m_slave_list[i]);
    }

    //main control loop
    while(m_sync->ok())
    {
      m_sync->awaitNextControl(); //wait for bus transaction

      //copy MISO data to SensorData Class
      copyData();
      setCurrentPositionCmd();

      if (m_sync ->printIndicatedFaults()){

      } else {
            m_interface -> getCommand(m_sensor_data,m_command);
            m_copy_cmd_jpos_atv = m_command->q;
            m_copy_cmd_jvel_atv = m_command->qdot;
            m_copy_cmd_jtrq_atv = m_commnad->jtrq;
      }
      copyCommand();

      m_rt_nh->updateRT();
      m_sync->finishControl(); //indicate that we're done

      ++m_count;
    }
    m_sync->awaitShutdownComplete();
  }
}
    void ScorpioNodelet::setCurrentPositionCmd(){
        for (int i = 0; i < m_num_atv_j; ++i) {
            m_copy_cmd_jpos_atv[i] = m_copy_jpos[sensordata_idx_list[i]]; 
        }
       m_copy_cmd_jvel_atv.setZero();
       m_copy_cmd_jtrq_atv.setZero();
    }

    void ScorpioNodelet::buildActiveJointIdx(){
        atv_idx_list.resize(m_num_atv_j);
        atv_idx_list[0] = m_sync->getSlaveIdx("Yaw_1");
        atv_idx_list[1] = m_sync->getSlaveIdx("Elev_Inertia_1");
        atv_idx_list[2] = m_sync->getSlaveIdx("Yaw_2");
        atv_idx_list[3] = m_sync->getSlaveIdx("Elev_Inertia_2");
        atv_idx_list[4] = m_sync->getSlaveIdx("Wrist_Skewed");
        atv_idx_list[5] = m_sync->getSlaveIdx("Wrist_Pitch");
        atv_idx_list[6] = m_sync->getSlaveIdx("Wrist_Roll");
    }

    void ScorpioNodelet::buildPayloadJointIdx(){
        payload_idx_list.resize(2);
        payload_idx_list[0] = m_sync->getSlaveIdx("Elev_Payload_1");
        payload_idx_list[1] = m_sync->getSlaveIdx("Elev_Payload_2");
    }
    
    void ScorpioNodelet::initialize(){
        m_count = 0;
        m_num_tot_j = 11;
        m_num_atv_j = 7;
        sensordata_idx_list = {0,1,4,5,8,9,10};//exclude passive joint idx

        //Initialize raw MISO data
        m_raw_jpos = Eigen::VectorXf::Zero(m_num_tot_j);
        m_raw_jvel = Eigen::VectorXf::Zero(m_num_tot_j);
        m_raw_jtrq_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_raw_mot_curr_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_raw_mot_temp_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_raw_mom_arm_atv = Eigen::VectorXf::One(m_num_atv_j); //only inertia joint changes
        m_raw_ft_sensor = Eigen::Vector6f::Zero();


        //Initialize raw MOSI data
        m_raw_cmd_jpos_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_raw_cmd_jvel_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_raw_cmd_jtrq_atv = Eigen::VectorXf::Zero(m_num_atv_j);

        //payload 
        m_raw_mot_temp_pld = Eigen::VectorXf::Zero(2);
        m_raw_pld_frc = Eigen::VectorXf::Zero(2);
        m_raw_cmd_pld_frc = Eigen::VectorXf::Zero(2);

        //TODO:for payload, 
        //m_nh.param("/scorpio/elev1_pld_n", m_elev1_handler->manual_cmd_pld_frc, 94.0);
        //m_nh.param("/scorpio/elev2_pld_n", m_elev2_handler->manual_cmd_pld_frc, 50.0);
        
        //initialize cache 
        m_rotor_inertia_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_static_gr_atv = Eigen::VectorXf::Zero(m_num_atv_j);
        m_static_torque_limits_atv = Eigen::VectorXf::Zero(m_num_atv_j);

        //parameter setting for ll gain and sensor limits
        parameterSetting();

        //initialize PnC package interface class
        m_interface = new ScorpioInterface();
        m_sensor_data = new ScorpioSensorData();
        m_command = new ScorpioCommand();

        //initialize copy data
        m_copy_jpos = Eigen::VectorXd::Zero(m_num_tot_j);
        m_copy_jvel = Eigen::VectorXd::Zero(m_num_tot_j);
        m_copy_jtrq_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_copy_mot_curr_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_copy_mot_temp_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_copy_mom_arm_atv = Eigen::VectorXd::One(m_num_atv_j); //only inertia joint changes
        m_copy_ft_sensor = Eigen::Vector6d::Zero();

        m_copy_mot_temp_pld = Eigen::VectorXd::Zero(2);
        m_copy_pld_frc = Eigen::VectorXd::Zero(2);

        //initialize copy command
        m_copy_cmd_jpos_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_copy_cmd_jvel_atv = Eigen::VectorXd::Zero(m_num_atv_j);
        m_copy_cmd_jtrq_atv = Eigen::VectorXd::Zero(m_num_atv_j);

        m_copy_cmd_pld_frc = Eigen::VectorXd::Zero(2);

        try {
            YAML::Node ll_config = 
                YAML::LoadFile(THIS_COM "Config/Scorpio/LOW_LEVEL_CONFIG.yaml");
            YAML::Node service_call_cfg = ll_config["safety_turn_off"];
            myUtils::readParameter(service_call_cfg, "max_temperature", maxTemperature);
            myUtils::readParameter(service_call_cfg, "max_position", maxJointPosition);
            myUtils::readParameter(service_call_cfg, "min_position", minJointPosition);
            myUtils::readParameter(service_call_cfg, "max_velocity", maxVelocity);
            myUtils::readParameter(service_call_cfg, "max_trq", maxTorque);
            myUtils::readParameter(service_call_cfg, "max_trq_pld", maxTorque_pld);
            myUtils::readParameter(service_call_cfg, "max_temperature_pld", maxTemperature_pld);

        } catch(std::runtime_error& e) {
        std::cout << "Error Reading Parameter [" << e.what() << "]" << std::endl;
        }
    }
    
    void ScorpioNodelet::preProcess(){
        //create mapping std vector
        buildActiveJointIdx();
        buildPayloadJointIdx();

        //active joint
        for (int i = 0; i < atv_idx_list.size(); ++i) {
            //set run mode
             m_sync->changeMode("JOINT_IMPEDANCE", m_slave_list[atv_idx_list[i]]);;

             //register active states
             m_sync->registerMISO(&(m_raw_jpos[sensordata_idx_list[i]]), "js__joint__position__rad", m_slave_list[atv_idx_list[i]],false);
             m_sync->registerMISO(&(m_raw_jvel[sensordata_idx_list[i]]), "js__joint__velocity__radps", m_slave_list[atv_idx_list[i]],false);
             m_sync->registerMISO(&(m_raw_jtrq_atv[i]), "js__joint__effor__Nm", m_slave_list[atv_idx_list[i]],false);
             m_sync->registerMISO(&(m_raw_mot_curr_atv[i]), "motor__current__A", m_slave_list[atv_idx_list[i]],false);
             m_sync->registerMISO(&(m_raw_mot_temp_atv[i]), "motor__core_temp_est_C", m_slave_list[atv_idx_list[i]],false);

             //register cmd
             m_sync->registerMOSIPtr(&(m_raw_cmd_jpos_atv[i]), "cmd__joint__position__rad", m_slave_list[atv_idx_list[i]], false);
             m_sync->registerMOSIPtr(&(m_raw_cmd_jvel_atv[i]), "cmd__joint__velocity__radps", m_slave_list[atv_idx_list[i]], false);
             m_sync->registerMOSIPtr(&(m_raw_cmd_jtrq_atv[i]), "cmd__joint__effort__nm", m_slave_list[atv_idx_list[i]], false);

             m_sync->logMISO("motor__position__Rad", m_slave_list[atv_idx_list[i]]);
        }    

        //payload joint
        for (int i = 0; i < payload_idx_list.size(); ++i) {
           m_sync->changeMode("ISOE", m_slave_list[payload_idx_list[i]]); 

           m_sync->registerMISOPtr(&(m_raw_mot_temp_pld[i]), "motor__core_temp_est__C", m_slave_list[payload_idx_list[i]], false);
           m_sync->registerMISOPtr(&(m_raw_pld_frc[i]), "payload__effort__N", m_slave_list[payload_idx_list[i]], false);
           m_sync->registerMOSIPtr(&(m_raw_cmd_pld_frc[i]), "cmd__payload__effort__n", m_slave_list[payload_idx_list[i]], false);

           m_sync->logMISO("motor__current__A", m_slave_list[payload_idx_list[i]]);
           m_sync->logMISO("motor__currentDes__A", m_slave_list[payload_idx_list[i]]);
           m_sync->logMISO("motor__position__Rad", m_slave_list[payload_idx_list[i]]);
           m_sync->logMISO("actuator__position__m", m_slave_list[payload_idx_list[i]]);
           m_sync->logMISO("motor__core_temp_est__C", m_slave_list[payload_idx_list[i]]);
        }


        //FTsensor data
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(0)), "ati__Tx__filt__Nm", "Wrist_Roll", false);
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(1)), "ati__Ty__filt__Nm", "Wrist_Roll", false);
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(2)), "ati__Tz__filt__Nm", "Wrist_Roll", false);
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(3)), "ati__Fx__filt__N", "Wrist_Roll", false);
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(4)), "ati__Fy__filt__N", "Wrist_Roll", false);
        m_sync->registerMISOPtr(&(m_raw_ft_sensor(5)), "ati__Fz__filt__N", "Wrist_Roll", false);

        //TODO:Connect Robotiq Gripper
    }
    
    void ScorpioNodelet::parameterSetting(){
        //set safety limit
        //set ll control parameter kp & kd

        Eigen::VectorXd jp_kp, jp_kd, actu_kp, actu_kd, current_limit, temperature_limit, rotor_inertia;
        Eigen::VectorXi en_auto_kd, en_dob;
        try {
            YAML::Node ll_config = 
                YAML::LoadFile(THIS_COM "Config/Scorpio/LOW_LEVEL_CONFIG.yaml");
            YAML::Node service_call_cfg = ll_config["service_call"];
            myUtils::readParameter(service_call_cfg, "jp_kp", jp_kp);
            myUtils::readParameter(service_call_cfg, "jp_kd", jp_kd);
            myUtils::readParameter(service_call_cfg, "actu_kp", actu_kp);
            myUtils::readParameter(service_call_cfg, "actu_kd", actu_kd);
            myUtils::readParameter(service_call_cfg, "current_limit", current_limit);
            myUtils::readParameter(service_call_cfg, "temperature_limit", temperature_limit);
            myUtils::readParameter(service_call_cfg, "rotor_inertia", rotor_inertia);
            myUtils::readParameter(service_call_cfg, "en_dob", en_dob);
            myUtils::readParameter(service_call_cfg, "en_auto_kd", en_auto_kd);

        } catch(std::runtime_error& e) {
        std::cout << "Error Reading Parameter [" << e.what() << "]" << std::endl;
        }
        
        //parametersetting for active joint
        for (int i = 0; i < m_num_atv_j; ++i) {
            apptronik_srvs::Float32 srv_float;
            apptronik_srvs::UInt16 srv_int;

            srv_float.request.set_data = jp_kp[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Joint__Impedance__KP",srv_float);

            srv_float.request.set_data = jp_kd[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Joint__Impedance__KD",srv_float);

            srv_float.request.set_data = actu_kp[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Actuator__Effort__KP",srv_float);

            srv_float.request.set_data = actu_kd[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Actuator__Effort__KD",srv_float);

            srv_float.request.set_data = current_limit[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Limits__Motor__Current_Max_A",srv_float);

            srv_float.request.set_data = temperature_limit[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Limits__Motor__Max_winding_temp_C",srv_float);

            srv_float.request.set_data = rotor_inertia[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Actuator__Sprung_mass_kg",srv_float);

            srv_int.request.set_data = en_dob[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Actuator__Effort__EN_DOB",srv_int);

            srv_int.request.set_data = en_auto_kd[i];
            callSetService(m_slave_list[atv_idx_list[i]],"Control__Actuator__Effort__AutoKD",srv_int);
        }
        //TODO:What about payload joint???

    }

    void ScorpioNodelet::copyData(){
        m_copy_jpos = m_raw_jpos.cast<double>();
        //based on URDF, mirror the active jpos to passive jpos
        m_copy_jpos[2] = static_cast<double>(-m_raw_jpos[1])
        m_copy_jpos[3] = static_cast<double>(m_raw_jpos[1])
        m_copy_jpos[6] = static_cast<double>(-m_raw_jpos[5])
        m_copy_jpos[7] = static_cast<double>(m_raw_jpos[5])

        //based on URDF, mirror the active jvel to passive jvel
        m_copy_jve} = m_raw_jvel.cast<double>();
        m_copy_jvel[2] = static_cast<double>(-m_raw_jvel[1])
        m_copy_jvel[3] = static_cast<double>(m_raw_jvel[1])
        m_copy_jvel[6] = static_cast<double>(-m_raw_jvel[5])
        m_copy_jvel[7] = static_cast<double>(m_raw_jvel[5])

        m_copy_jtrq_atv = m_raw_jtrq_atv.cast<double>();
        m_copy_mot_curr_atv = m_raw_mot_curr_atv.cast<double>();
        m_copy_mot_temp_atv = m_raw_mot_temp_atv.cast<double>();
        m_copy_mot_temp_pld = m_raw_mot_temp_pld.cast<double>();
        m_copy_mom_arm_atv = m_raw_mom_arm_atv.cast<double>();
        m_copy_ft_sensor = m_raw_ft_sensor.cast<double>();

        //check sensor data if satisfying limits
        checkSensorData();

        m_sensor_data->q = m_copy_jpos;
        m_sensor_data->qdot = m_copy_jvel;
        m_sensor_data->jtrq = m_copy_jtrq_atv;
        m_sensor_data->pld_eff = m_copy_pld_frc;
        m_sensor_data->mot_curr = m_copy_mot_curr_atv;
        m_sensor_data->mot_temp_atv = m_copy_mot_temp_atv;
        m_sensor_data->mot_temp_pld = m_copy_mot_temp_pld;
        m_sensor_data->mom_arm = m_copy_mom_arm_atv;
        m_sensor_data->wrist_ft = m_copy_ft_sensor;

    }

    void ScorpioNodelet::checkSensorData(){
        if (!(myUtils::isInBoundingBox(m_copy_jpos, minJointPosition, maxJointPosition))){
            std::cout << "Measured Joint Position Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_mot_temp_atv, Eigen::VectorXd::Constant(m_num_atv_j,-50.), maxTemperature))){
            std::cout << "Measured Active Joint Motor Temperature Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_mot_temp_pld, Eigen::VectorXd::Constant(2,-50.), maxTemperature_pld))){
            std::cout << "Measured Payload Joint Motor Temperature Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_jvel, -maxJointVelocity, maxJointVelocity))){
            std::cout << "Measured Joint Velocity Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_jtrq_atv, -maxTorque, maxTorque))){
            std::cout << "Measured Joint Torque Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_pld_frc, -maxTorque_pld, maxTorque_atv_pld))){
            std::cout << "Measured Joint Torque Hits the Limit" << std::endl;
            turnOff();
        } else{
            //Do Nothing
        }
    }

    void ScorpioNodelet::turnOff(){
       if (m_count > 400){
        for (int i = 0; i < atv_idx_list.size(); ++i) {
           m_sync -> changeMode("OFF", m_slave_list[atv_idx_list[i]]) 
        }
        for (int i = 0; i < payload_idx_list.size(); ++i) {
           m_sync -> changeMode("OFF", m_slave_list[payload_idx_list[i]]) 
        }
       } 
    }

    void ScorpioNodelet::checkCommand(){
        if (!(myUtils::isInBoundingBox(m_copy_cmd_jpos_atv, minJointPosition, maxJointPosition))){
            std::cout << "Command Joint Position Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_cmd_jvel_atv,-maxVelocity,maxVelocity))){
            std::cout << "Command Joint Velocity Hits the Limit" << std::endl;
            turnOff();
        } else if (!(myUtils::isInBoundingBox(m_copy_cmd_jtrq_atv, -maxTorque, maxTorque))){
            std::cout << "Command Joint Torque Hits the Limit" << std::endl;
            turnOff();
        } else{
            //Do Nothing
        }
    }

    void ScorpioNodelet::copyCommand(){
        checkCommand();
        m_raw_cmd_jpos_atv = m_copy_cmd_jpos_atv.cast<float>();
        m_raw_cmd_jvel_atv = m_copy_cmd_jvel_atv.cast<float>();
        m_raw_cmd_jtrq_atv = m_copy_cmd_jtrq_atv.cast<float>();
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(scorpio_nodelet::ScorpioNodelet, nodelet::Nodelet)
