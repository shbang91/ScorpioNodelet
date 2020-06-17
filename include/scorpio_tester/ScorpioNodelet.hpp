#ifndef SCORPIO_NODELET
#define SCORPIO_NODELET

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <rt_utils/utils.hpp>
//#include <scorpio_tester/scorpio_tester.hpp>
#include <apptronik_srvs/UInt16.h>
#include <apptronik_srvs/Float32.h>

#include <Eigen/Dense>

#include "PnC/ScorpioPnC/ScorpioInterface.hpp"


namespace scorpio_nodelet
{
  class ScorpioNodelet: public nodelet::Nodelet
  {
  public:
    ~ScorpioNodelet();
    void onInit();
    void spinThread();

  private:
    boost::shared_ptr<aptk::comm::Synchronizer> m_sync;
    boost::shared_ptr<aptk::comm::RTNodeHandle> m_rt_nh;

    boost::shared_ptr<boost::thread> m_spin_thread;

    //std::unordered_map<std::string, boost::shared_ptr<ScorpioTester> > m_testers;

    bool m_controller_rt;
    int m_controller_core;
    int m_controller_priority;

    std::vector<std::string> m_slave_list;
    std::vector<int> atv_idx_list;
    std::vector<int> payload_idx_list;
    std::vector<int> sensordata_idx_list;

    void buildActiveJointIdx();
    void buildPayloadJointIdx();
    void preProcess();

    int m_count;
    int m_num_atv_j;
    int m_num_tot_j;

    //active MISO & MOSI
    Eigen::VectorXf m_raw_jpos;
    Eigen::VectorXf m_raw_jvel;
    Eigen::VectorXf m_raw_jtrq_atv;
    Eigen::VectorXf m_raw_mot_curr_atv;
    Eigen::VectorXf m_raw_mot_temp_atv;
    Eigen::VectorXf m_raw_mom_arm_atv;//only inertia joint changes
    Eigen::Vector6f m_raw_ft_sensor;

    Eigen::VectorXf m_raw_cmd_jpos_atv;
    Eigen::VectorXf m_raw_cmd_jvel_atv;
    Eigen::VectorXf m_raw_cmd_jtrq_atv;

    //payload MISO & MOSI
    Eigen::VectorXf m_raw_mot_temp_pld;
    Eigen::VectorXf m_raw_pld_frc;
    Eigen::VectorXf m_raw_cmd_pld_frc;

    //??
    Eigen::VectorXd m_rotor_inertia_atv;
    Eigen::VectorXf m_static_gr_atv;
    Eigen::VectorXf m_static_torque_limits_atv;

    ScorpioInterface* m_interface;
    ScorpioSensorData* m_sensor_data;
    ScorpioCommand* m_command;

    Eigen::VectorXd m_copy_jpos;
    Eigen::VectorXd m_copy_jvel;
    Eigen::VectorXd m_copy_jtrq_atv;
    Eigen::VectorXd m_copy_mot_curr_atv;
    Eigen::VectorXd m_copy_mot_temp_atv;
    Eigen::VectorXd m_copy_mom_arm_atv;//only inertia joint changes
    Eigen::Vector6d m_copy_ft_sensor;

    Eigen::VectorXd m_copy_cmd_jpos_atv;
    Eigen::VectorXd m_copy_cmd_jvel_atv;
    Eigen::VectorXd m_copy_cmd_jtrq_atv;

    //payload MISO & MOSI
    Eigen::VectorXd m_copy_mot_temp_pld;
    Eigen::VectorXd m_copy_pld_frc;
    Eigen::VectorXd m_copy_cmd_pld_frc;

    //Safety Factors
    Eigen::VectorXd maxTemperature;
    Eigen::VectorXd maxJointPosition;
    Eigen::VectorXd minJointPosition;
    Eigen::VectorXd maxVelocity;
    Eigen::VectorXd maxTorque;
    Eigen::VectorXd maxTemperature_pld;
    Eigen::VectorXd maxTorque_pld;
    void turnOff();


    void copyData();
    void checkSensorData();
    void parameterSetting();
    void setCurrentPositionCmd();
    void copyCommand();

    template <class SrvType>
    void callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)

    template <class SrvType>
    void callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
  };

  template <class SrvType>
      void ScorpioNodelet::callSetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
      {
          std::string full_set_service = "/" + slave_name + "/" + srv_name + "/" + "set";
          ros::NodeHandle nh("~");

          ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

          if (client.call(srv_obj))
          {
              ROS_INFO("Called /%s/%s/set", slave_name.c_str(), srv_name.c_str());
          }
          else
          {
              ROS_ERROR("Failed to call service: %s", full_set_service.c_str());
          }
      }

  template <class SrvType>
      void ScorpioNodelet::callGetService(const std::string& slave_name, const std::string& srv_name, SrvType& srv_obj)
      {
          std::string full_get_service = "/" + slave_name + "/" + srv_name + "/" + "get";
          ros::NodeHandle nh("~");

          ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

          if (client.call(srv_obj))
          {
              ROS_INFO("Called /%s/%s/set", slave_name.c_str(), srv_name.c_str());
          }
          else
          {
              ROS_ERROR("Failed to call service: %s", full_get_service.c_str());
          }
      }
}

#endif //SCORPIO_NODELET
