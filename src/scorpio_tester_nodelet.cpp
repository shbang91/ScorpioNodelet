#include "scorpio_tester/scorpio_tester_nodelet.hpp"

namespace scorpio_tester
{
  ScorpioTesterNodelet::~ScorpioTesterNodelet()
  {
    m_spin_thread->join();
  }

  void ScorpioTesterNodelet::onInit()
  {
    m_spin_thread.reset(new boost::thread(boost::bind(&ScorpioTesterNodelet::spinThread, this)));
  }

  void ScorpioTesterNodelet::spinThread()
  {
    m_sync.reset(new aptk::comm::Synchronizer(true, "scorpio_tester"));
    m_rt_nh.reset(new aptk::comm::RTNodeHandle());
    ros::NodeHandle nh("~");

    nh.param("/rt/controller_rt", m_controller_rt, true);
    nh.param("/rt/controller_core", m_controller_core, 3);
    nh.param("/rt/controller_priority", m_controller_priority, 5);
    if(m_controller_rt)
      aptk::comm::enableRT(m_controller_priority, m_controller_core);

    m_damping_ratio = 0.707;
    m_base_kd = 0.5;

    m_rt_nh->addStateStreamUIInterface("Time", &m_relative_bus_time, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface("damping_ratio", &m_damping_ratio, 0.0, 2.0, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface("base_kd", &m_base_kd, 0.0, 1.0, "scorpio_tester");

    //initialize slaves & register MISO and MOSI
    for(auto slave_name : m_sync->getSlaveList())
    {
      if(slave_name.find("Axon") == std::string::npos)
      {
        continue;
      }
      m_testers[slave_name].reset(new ScorpioTester(slave_name, m_sync, m_rt_nh, &m_damping_ratio, &m_base_kd));
    }
    m_rt_nh->finalizeUITab("scorpio_tester");

    m_sync->connect();

    //main control loop
    while(m_sync->ok())
    {
      m_sync->awaitNextControl(); //wait for bus transaction

      //TODO:set controller's registered MOSI variables based on updated registered MISO variables

      m_relative_bus_time = m_sync->getRelativeBusTimeS();
      for(auto& tester : m_testers)
      {
        tester.second->process(m_relative_bus_time);
      }

      m_rt_nh->updateRT();
      m_sync->finishControl(); //indicate that we're done
    }
    m_sync->awaitShutdownComplete();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(scorpio_tester::ScorpioTesterNodelet, nodelet::Nodelet)
