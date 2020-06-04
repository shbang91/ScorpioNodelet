#ifndef SCORPIO_TESTER_NODELET
#define SCORPIO_TESTER_NODELET

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <rt_utils/utils.hpp>
#include <scorpio_tester/scorpio_tester.hpp>

namespace scorpio_tester
{
  class ScorpioTesterNodelet: public nodelet::Nodelet
  {
  public:
    ~ScorpioTesterNodelet();
    void onInit();
    void spinThread();

  private:
    boost::shared_ptr<aptk::comm::Synchronizer> m_sync;
    boost::shared_ptr<aptk::comm::RTNodeHandle> m_rt_nh;

    boost::shared_ptr<boost::thread> m_spin_thread;

    std::unordered_map<std::string, boost::shared_ptr<ScorpioTester> > m_testers;

    double m_relative_bus_time;

    bool m_controller_rt;
    int m_controller_core;
    int m_controller_priority;

    double m_damping_ratio;
    double m_base_kd;
  };
}

#endif //SCORPIO_TESTER_NODELET
