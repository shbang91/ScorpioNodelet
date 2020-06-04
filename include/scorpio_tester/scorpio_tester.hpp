#ifndef SCORPIO_TESTER
#define SCORPIO_TESTER

#include <control_utils/control_loop.hpp>
#include <rt_utils/utils.hpp>
#include <rt_utils/time_profiler.hpp>
#include <rt_utils/rt_nodehandle.hpp>
#include <boost/algorithm/string/join.hpp>

namespace scorpio_tester
{
  class ScorpioTester
  {
  public:
    enum Modes{OFF, ZERO_TORQUE, GRAV_COMP, STATIONARY, SINE_WAVE};

    ScorpioTester(std::vector<std::string> slave_name, boost::shared_ptr<aptk::comm::Synchronizer> sync, boost::shared_ptr<aptk::comm::RTNodeHandle> rt_nh, double* damping_ratio, double* base_kd);
    void process(double t);

  private:
    boost::shared_ptr<aptk::comm::Synchronizer> m_sync;
    boost::shared_ptr<aptk::comm::RTNodeHandle> m_rt_nh;
    std::vector<std::string> m_slave_names;
    int m_slave_idx;

    //rosparams
    bool m_log_data;
    double* m_damping_ratio;
    double* m_base_kd;
    
    //metadata
    float m_max_joint_pos;
    float m_min_joint_pos;

    //states
    float m_pos;
    float m_vel;
    float m_force;
    float m_torque;
    float m_temp;
    uint16_t m_faults;

    //UI states
    uint16_t m_ll_mode;
    uint16_t m_hl_mode;
    std::string m_fault_str;
    float m_sine_run_time;
    float m_queued_jnt_imp_kp;
    float m_queued_jnt_imp_kd;
    float m_using_jnt_imp_kp;
    float m_using_jnt_imp_kd;
    float m_ff_inertia;

    //commands
    float m_pos_d;
    float m_vel_d;
    float m_acc_d; //Note: not sent to axons
    float m_eff_d;

    //UI commands
    double m_frequency;
    double m_waveform_max;
    double m_waveform_min;

    //UI params
    double m_ff_mass;
    double m_ff_length;
    double m_ff_inertia_override;
    bool m_ff_inertia_use_pt_mass;
    double m_kd_adj_factor;

    //misc
    double m_last_frequency;
    double m_freq_rad;
    double m_freq_rad_2;

    bool m_init_mode;
    double m_start_time;
    double m_sine_phase;

    void hlModeCallback(uint16_t data);
    void stiffnessCallback(double data);

    void processOff(double t);
    void processZero(double t);
    void processSine(double t);
    void processStationary(double t);
    void processGravity(double t);

    double feedforwardTorque(double pos, double vel, double acc);

    void calcQueuedGains(double stiffness_factor);
    void setImpedanceGains(float kp, float kd);
  };
}

#endif //SCORPIO_TESTER
