#include <scorpio_tester/scorpio_tester.hpp>

namespace scorpio_tester
{
  ScorpioTester::ScorpioTester(std::vector<std::string> slave_name, boost::shared_ptr<aptk::comm::Synchronizer> sync, boost::shared_ptr<aptk::comm::RTNodeHandle> rt_nh, double* damping_ratio, double* base_kd)
  {
    m_sync = sync;
    m_rt_nh = rt_nh;
    m_slave_names = slave_name;
    m_slave_idx = m_sync->getSlaveIdx(m_slave_name);

    m_damping_ratio = damping_ratio;
    m_base_kd = base_kd;

    m_sine_run_time = 0;
    m_hl_mode = 0;
    std::vector<std::string> mode_labels = {"OFF", "ZERO_TORQUE", "GRAV_COMP", "STATIONARY", "SINE_WAVE"};

    //get metadata
    if(!m_sync->getCachedParam("meta_cmd__joint__position__rad_min", m_min_joint_pos, m_slave_name))
    {
      m_min_joint_pos = 1.0;
    }
    if(!m_sync->getCachedParam("meta_cmd__joint__position__rad_max", m_max_joint_pos, m_slave_name))
    {
      m_max_joint_pos = 2.0;
    }
    std::cerr << "Joint " << m_slave_name << " bounded by " << m_min_joint_pos << ", " << m_max_joint_pos << std::endl;


    m_queued_jnt_imp_kp = 0;
    m_queued_jnt_imp_kd = 0;
    m_using_jnt_imp_kp = 0;
    m_using_jnt_imp_kd = 0;


    //connect states to synchronizer
    m_sync->registerMISOPtr(&m_pos, "js__joint__position__rad", m_slave_name);
    m_sync->registerMISOPtr(&m_vel, "js__joint__velocity__radps", m_slave_name);
    m_sync->registerMISOPtr(&m_torque, "js__joint__effort__Nm", m_slave_name);
    m_sync->registerMISOPtr(&m_force, "actuator__force__N", m_slave_name);
    m_sync->registerMISOPtr(&m_temp, "motor__core_temp_est__C", m_slave_name);
    m_sync->registerMISOPtr(&m_faults, "diag__faults__x", m_slave_name);

    //connect commands to synchronizer
    m_sync->registerMOSIPtr(&m_pos_d, "cmd__joint__position__rad", m_slave_name);
    m_sync->registerMOSIPtr(&m_vel_d, "cmd__joint__velocity__radps", m_slave_name);
    m_sync->registerMOSIPtr(&m_eff_d, "cmd__joint__effort__nm", m_slave_name);

    //set up UI states
    m_rt_nh->addHeading("state", m_slave_name, "scorpio_tester");
    m_rt_nh->addEnumStateStreamUIInterface(m_slave_name + "/low_level_mode", &m_ll_mode, m_sync->getSlaveModeNames(m_slave_name), "scorpio_tester");
    m_rt_nh->addEnumStateStreamUIInterface(m_slave_name + "/high_level_mode", &m_hl_mode, mode_labels, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/position_rad", &m_pos, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/velocity_radps", &m_vel, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/ff_inertia", &m_ff_inertia, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/force_N", &m_force, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/torque_Nm", &m_torque, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/temp_C", &m_temp, "scorpio_tester");
    m_rt_nh->addStringStateStreamUIInterface(m_slave_name + "/faults", &m_fault_str, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/sine_run_time_s", &m_sine_run_time, "scorpio_tester");

    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/queued_jnt_imp_kp", &m_queued_jnt_imp_kp, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/queued_jnt_imp_kd", &m_queued_jnt_imp_kd, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/using_jnt_imp_kp", &m_using_jnt_imp_kp, "scorpio_tester");
    m_rt_nh->addStateStreamUIInterface(m_slave_name + "/using_jnt_imp_kd", &m_using_jnt_imp_kd, "scorpio_tester");

    //set up UI commands
    m_frequency = 0.5;
    float center_joint_pos = (m_min_joint_pos + m_max_joint_pos) / 2.0;
    float quarter_joint_travel = (m_max_joint_pos - m_min_joint_pos) / 4.0;
    m_waveform_min = center_joint_pos - quarter_joint_travel; //default waveform to 50% total travel, centered
    m_waveform_max = center_joint_pos + quarter_joint_travel;
    m_rt_nh->addHeading("command", m_slave_name, "scorpio_tester");
    m_rt_nh->addCommandStreamUIInterface(m_slave_name + "/frequency_hz", &m_frequency, 0.01, 2.0, "scorpio_tester");

    //set up UI params
    m_kd_adj_factor = 1;
    m_ff_mass = 0.5648;
    m_ff_length = 0.145;
    m_ff_inertia_override = 0;
    m_ff_inertia_use_pt_mass = true;

    m_rt_nh->addHeading("param", m_slave_name, "scorpio_tester");
    m_rt_nh->addEnumParameterUIInterface(m_slave_name + "/high_level_mode", &m_hl_mode, mode_labels, "scorpio_tester", boost::bind(&ScorpioTester::hlModeCallback, this, _1));
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/kd_adj_factor", &m_kd_adj_factor, 0.0, std::numeric_limits<double>::max(), "scorpio_tester", boost::bind(&ScorpioTester::stiffnessCallback, this, _1));
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/ff_mass_kg", &m_ff_mass, 0.0, 6.0, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/ff_length_m", &m_ff_length, 0.0, 0.25, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/ff_inertia_override_kgm2", &m_ff_inertia_override, 0.0, 2.0, "scorpio_tester");
    m_rt_nh->addCheckboxParameterUIInterface(m_slave_name + "/ff_inertia_use_pt_mass", &m_ff_inertia_use_pt_mass, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/waveform_min_rad", &m_waveform_min, m_min_joint_pos, m_max_joint_pos, "scorpio_tester");
    m_rt_nh->addBoundedParameterUIInterface(m_slave_name + "/waveform_max_rad", &m_waveform_max, m_min_joint_pos, m_max_joint_pos, "scorpio_tester");

    m_rt_nh->info(aptk::comm::stream() << m_slave_name << " tester initialized");
  }

  void ScorpioTester::hlModeCallback(uint16_t data)
  {
    m_init_mode = true;
  }

  void ScorpioTester::stiffnessCallback(double data)
  {
  }

  void ScorpioTester::setImpedanceGains(float kp, float kd)
  {
    m_using_jnt_imp_kp = kp;
    m_using_jnt_imp_kd = kd;
    m_rt_nh->info(aptk::comm::stream() << "Setting gains to " << kp << ", " << kd);
    m_sync->setParam<float>("Control__Joint__Impedance__KP", kp, m_slave_name);
    m_sync->setParam<float>("Control__Joint__Impedance__KD", kd, m_slave_name);
  }

  void ScorpioTester::calcQueuedGains(double stiffness_factor)
  {
    double kd = *m_base_kd * stiffness_factor;
    double inertia = m_ff_inertia;

    if(inertia == 0)
    {
      inertia = m_ff_mass * m_ff_length * m_ff_length; //I = mr^2 for point mass
      if(inertia == 0)
      {
        m_queued_jnt_imp_kp = 0;
        m_queued_jnt_imp_kd = 0;
        m_rt_nh->warn("Zero inertia / damping_ratio -> bad stiffness calculation");
        return;
      }
    }

    double omega_n = kd / (2. * (*m_damping_ratio) * inertia);
    double kp = omega_n * omega_n * inertia;

    m_queued_jnt_imp_kp = kp;
    m_queued_jnt_imp_kd = kd;
  }

  double ScorpioTester::feedforwardTorque(double pos, double vel, double acc)
  {
    return m_ff_length * m_ff_mass * 9.81 * sin(pos) + m_ff_inertia * acc;
  }

  void ScorpioTester::processOff(double t)
  {
    if(m_init_mode)
    {
      m_sync->changeMode("OFF", m_slave_name);
      setImpedanceGains(0, 0);
      m_init_mode = false;
    }

    m_pos_d = m_pos;
    m_vel_d = 0;
    m_acc_d = 0;
    m_eff_d = 0;
  }

  void ScorpioTester::processZero(double t)
  {
    if(m_init_mode)
    {
      m_sync->setRunMode("JOINT_IMPEDANCE", m_slave_name);
      setImpedanceGains(0, 0);
      m_init_mode = false;
    }

    m_pos_d = m_pos;
    m_vel_d = 0;
    m_acc_d = 0;
    m_eff_d = 0;
  }

  void ScorpioTester::processStationary(double t)
  {
    if(m_init_mode)
    {
      m_sync->setRunMode("JOINT_IMPEDANCE", m_slave_name);
      setImpedanceGains(m_queued_jnt_imp_kp, m_queued_jnt_imp_kd);
      m_pos_d = m_pos;
      m_vel_d = 0;
      m_acc_d = 0;
      m_init_mode = false;
    }
    m_eff_d = feedforwardTorque(m_pos, m_vel_d, m_acc_d);
  }

  void ScorpioTester::processGravity(double t)
  {
    if(m_init_mode)
    {
      m_sync->setRunMode("JOINT_IMPEDANCE", m_slave_name);
      setImpedanceGains(0, 0);
      m_init_mode = false;
    }
    m_pos_d = m_pos;
    m_vel_d = 0;
    m_acc_d = 0;
    m_eff_d = feedforwardTorque(m_pos, m_vel_d, m_acc_d);
  }

  void ScorpioTester::processSine(double t)
  {
    if(m_init_mode)
    {
      //check phase sync-ability, alter bounds if we're outside
      if(m_pos > m_waveform_max)
      {
        m_rt_nh->warn(aptk::comm::stream() << "Current position " << m_pos << " is outside specified waveform bounds [" << m_waveform_min << ", " << m_waveform_max << "]! Expanding bounds to allow operation!");
        m_waveform_max = m_pos;
      }
      if(m_pos < m_waveform_min)
      {
        m_rt_nh->warn(aptk::comm::stream() << "Current position " << m_pos << " is outside specified waveform bounds [" << m_waveform_min << ", " << m_waveform_max << "]! Expanding bounds to allow operation!");
        m_waveform_min = m_pos;
      }

      m_sync->setRunMode("JOINT_IMPEDANCE", m_slave_name);
      setImpedanceGains(m_queued_jnt_imp_kp, m_queued_jnt_imp_kd);
      m_start_time = t;
      m_last_frequency = std::numeric_limits<double>::quiet_NaN();
      m_freq_rad = 2.0 * M_PI * m_frequency;
      m_freq_rad_2 = m_freq_rad * m_freq_rad;
      m_pos_d = m_pos;
      m_init_mode = false;
    }

    double amplitude = (m_waveform_max - m_waveform_min) / 2.0;
    double offset = (m_waveform_max + m_waveform_min) / 2.0;
    m_sine_run_time = t - m_start_time;

    if(m_last_frequency != m_frequency) //user moved the frequency slider, need to recompute phase
    {
      double last_freq_rad = m_freq_rad;
      m_freq_rad = 2.0 * M_PI * m_frequency;
      m_freq_rad_2 = m_freq_rad * m_freq_rad;

      if((last_freq_rad * amplitude * sin(last_freq_rad * (m_sine_run_time + m_sine_phase))) < 0) //negative vel -> invert phase
      {
        m_sine_phase = (2.0 * M_PI - acos((m_pos_d - offset) / amplitude)) / m_freq_rad - (m_sine_run_time);
      }
      else
      {
        m_sine_phase = acos((m_pos_d - offset) / amplitude) / m_freq_rad - (m_sine_run_time);
      }

      m_last_frequency = m_frequency;
    }

    double angle = m_freq_rad * (m_sine_run_time + m_sine_phase);
    m_pos_d = amplitude * cos(angle) + offset;
    m_vel_d = -m_freq_rad * amplitude * sin(angle);
    m_acc_d = -m_freq_rad_2 * amplitude * cos(angle);
    m_eff_d = feedforwardTorque(m_pos, m_vel_d, m_acc_d);
  }

  void ScorpioTester::process(double t)
  {
    m_sync->getCachedParam("mode", m_ll_mode, m_slave_name);

    m_ff_inertia = m_ff_inertia_use_pt_mass ? (m_ff_mass * m_ff_length * m_ff_length) : m_ff_inertia_override;
    calcQueuedGains(m_kd_adj_factor);

    //check faults, switch to off mode and try to clear faults if faulted
    uint16_t faults;
    std::vector<std::string> fault_names;
    m_sync->getIndicatedSlaveFaults(m_slave_idx, faults, fault_names);
    m_fault_str = boost::algorithm::join(fault_names, ", ");

    if(faults) //ignore faults if low-level is in off mode
    {
      if((fault_names.size() > 1) || (fault_names[0] != "DRIVER"))
      {
        m_rt_nh->warn(aptk::comm::stream() << "got faults: " << m_fault_str);
        m_hl_mode = OFF;
        m_sync->changeMode("OFF", m_slave_name);
        m_sync->getCachedParam("mode", m_ll_mode, m_slave_name);
        m_sync->clearFaults(m_slave_name);
        return;
      }
    }

    //handle the mode we're in
    switch(m_hl_mode)
    {
      case OFF: processOff(t); break;
      case ZERO_TORQUE: processZero(t); break;
      case SINE_WAVE: processSine(t); break;
      case STATIONARY: processStationary(t); break;
      case GRAV_COMP: processGravity(t); break;
      default:
      {
        m_rt_nh->error("Invalid mode selected! Reverting to OFF mode!");
        m_hl_mode = OFF;
      }
    }
  }
}
