#ifndef DELTA_ASDA_ETHERCAT_MASTER_H
#define DELTA_ASDA_ETHERCAT_MASTER_H
#include <string>
#include <vector>
// Boost
#include <boost/variant.hpp>
// SOEM
#include "ethercat.h"
//
#include "ethercat/common.h"
#include "ethercat/registry_idx.h"
#include "ethercat/pdo.h"


namespace delta { namespace asda { namespace ethercat {


inline int slave_setup(uint16 slave_idx)
{
  int wkc = 0;

  {
    uint16 sdo_1c12[] = { 0x00, 0x1603 };
    uint16 sdo_1c13[] = { 0x00, 0x1A03 };
    wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x00, sdo_1c12[0]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x00, sdo_1c13[0]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x01, sdo_1c12[1]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x01, sdo_1c13[1]);
  }

  // PDO mapping
  uint32 sdo_1603[] =
  {
    0x02,
    0x60400010,
    0x60C10120,
  };
  wkc += writeSDO<uint8>(slave_idx, 0x1603, 0x00, sdo_1603[0]);
  wkc += writeSDO<uint32>(slave_idx, 0x1603, 0x01, sdo_1603[1]);
  wkc += writeSDO<uint32>(slave_idx, 0x1603, 0x02, sdo_1603[2]);
  uint32 sdo_1A03[] =
  {
    0x02,
    0x60410010,
    0x60640020,
  };
  wkc += writeSDO<uint8>(slave_idx, 0x1A03, 0x00, sdo_1A03[0]);
  wkc += writeSDO<uint32>(slave_idx, 0x1A03, 0x01, sdo_1A03[1]);
  wkc += writeSDO<uint32>(slave_idx, 0x1A03, 0x02, sdo_1A03[2]);

  // Sync Managers mapping
  {
    // uint16 sdo_1c12[] = { 0x01, 0x1601 };
    // uint16 sdo_1c13[] = { 0x01, 0x1A01 };
    uint16 sdo_1c12[] = { 0x01, 0x1603 };
    uint16 sdo_1c13[] = { 0x01, 0x1A03 };
    wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x00, sdo_1c12[0]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x00, sdo_1c13[0]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x01, sdo_1c12[1]);
    wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x01, sdo_1c13[1]);
  }

  // Sync Managers synchronization
  uint16 sdo_1c32[] = { 0x20, 0x02 };
  uint16 sdo_1c33[] = { 0x20, 0x02 };
  // uint16 sdo_1c32[] = { 0x20, 0x01 };
  // uint16 sdo_1c33[] = { 0x20, 0x22 };
  wkc += writeSDO<uint16>(slave_idx, 0x1c32, 0x01, sdo_1c32[1]);
  wkc += writeSDO<uint16>(slave_idx, 0x1c33, 0x01, sdo_1c33[1]);

  return wkc;
}


class Master {
private:
  int ec_state = EC_STATE_NONE;

  const static size_t MAX_IO_MAP_SIZE = 4096;
  uint8 io_map[MAX_IO_MAP_SIZE];

  std::string ifname;
  std::vector<std::string> slaves;


  bool network_configuration()
  {
    for (int i = 0; i < slaves.size(); i++)
    {
      const uint16 slave_idx = 1 + i;
      if (strcmp(ec_slave[slave_idx].name, slaves[i].c_str()))
      {
        return false;
      }
    }
    return true;
  }

public:
  int wkc = 0;
  delta::asda::ethercat::pdo::RxPDO4 rx_pdo[10];
  delta::asda::ethercat::pdo::TxPDO4 tx_pdo[10];

  Master() { }

  Master(const std::string &ifname, const std::vector<std::string> &slaves) :
    ifname(ifname), slaves(slaves) { }


  bool init()
  {
    if (ec_init(ifname.c_str()) > 0)
    {
      printf("EtherCAT socket on: %s\n", ifname.c_str());
    }
    else
    {
      printf("Coludn't initialize EtherCAT Master socket on: %s\n", ifname.c_str());
      return false;
    }

    if (ec_config_init(FALSE) > 0)
    {
      printf("Slaves found and configured: %d\n", ec_slavecount);
    }
    else
    {
      printf("Coludn't find and configure any slave.\n");
      return false;
    }

    ec_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    // Network Configuration
    if (!network_configuration())
    {
      printf("Mismatch of network units!\n");
      return false;
    }

    // Distributed Clock
    ec_configdc();
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_dcsync0(slave_idx, TRUE, 8000000U, 0);
    }

    // Pre-Operational -> Safe-Operational
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].PO2SOconfig = slave_setup;
    }

    int used_mem = ec_config_map(&io_map);
    if (used_mem > sizeof(io_map))
    {
      printf("IO Map size: %d > MAX_IO_MAP_SIZE: %lu\n", used_mem, sizeof(io_map));
      return false;
    }
    printf("IO Map size: %d\n", used_mem);

    // print slaves configuration
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      print_sm(slave_idx, 0);     // SM0
      print_sm(slave_idx, 1);     // SM1
      print_sm(slave_idx, 2);     // SM2 (output)
      print_sm(slave_idx, 3);     // SM3 (input)
      print_fmmu(slave_idx, 0);   // FMMU0
      print_fmmu(slave_idx, 1);   // FMUU1
    }

    // SAFE OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
    }

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      set_mode_of_operation(slave_idx, mode_of_operation_t::INTERPOLATED_POSITION);
      config_position_interpolation(slave_idx, interpolation_sub_mode_t::LINEAR_INTERPOLATION, 8);
    }

    return true;
  }


  int fault_reset(uint16 slave_idx)
  {
    uint16 control_word = 0x0086;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
    return wkc;
  }


  int set_mode_of_operation(uint16 slave_idx, mode_of_operation_t mode_of_operation)
  {
    wkc += writeSDO<int8>(slave_idx, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);
    return wkc;
  }


  int config_position_interpolation(uint16 slave_idx, interpolation_sub_mode_t interpolation_sub_mode_select, uint8 interpolation_time_units, int8 interpolation_time_index = -3)
  {
    wkc += writeSDO<int16>(slave_idx, INTERPOLATION_SUB_MODE_SELECT_IDX, 0x00, interpolation_sub_mode_select);
    wkc += writeSDO<uint8>(slave_idx, INTERPOLATION_TIME_PERIOD_IDX, 0x01, interpolation_time_units);
    wkc += writeSDO<int8>(slave_idx, INTERPOLATION_TIME_PERIOD_IDX, 0x02, interpolation_time_index);
    return wkc;
  }


  int config_following_error_window(uint16 slave_idx, uint32 following_error_window, uint32 position_window, uint16 position_window_time)
  {
    wkc += writeSDO<uint32>(slave_idx, FOLLOWING_ERROR_WINDOW_IDX, 0x00, following_error_window);
    wkc += writeSDO<uint32>(slave_idx, POSITION_WINDOW_IDX, 0x00, position_window);
    wkc += writeSDO<uint16>(slave_idx, POSITION_WINDOW_TIME_IDX, 0x00, position_window_time);
    return wkc;
  }


  int config_quickstop(uint16 slave_idx, uint32 quickstop_deceleration)
  {
    wkc += writeSDO<uint32>(slave_idx, QUICKSTOP_DECELERATION_IDX, 0x00, quickstop_deceleration);
    return wkc;
  }


  int config_homing(uint16 slave_idx, int8 homing_method = 0, uint32 homing_speed_to_switch = 0, uint32 homing_speed_to_zero = 0, uint32 homing_acceleration = 0, int32 home_offset = 0, uint8 home_switch = 0x08)
  {
    wkc += writeSDO<int8>(slave_idx, HOMING_METHOD_IDX, 0x00, homing_method);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x01, homing_speed_to_switch);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x02, homing_speed_to_zero);
    wkc += writeSDO<uint32>(slave_idx, HOMING_ACCELERATION_IDX, 0x00, homing_acceleration);
    wkc += writeSDO<int32>(slave_idx, HOME_OFFSET_IDX, 0x00, home_offset);
    return wkc;
  }


  int get_actual_position(uint16 slave_idx, int32 &actual_position)
  {
    wkc += readSDO<int32>(slave_idx, POSITION_ACTUAL_VALUE_IDX, 0x00, actual_position);
    return wkc;
  }


  int set_position_offset(uint16 slave_idx, int32 position_offset)
  {
    wkc += writeSDO<int32>(slave_idx, POSITION_OFFSET_IDX, 0x00, position_offset);
    return wkc;
  }


  int set_position_factor(uint16 slave_idx, uint32 numerator, uint32 feed_costant)
  {
    wkc += writeSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x01, numerator);
    wkc += writeSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x02, feed_costant);
    return wkc;
  }


  int init_profile(uint16 slave_idx)
  {
    int32 target_position = 0;
    wkc += writeSDO(slave_idx, TARGET_POSITION_IDX, 0x00, target_position);

    uint32 profile_velocity = 200000;
    wkc += writeSDO(slave_idx, PROFILE_VELOCITY_IDX, 0x00, profile_velocity);

    uint32 profile_acceleration = 200000;
    wkc += writeSDO(slave_idx, PROFILE_ACCELERATION_IDX, 0x00, profile_acceleration);

    uint32 profile_deceleration = 200000;
    wkc += writeSDO(slave_idx, PROFILE_DECELERATION_IDX, 0x00, profile_deceleration);

    return wkc;
  }


  bool start()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
      rx_pdo[slave_idx].interpolated_position_command = 0;
    }

    update();

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
    }

    ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    print_ec_state(0);
    return true;
  }


  bool fault_reset()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0080;
    }

    return true;
  }


  bool ready_to_switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
    }

    return true;
  }


  bool switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0007;
    }

    return true;
  }


  bool switch_off()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFE;
    }

    return true;
  }


  bool start_motion()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }

    return true;
  }


  // Starting the Homing Procedure

  /* Set the Homing Method required using OD entry 6098h. To start the homing
   * procedure, bit 4 of the controlword OD entry located at dictionary address
   * 6040h, must transition from 0 to 1. The status of the homing procedure can
   * be monitored using the statusword OD entry 6041h. */

  bool start_homing()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x001F;
    }

    return true;
  }

  // Enable Cyclic Synchronous Position Mode

  /* In this mode the master controller generates a trajectory and sends target
   * position (0x607A) to the drive at every PDO update cycle. The primary feedback
   * from the drive is the actual motor position and optionally, actual motor
   * velocity and torque. Position, velocity, and torque control loops are all
   closed in the drive which acts as a follower for the position commands. */

  bool start_cyclic_syncronous_position()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }

    return true;
  }

  // Enable Cyclic Synchronous Velocity Mode

  /* In this mode the master controller sends target velocity (0x60FF) to the
   * drive at every PDO update cycle. The primary feedback from the drive is the
   * actual motor position and optionally, actual motor velocity and torque.
   * Velocity and torque control loops are closed in the drive. If necessary,
   * position loop is closed in the master controller. */

  bool start_cyclic_syncronous_velocity()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }

    return true;
  }


  int ovf = 0;
  uint64 dc_time;
  uint64 dc_time_1;

  int64 t_off = 0;

  int update()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
    }

    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
    }

    // int64 t_off;
    // ec_sync(ec_DCtime, 2000000U, &t_off);
    // printf("t_off: %ld\n", t_off);

    // printf("ec_DCtime: %ld\n", ec_DCtime);

    // int64 dc_delta = ec_DCtime % 2000000U;

    // dc_time_1 = dc_time;
    // if (ec_DCtime / 2000000U == 0)
    // {
    //   ovf++;
    //   dc_time = ovf * std::pow(2, 32) + ec_DCtime;
    // }
    // else
    // {
    //   dc_time = ovf * std::pow(2, 32) + ec_DCtime;
    // }
    // printf("dc_time: %lu\n", dc_time);
    //
    // ec_sync(dc_time, 2000000U, &t_off);
    return wkc;
  }


  void ec_sync(uint64 dc_time, int64 dc_cycle , int64 *t_off)
  {
    static int64 integral = 0;
    int64 delta = (dc_time - 1000000U) % dc_cycle;

    if (delta > (dc_cycle / 2))
    {
      delta = delta - dc_cycle;
    }
    if (delta > 0)
    {
      integral++;
    }
    if (delta < 0)
    {
      integral--;
    }
    *t_off = -(delta / 100) - (integral / 20);
  }


  bool halt()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0100;
    }

    return true;
  }


  bool quick_stop()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0b1111111111111011;
    }

    return true;
  }


  int get_error_code(const uint16 slave_idx, uint16 &error_code)
  {
    wkc += readSDO<uint16>(slave_idx, ERROR_CODE_IDX, 0x00, error_code);
    return wkc;
  }


  void close()
  {
    ec_close();
  }


  // void print_slave_status(const uint16 slave_idx)
  // {
  //   if ((tx_pdo[slave_idx].status_word >> 0) & 0x01)   // Ready to Switch On
  //   {
  //     ROS_INFO("Slave[%d]: Ready to Switch On", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 1) & 0x01)   // Switched On
  //   {
  //     ROS_INFO("Slave[%d]: Switched On", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 2) & 0x01)   // Operation Enabled
  //   {
  //     ROS_INFO("Slave[%d]: Operation Enabled", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 3) & 0x01)   // Fault
  //   {
  //     ROS_ERROR("Slave[%d]: Fault!!", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 4) & 0x01)   // Voltage Enabled
  //   {
  //     ROS_INFO("Slave[%d]: Voltage Enabled", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 5) & 0x01)   // Quick Stop
  //   {
  //     ROS_INFO("Slave[%d]: Quick Stop Enabled", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 6) & 0x01)   // Switch On Disabled
  //   {
  //     ROS_WARN("Slave[%d]: Switch On Disabled", slave_idx);
  //   }
  //   if ((tx_pdo[slave_idx].status_word >> 7) & 0x01)   // Warning
  //   {
  //     ROS_WARN("Slave[%d]: Warning", slave_idx);
  //   }
  //
  //   switch (tx_pdo[slave_idx].mode_of_operation_display)
  //   {
  //     case 6:   // HOMING
  //       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
  //       {
  //         ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
  //       {
  //         ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 12) & 0x01)           // Homing Attained
  //       {
  //         ROS_INFO("Slave[%d]: Homing Attained.", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Homing Error
  //       {
  //         ROS_ERROR("Slave[%d]: Homing Error.", slave_idx);
  //       }
  //       break;
  //
  //     case 8:   // CYCLIC SYNCHRONOUS POSITION
  //       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
  //       {
  //         ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
  //       {
  //         ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
  //       {
  //         ROS_ERROR("Slave[%d]: Following Error.", slave_idx);
  //       }
  //       break;
  //
  //     case 9:   // CYCLIC SYNCHRONOUS VELOCITY
  //       if ((tx_pdo[slave_idx].status_word >> 10) & 0x01)           // Target Reached
  //       {
  //         ROS_INFO("Slave[%d]: Target Reached.", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 11) & 0x01)           // Internal Limit Active
  //       {
  //         ROS_WARN("Slave[%d]: Internal Limit Active", slave_idx);
  //       }
  //       if ((tx_pdo[slave_idx].status_word >> 13) & 0x01)           // Following Error
  //       {
  //         ROS_ERROR("Slave[%d]: Following Error.", slave_idx);
  //       }
  //       break;
  //
  //     default:
  //
  //       break;
  //   }
  // }

};

} } }  // namespace
#endif
