// STL
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
//
#include <pthread.h>
#include <unistd.h>
// Boost
#include <boost/program_options.hpp>

#include "ethercat/common.h"
#include "ethercat/master.h"

#define POSITION_STEP_FACTOR  1280000
#define VELOCITY_STEP_FACTOR  1280000


int main(int argc, char* argv[])
{
  std::string ifname;
  std::vector<std::string> slaves;
  int limit;

  boost::program_options::options_description opt("Allowed options");
  opt.add_options()
    ("help,h", "Print this help and exit.")
    ("ifname,i", boost::program_options::value<std::string>(&ifname), "EtherCAT interface to use.")
    ("slaves,s", boost::program_options::value<std::vector<std::string>>(&slaves), "EtherCAT slaves.")
    ("limit,l", boost::program_options::value<int>(&limit)->default_value(POSITION_STEP_FACTOR), "Limit funcion between COUNTS.");

  boost::program_options::positional_options_description pos_opt;
  pos_opt.add("ifname", 1);
  pos_opt.add("slaves", -1);

  boost::program_options::command_line_parser cmd_line_parser(argc, argv);
  cmd_line_parser.options(opt);
  cmd_line_parser.positional(pos_opt);
  boost::program_options::variables_map var;
  boost::program_options::store(cmd_line_parser.run(), var);
  boost::program_options::notify(var);

  if (var.count("help"))
  {
    std::cout << "Usage: sin-test [options] <ifname> <slaves>...\n"
              << opt
              << std::endl;
    return 1;
  }

  // Init
  delta::asda::ethercat::Master ec_master(ifname, slaves);
  if (!ec_master.init())
  {
    return 1;
  }

  // Start
  const int n_slaves = slaves.size();

  std::vector<int> a_pos;     std::vector<int> a_pos_cmd;
  std::vector<int> a_vel;     std::vector<int> a_vel_cmd;
  std::vector<int> a_eff;     std::vector<int> a_eff_cmd;

  a_pos.resize(n_slaves, 0);  a_pos_cmd.resize(n_slaves, 0);
  a_vel.resize(n_slaves, 0);  a_vel_cmd.resize(n_slaves, 0);
  a_eff.resize(n_slaves, 0);  a_eff_cmd.resize(n_slaves, 0);

  if (!ec_master.start())
  {
    return 1;
  }

  // Loop
  auto t0 = std::chrono::steady_clock::now();
  for (int iter = 1; iter < 2000; iter++)
  {
    std::this_thread::sleep_until(t0 + iter * std::chrono::milliseconds(2));
    auto t = std::chrono::steady_clock::now();

    if (iter == 1)
    {
      std::cout << "Fault Reset ... ";
      if (ec_master.fault_reset())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 100)
    {
      std::cout << "Ready to Switch On ... ";
      if (ec_master.ready_to_switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 200)
    {
      std::cout << "Switch On ... ";
      if (ec_master.switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    // if (iter == 300)
    // {
    //   std::cout << "Enable Motion ... ";
    //   if (ec_master.start_motion())
    //   {
    //     std::cout << "SUCCESS" << std::endl;
    //   }
    //   else
    //   {
    //     std::cout << "FAILURE" << std::endl;
    //     return 0;
    //   }
    // }


    for (int i = 0; i < n_slaves; i++)
    {
      const uint16 slave_idx = 1 + i;
      uint16 status_word = ec_master.tx_pdo[slave_idx].status_word;
      int32 actual_position = ec_master.tx_pdo[slave_idx].actual_position;
      printf("wkc: %d\tstatus_word: 0x%.4X\n", ec_master.wkc, status_word);
      printf("wkc: %d\tactual_position: %d\n", ec_master.wkc, actual_position);

      a_pos[i] = actual_position;
      a_pos_cmd[i] = actual_position;

      ec_master.rx_pdo[slave_idx].target_position = a_pos_cmd[i];
    }


    for (int i = 0; i < n_slaves; i++)
    {
      const uint16 slave_idx = 1 + i;

      uint32 position_factor[3];
      ec_master.wkc += delta::asda::ethercat::readSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x01, position_factor[1]);
      ec_master.wkc += delta::asda::ethercat::readSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x02, position_factor[2]);
      printf("wkc: %d\tposition_factor: %d:%d\n", ec_master.wkc, position_factor[1], position_factor[2]);

      int32 actual_position;
      ec_master.get_actual_position(slave_idx, actual_position);
      printf("wkc: %d\tactual_position: %d\n", ec_master.wkc, actual_position);
    }

    ec_master.update();
  }

  ec_master.close();
  return 0;
}
