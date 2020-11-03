// STL
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <ratio>
//
#include <pthread.h>
#include <unistd.h>
// soem
#include "ethercat.h"
// Boost
#include <boost/program_options.hpp>

#include "master.h"

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


  esa::ewdl::ethercat::Master ec_master(ifname, slaves);
  if (!ec_master.init())
  {
    return 0;
  }

  // pthread_t pthread;
  // pthread_attr_t pthread_attr;
  // pthread_create()

  const int n_slaves = slaves.size();

  std::vector<int> a_pos;     a_pos.resize(n_slaves, 0);
  std::vector<int> a_pos_cmd; a_pos_cmd.resize(n_slaves, 0);

  if (!ec_master.start())
  {
    return 0;
  }


  std::ofstream file("ewdl.log", std::ofstream::out);

  auto t0 = std::chrono::steady_clock::now();
  for (int iter = 0; iter < 2000; iter++)
  {
    std::this_thread::sleep_until(t0 + iter * std::chrono::microseconds(4000));
    auto t = std::chrono::steady_clock::now();

    if (iter == 0)
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
      std::cout << "Set Zero Position ... ";
      if (ec_master.set_zero_position())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 300)
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

    if (iter == 400)
    {
      std::cout << "Start Cyclic Synchronous Position Mode (CSP) ... ";
      if (ec_master.start_cyclic_syncronous_position())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter > 500)
    {
      auto t_cmd = t - 500 * std::chrono::microseconds(4000) - t0;

      for (int i = 0; i < n_slaves; i++)
      {
        const uint16 slave_idx = 1 + i;

        // read
        a_pos[i] = ec_master.tx_pdo[slave_idx].position_actual_value;

        // write
        a_pos_cmd[i] = limit * std::sin(M_PI * t_cmd.count() / 1000000000.0);
        ec_master.rx_pdo[slave_idx].target_position = a_pos_cmd[i];

        char record[1024];
        sprintf(record, "%ld\t%d\t%d", t_cmd.count(), a_pos[i], a_pos_cmd[i]);
        file << record << std::endl;
      }
    }

    ec_master.update();
  }


  file.close();

  ec_master.close();
  return 0;
}
