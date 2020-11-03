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
// Boost
#include <boost/program_options.hpp>
// soem
#include "ethercat.h"

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


  delta::asda::ethercat::Master ec_master(ifname, slaves);
  if (!ec_master.init())
  {
    return 1;
  }


  const int n_slaves = slaves.size();

  std::vector<int> a_pos;     a_pos.resize(n_slaves, 0);
  std::vector<int> a_pos_cmd; a_pos_cmd.resize(n_slaves, 0);

  if (!ec_master.start())
  {
    return 1;
  }


  auto t0 = std::chrono::steady_clock::now();
  for (int iter = 0; iter < 2000; iter++)
  {
    std::this_thread::sleep_until(t0 + iter * std::chrono::microseconds(4000));
    auto t = std::chrono::steady_clock::now();



    ec_master.update();
  }

  ec_master.close();
  return 0;
}
