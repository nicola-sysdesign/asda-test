#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
//
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
// Boost
#include <boost/program_options.hpp>

#include "ethercat/master.h"
#include "ethercat/time.h"

#define POSITION_STEP_FACTOR  100000
#define VELOCITY_STEP_FACTOR  100000


using namespace delta::asda::ethercat;

void* control_loop(void* arg)
{
  delta::asda::ethercat::Master* ec_master = (delta::asda::ethercat::Master*)arg;

  struct timespec t, t_1, t0_cmd;
  clock_gettime(CLOCK_MONOTONIC, &t);

  for (int iter = 1; iter < 1800000; iter++)
  {
    add_timespec(&t, 2000000U + ec_master->t_off);
    printf("t_add:\t%d\n", 2000000U + ec_master->t_off);

    struct timespec t_left;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);

    struct timespec t_period;
    diff_timespec(t, t_1, &t_period);

    if (to_nsec(t_period) < 1900000 || 2100000 < to_nsec(t_period))
    {
      printf("t_period: %lu", to_nsec(t_period));
    }

    if (iter == 50)
    {
      std::cout << "Fault Reset ... ";
      if (ec_master->fault_reset())
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
      if (ec_master->ready_to_switch_on())
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
      if (ec_master->switch_on())
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
      std::cout << "Enable Motion ... ";
      if (ec_master->start_motion())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 500) t0_cmd = t;
    if (iter >= 500)
    {
      struct timespec t_cmd;
      diff_timespec(t, t0_cmd, &t_cmd);
      printf("t_cmd: %.3f\n", t_cmd.tv_sec + t_cmd.tv_nsec / 1000000000.0);

      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;

        uint16 status_word = ec_master->tx_pdo[slave_idx].status_word;
        int32 actual_position = ec_master->tx_pdo[slave_idx].actual_position;

        int32 target_position = 0;
        ec_master->rx_pdo[slave_idx].interpolated_position_command = target_position;
      }
    }

    ec_master->update();
    t_1 = t;
  }

  std::cout << "Finished." << std::endl;
}


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

  for (int i = 0; i < n_slaves; i++)
  {
    const uint16 slave_idx = 1 + i;

    int32 actual_position;
    ec_master.get_actual_position(slave_idx, actual_position);
    ec_master.set_position_offset(slave_idx, actual_position);

    uint32 numerator = 128, feed_costant = 10;
    ec_master.set_position_factor(slave_idx, numerator, feed_costant);
  }

  if (!ec_master.start())
  {
    return 1;
  }

  // POSIX Thread
  pthread_t pthread;
  pthread_attr_t pthread_attr;

  errno = pthread_attr_init(&pthread_attr);
  if (errno != 0)
  {
    perror("pthread_attr_init");
    return 1;
  }

  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set); CPU_SET(1, &cpu_set);
  errno = pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set), &cpu_set);
  if (errno != 0)
  {
    perror("pthread_attr_setaffinity_np");
    return 1;
  }

  errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
  if (errno != 0)
  {
    perror("%d, pthread_attr_setschedpolicy");
    return 1;
  }

  errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
  if (errno != 0)
  {
    perror("pthread_attr_setschedpolicy");
    return 1;
  }

  sched_param sched_param
  {
    .sched_priority = 80
  };
  errno = pthread_attr_setschedparam(&pthread_attr, &sched_param);
  if (errno != 0)
  {
    perror("pthread_attr_setschedparam");
    return 1;
  }

  errno = pthread_create(&pthread, &pthread_attr, &control_loop, &ec_master);
  if (errno != 0)
  {
    perror("pthread_create");
    return 1;
  }

  errno = pthread_attr_destroy(&pthread_attr);
  if (errno != 0)
  {
    perror("pthread_attr_destroy");
    return 1;
  }

  std::cin.get();

  ec_master.close();
  return 0;
}
