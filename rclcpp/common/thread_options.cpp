#include "thread_options.hpp"
#include <rttest/rttest.h>

void ThreadOptions::report() {
  std::cout << "["<< name_ << " thread ] policy = " << get_policy_name(policy_)
            << " ,priority = " << priority_
            << " ,core assign(affinity) = " << affinity_ << std::endl;
}

void ThreadOptions::set_sched_priority(const std::string &thread_name,
                                       size_t priority, int policy) {
  if (rttest_set_sched_priority(priority, policy) != 0) {
    std::cout << "failed to set priority of " << thread_name;
  }
}

void ThreadOptions::set_affinity(const std::string &thread_name,
                                 const pthread_t &thread, int core_num) {
  cpu_set_t cpu_set;

  CPU_ZERO(&cpu_set);
  CPU_SET(core_num, &cpu_set);

  int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpu_set);
  if (result != 0) {
    std::cout << "failed to set affinity of " << thread_name << std::endl;
  }
}
void ThreadOptions::setup(pthread_t thread) {
    if (priority_ > sched_get_priority_min(policy_)) {
      set_sched_priority(name_, priority_, policy_);
    }
    if (affinity_ >= 0) {
      set_affinity(name_, thread, affinity_);
    }
}

std::string ThreadOptions::get_policy_name(int policy) {
  switch (policy) {
  case SCHED_OTHER:
    return "SCHED_OTHER";
  case SCHED_RR:
    return "SCHED_RR";
  case SCHED_BATCH:
    return "SCHED_BATCH";
  case SCHED_ISO:
    return "SCHED_ISO";
  case SCHED_IDLE:
    return "SCHED_IDLE";
  case SCHED_FIFO:
    return "SCHED_FIFO";
  case SCHED_DEADLINE:
    return "SCHED_DEADLINE";
  default:
    return "unknown";
  }
}
