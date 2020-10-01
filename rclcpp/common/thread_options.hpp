#include <string>
#include <iostream>
#include <thread>

class ThreadOptions {
 public:
  ThreadOptions(std::string name)
      : name_(name),priority_(-1), policy_(-1), affinity_(-1) {
  }
   void setup(const pthread_t thread);
   void report();
   std::string name_;
   int priority_;
   int policy_;
   int affinity_;

 private:
   std::string get_policy_name(int policy);
   void set_sched_priority(const std::string &thread_name, size_t priority,
                           int policy);
   void set_affinity(const std::string &thread_name, const pthread_t &thread,
                     int affinity);
};
