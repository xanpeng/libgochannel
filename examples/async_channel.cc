#include "../channel.h"
#include <sstream>
#include <thread>
#include <vector>

utils::channel<unsigned long long, 6> c;

unsigned long long thread_id(std::thread::id id) {
  std::stringstream ss;
  ss << id;
  return std::stoull(ss.str());
}

void log(unsigned long long me, unsigned long long other, std::string msg) {
  // use printf, instead of iostream, to get better console output
  printf("%lld %s: %lld\n", me, msg.c_str(), other);
}

void thread_send() {
  unsigned long long my_id = thread_id(std::this_thread::get_id());
  c.send(my_id);
  log(my_id, my_id, "send");
}

void thread_recv() {
  unsigned long long other_id = c.recv();
  unsigned long long my_id = thread_id(std::this_thread::get_id());
  log(my_id, other_id, "recv");
}

int main() {
  std::vector<std::thread> threads;
  for (int i = 0; i < 20; ++i) {
    if (i % 2 == 0) threads.push_back(std::thread(thread_send));
    if (i % 2 != 0) threads.push_back(std::thread(thread_recv));
  }
  for (std::thread& t : threads)
    t.join();

  return 0;
}
