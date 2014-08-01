#include "../include/channel.h"
#include <iostream>
#include <thread>

void log(std::string thread_name) {
  std::cout << thread_name << " running, id " << std::this_thread::get_id() << "\n";
}

void thread_a(utils::channel<char> c) {
  log("thread_a");
	c.send('A');
  std::cout << "thread_a sent: A\n";
}

void thread_b(utils::channel<char> c) {
  log("thread_b");
  char r = c.recv();
  assert('A' == r);
  std::cout << "thread_b received: " << r << std::endl;
}

void thread_c(utils::channel<char> c) {
  log("thread_c");
	c.send('C');
  std::cout << "thread_c sent: C\n";

  char r = c.recv();
  assert('D' == r);
  std::cout << "thread_c received: " << r << std::endl;
}

void thread_d(utils::channel<char> c) {
  log("thread_d");

  char r = c.recv();
  assert('C' == r);
  std::cout << "thread_d received: " << r << std::endl;

	c.send('D');
  std::cout << "thread_d sent: D\n";
}

int main() {
  std::cout << "====simple case: a send, b recv\n";
  { 
    utils::channel<char> c;
    std::thread b(thread_b, c);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::thread a(thread_a, c);

    std::cout << "threads joined\n";
    b.join();
    a.join();
  }

  std::cout << "\n====more complicate case: a and b both send and recv\n";
  { 
    utils::channel<char> c;
    std::thread b(thread_c, c);
    std::thread a(thread_d, c);

    std::cout << "threads joined\n";
    b.join();
    a.join();
  }

  return 0;
}
