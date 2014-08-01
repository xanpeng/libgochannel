#include "../include/channel.h"
#include <functional>
#include <array>

#include "gtest/gtest.h"

// N is the number of philosophers, traditionally five

// For comparison, here's the equivalent machine-readable
// CSP code of ChannelTest::DiningPhilosophersDeadlockFree:
//
// N = 5
// 
// FORK_ID = {0..N-1}
// PHIL_ID = {0..N-1}
// 
// channel picksup, putsdown:FORK_ID.PHIL_ID
// 
// PHIL(i) = picksup!i!i -> picksup!((i+1)%N)!i ->
//   putsdown!((i+1)%N)!i -> putsdown!i!i -> PHIL(i)
// 
// FORK(i) = picksup!i?j -> putsdown!i!j -> FORK(i)
// 
// LPHIL(i) = picksup!((i+1)%N)!i -> picksup!i!i ->
//   putsdown!((i+1)%N)!i -> putsdown!i!i -> LPHIL(i)
// 
// PHILS' = ||| i:PHIL_ID @ if i==0 then LPHIL(0) else PHIL(i)
// 
// SYSTEM = PHILS'[|{|picksup, putsdown|}|]FORKS
template<size_t N>
struct dining_table {
  std::array<utils::channel<size_t>, N> picksup;
  std::array<utils::channel<size_t>, N> putsdown;
};

// Allow fork to be used twice, so it can be picked up and
// put down by the person to the fork's right and left
template<size_t N>
void phil_fork(size_t i, dining_table<N>& t) {
  t.picksup.at(i).recv();
  t.putsdown.at(i).recv();
  t.picksup.at(i).recv();
  t.putsdown.at(i).recv();
}

// Picks up left and then right fork;
// finally, puts down both forks
template<size_t N>
void phil_person(size_t i, dining_table<N>& t) {
  t.picksup.at(i).send(i);
  t.picksup.at((i + 1) % N).send(i);
  t.putsdown.at(i).send(i);
  t.putsdown.at((i + 1) % N).send(i);
}

// Picks up right fork then left fork;
// finally, puts down both forks
template<size_t N>
void phil_different_person(size_t i, dining_table<N>& t) {
  t.picksup.at((i + 1) % N).send(i);
  t.picksup.at(i).send(i);
  t.putsdown.at(i).send(i);
  t.putsdown.at((i + 1) % N).send(i);
}

TEST(ChannelTest, DiningPhilosophersDeadlockFree) {
  constexpr size_t N = 5;
  dining_table<N> t;
  std::thread forks[N];
  std::thread phils[N];

  for (size_t i = 0; i < N; i++) {
    forks[i] = std::thread(phil_fork<N>, i, std::ref(t));

    // asymmetric philosophers solution to prevent deadlock
    if (i == 0) phils[i] = std::thread(phil_different_person<N>, i, std::ref(t));
    else phils[i] = std::thread(phil_person<N>, i, std::ref(t));
  }

  for (size_t i = 0; i < N; i++) {
    forks[i].join();
    phils[i].join();
  }
}

template<size_t N>
void generate_numbers(utils::ochannel<unsigned> c) {
  for (unsigned i = 2; i <= N; i++)
    c.send(i);
}

template<size_t N>
void filter_numbers(utils::ichannel<unsigned> in, utils::ochannel<unsigned> out, const unsigned prime) {
  unsigned i;
  do {
    i = in.recv();
    if (i % prime != 0)
      out.send(i);
  } while (i < N);
}

// The prime sieve up to N: daisy-chain threads together
template<size_t N>
void sieve_numbers(utils::channel<unsigned> primes) {
  utils::channel<unsigned> c;
  std::vector<std::thread> threads;
  threads.emplace_back(generate_numbers<N>, c);

  unsigned prime;
  for (;;) {
    prime = c.recv();
    primes.send(prime);

    if (prime >= N)
      break;

    utils::channel<unsigned> c_prime;
    threads.emplace_back(filter_numbers<N>, c, c_prime, prime);
    c = c_prime;
  }

  for (std::thread& thread : threads)
    thread.join();
}

// Classical inefficient concurrent prime sieve
//
// \see http://blog.onideas.ws/eratosthenes.go
// \see http://golang.org/test/chan/sieve1.go
TEST(ChannelTest, Sieve) {
  constexpr unsigned N = 97;
  utils::channel<unsigned> primes;
  std::thread sieve(sieve_numbers<N>, primes);
  utils::thread_guard sieve_guard(sieve);
  std::vector<unsigned> expected = {2, 3, 5, 7, 11, 13, 17, 19, 23,
                                    29, 31, 37, 41, 43, 47, 53, 59,
                                    61, 67, 71, 73, 79, 83, 89, N};
  for (unsigned expect : expected) {
    unsigned actual = primes.recv();
    EXPECT_EQ(expect, actual);
  }
}

void thread_a(utils::channel<char> c) {
  c.send('A');
  const char r = c.recv();

  EXPECT_EQ('B', r);
}

void thread_b(utils::channel<char> c) {
  char r = c.recv();
  c.send('B');

  EXPECT_EQ('A', r);
}

TEST(ChannelTest, SenderReceiver) {
  utils::channel<char> c;

  std::thread a(thread_a, c);
  utils::thread_guard a_guard(a);

  std::thread b(thread_b, c);
  utils::thread_guard b_guard(b);
}

void sender_thread_a(utils::channel<char> c) {
  c.send('A');
}

void sender_thread_b(utils::channel<char> c) {
  c.send('B');
}

void receiver_thread_a(utils::channel<char> c) {
  char r = c.recv();
  EXPECT_TRUE(r == 'A' || r == 'B');
}

void receiver_thread_b(utils::channel<char> c) {
  char r = c.recv();
  EXPECT_TRUE(r == 'A' || r == 'B');
}

TEST(ChannelTest, SenderReceiverWithMultipleChannels) {
  utils::channel<char> c;

  std::thread sa(sender_thread_a, c);
  utils::thread_guard sa_guard(sa);

  std::thread sb(sender_thread_b, c);
  utils::thread_guard sb_guard(sb);

  std::thread ra(receiver_thread_a, c);
  utils::thread_guard ra_guard(ra);

  std::thread rb(receiver_thread_a, c);
  utils::thread_guard rb_guard(rb);
}

TEST(ChannelTest, Copy) {
  utils::channel<int> c;
  utils::channel<int> d(c);
  EXPECT_EQ(c, d);
}

TEST(ChannelTest, Assignment) {
  utils::channel<int> c;
  utils::channel<int> d;

  EXPECT_NE(c, d);

  d = c;
  EXPECT_EQ(c, d);
}

TEST(ChannelTest, Cast) {
  utils::channel<int> c;
  utils::ichannel<int> d(c);
  utils::ochannel<int> e(c);

  EXPECT_EQ(c, d);
  EXPECT_EQ(d, c);
  EXPECT_EQ(c, e);
  EXPECT_EQ(e, c);
}

TEST(ChannelTest, AssignmentWithCast) {
  utils::channel<int> c;
  utils::ichannel<int> d(c);
  utils::ochannel<int> e(c);

  d = c;
  e = c;
  EXPECT_EQ(c, d);
  EXPECT_EQ(d, c);
  EXPECT_EQ(c, e);
  EXPECT_EQ(e, c);
}

void sender(utils::channel<int> c) {
  c.send(7);
}

void receiver(utils::channel<int> c, bool& done) {
  int r = c.recv();
  EXPECT_EQ(7, r);
  done = true;
}

TEST(ChannelTest, Channel) {
  bool done = false;
  utils::channel<int> c;

  std::thread f(sender, c);
  std::thread g(receiver, c, std::ref(done));

  f.join();
  g.join();

  EXPECT_TRUE(done);
}

void out(utils::ochannel<int> c) {
  c.send(7);
}

void in(utils::ichannel<int> c, bool& done) {
  int r = c.recv();
  EXPECT_EQ(7, r);
  done = true;
}

TEST(ChannelTest, DirectedChannel) {
  bool done = false;
  utils::channel<int> c;

  std::thread f(out, c);
  std::thread g(in, c, std::ref(done));

  f.join();
  g.join();

  EXPECT_TRUE(done);
}

void higher_order(utils::ichannel<utils::channel<bool>> c) {
  utils::channel<bool> done(c.recv());
  done.send(true);
}

TEST(ChannelTest, HigherOrderChannel) {
  utils::channel<utils::channel<bool>> c;
  utils::channel<bool> done;

  std::thread f(higher_order, c);
  utils::thread_guard f_guard(f);

  c.send(done);
  EXPECT_TRUE(done.recv());
}

void higher_order_with_cast(utils::ichannel<utils::channel<bool>> c) {
  utils::ochannel<bool> done(c.recv());
  done.send(true);
}

TEST(ChannelTest, HigherOrderChannelWithCast) {
  utils::channel<utils::channel<bool>> c;
  utils::channel<bool> done;

  std::thread f(higher_order_with_cast, c);
  utils::thread_guard f_guard(f);

  c.send(done);
  EXPECT_TRUE(done.recv());
}

struct int_wrapper {
  int i;
  explicit int_wrapper() : i(0) {}
  explicit int_wrapper(int j) : i(j) {}
  int_wrapper(const int_wrapper& other) : i(other.i) {}
};

void foo_sender(utils::channel<int_wrapper> c) {
  c.send(int_wrapper(7));
}

void foo_receiver(utils::channel<int_wrapper> c, bool& done) {
  std::unique_ptr<int_wrapper> foo_ptr(c.recv_ptr());
  EXPECT_EQ(7, foo_ptr->i);
  done = true;
}

// use channel<T>::recv_ptr()
TEST(ChannelTest, ExceptionUnsafeClassWithPtr) {
  bool done = false;
  utils::channel<int_wrapper> c;

  std::thread f(foo_sender, c);
  std::thread g(foo_receiver, c, std::ref(done));

  f.join();
  g.join();

  EXPECT_TRUE(done);
}

void bar_sender(utils::channel<int_wrapper> c) {
  c.send(int_wrapper(7));
}

void bar_receiver(utils::channel<int_wrapper> c, bool& done) {
  int_wrapper foo(0);
  c.recv(foo);
  EXPECT_EQ(7, foo.i);
  done = true;
}

// use channel<T>::recv(T&)
TEST(ChannelTest, ExceptionUnsafeClassWithArg) {
  bool done = false;
  utils::channel<int_wrapper> c;

  std::thread f(bar_sender, c);
  std::thread g(bar_receiver, c, std::ref(done));

  f.join();
  g.join();

  EXPECT_TRUE(done);
}

void receive_buffered_chars(utils::channel<char, 3> c) {
  // nonblocking, elements are received
  // in the order they are sent
  char char_a(c.recv());
  char char_b(c.recv());
  char char_c(c.recv());

  EXPECT_EQ('A', char_a);
  EXPECT_EQ('B', char_b);
  EXPECT_EQ('C', char_c);
}

TEST(ChannelTest, InterThreadAsynchronousChannel) {
  utils::channel<char, 3> c;

  // nonblocking
  c.send('A');
  c.send('B');
  c.send('C');

  // receive elements in another thread
  std::thread f(receive_buffered_chars, c);
  utils::thread_guard f_guard(f);
}

TEST(ChannelTest, IntraThreadAsynchronousChannel) {
  utils::channel<char, 3> c;

  // nonblocking
  c.send('A');
  c.send('B');
  c.send('C');

  char char_a(c.recv());
  char char_b(c.recv());
  char char_c(c.recv());

  EXPECT_EQ('A', char_a);
  EXPECT_EQ('B', char_b);
  EXPECT_EQ('C', char_c);
}

template<char Last>
void send_chars(utils::ochannel<char> out) {
  for (char i = 'A'; i <= Last; i++)
    out.send(i);
}

TEST(ChannelTest, SelectRecv) {
  utils::channel<char> c;
  utils::ichannel<char> in(c);
  char i = '\0';

  std::thread a(send_chars<'F'>, c);
  utils::thread_guard a_guard(a);

  utils::select().recv_only(c, i).wait();
  EXPECT_EQ('A', i);

  utils::select().recv(c, i, [](){}).wait();
  EXPECT_EQ('B', i);

  utils::select().recv_only(in, i).wait();
  EXPECT_EQ('C', i);

  utils::select().recv(in, i, [](){}).wait();
  EXPECT_EQ('D', i);

  utils::select().recv(c, [&i](const char k) { i = k; }).wait();
  EXPECT_EQ('E', i);

  utils::select().recv(in, [&i](const char k) { i = k; }).wait();
  EXPECT_EQ('F', i);
}

// N is the number of elements to receive, these are then stored in 'chars'
template<size_t N>
void recv_chars(utils::ichannel<char> in, std::vector<char>& chars) {
   for(size_t i = 0; i < N; i++)
     chars.push_back(in.recv());
}

TEST(ChannelTest, SelectSend) {
  constexpr size_t N = 8;

  utils::channel<char> c;
  utils::ochannel<char> out(c);
  std::vector<char> chars;
  unsigned i = 0;

  std::thread a(recv_chars<N>, c, std::ref(chars));
  utils::thread_guard a_guard(a);

  utils::select().send_only(c, 'A').wait();

  char char_b = 'B';
  utils::select().send_only(c, char_b).wait();

  utils::select().send_only(out, 'C').wait();

  char char_d = 'D';
  utils::select().send_only(out, char_d).wait();

  utils::select().send(c, 'E', [&i](){ i++; }).wait();
  EXPECT_EQ(1, i);

  char char_f = 'F';
  utils::select().send(c, char_f, [&i](){ i++; }).wait();
  EXPECT_EQ(2, i);

  utils::select().send(out, 'G', [&i](){ i++; }).wait();
  EXPECT_EQ(3, i);

  char char_h = 'H';
  utils::select().send(out, char_h, [&i](){ i++; }).wait();
  EXPECT_EQ(4, i);

  a.join();

  EXPECT_EQ(N, chars.size());
  for (size_t i = 0; i < N; i++)
    EXPECT_EQ('A' + i, chars.at(i));
}

// \see http://golang.org/test/chan/select4.go
TEST(ChannelTest, SelectOnlyAvailable) {
  utils::channel<unsigned, 1> c;
  utils::channel<unsigned> c_prime;
  c.send(42);

  unsigned v = 0;
  utils::select select;
  select.recv(c_prime, [](const unsigned){ throw "Bug"; });
  select.recv_only(c, v);
  select.wait();
  EXPECT_EQ(42, v);
}

void select_deque_1(utils::ichannel<bool> c1) {
  c1.recv();
}

void select_deque_2(utils::ichannel<bool> c1, utils::ichannel<bool> c2, utils::ochannel<bool> c3) {
  utils::select select;
  select.recv(c1, [](const bool){ throw "Bug"; });
  select.recv(c2, [&c3](const bool){ c3.send(true); });
  select.wait();
  c1.recv();
}

void select_deque_3(utils::ochannel<bool> c2) {
  c2.send(true);
}

// \see http://golang.org/test/chan/select6.go
// \see http://code.google.com/p/go/issues/detail?id=2075
TEST(ChannelTest, SelectDeque) {
  utils::channel<bool> c1;
  utils::channel<bool> c2;
  utils::channel<bool> c3;

  std::thread t1(select_deque_1, c1);
  utils::thread_guard t1_guard(t1);

  std::thread t2(select_deque_2, c1, c2, c3);
  utils::thread_guard t2_guard(t2);

  std::thread t3(select_deque_3, c2);
  utils::thread_guard t3_guard(t3);

  c3.recv();
  c1.send(true);
  c1.send(true);
}

void discard_recv1(utils::ichannel<int> c) {
  c.recv();
}

void discard_recv2(utils::ichannel<int> c) {
  int k;
  utils::select().recv_only(c, k).wait();
}

void discard_recv3(utils::ichannel<int> c) {
  utils::channel<int> c2;
  int k;
  utils::select().recv_only(c, k).recv_only(c2, k).wait();
}

template<class Recv>
void discard_send1(Recv f) {
  utils::channel<int> c;
  std::thread t(f, c);
  utils::thread_guard t_guard(t);
  c.send(1);
}

template<class Recv>
void discard_send2(Recv f) {
  utils::channel<int> c;
  std::thread t(f, c);
  utils::thread_guard t_guard(t);
  utils::select().send_only(c, 1).wait();
}

template<class Recv>
void discard_send3(Recv f) {
  utils::channel<int> c;
  std::thread t(f, c);
  utils::thread_guard t_guard(t);
  utils::channel<int> c2;
  utils::select().send_only(c, 1).send_only(c2, 1).wait();
}

// \see http://golang.org/test/chan/select7.go
// TODO: support the case where both ends of a channel are inside a select
TEST(ChannelTest, SelectDiscard) {
  discard_send1(discard_recv1);
  discard_send2(discard_recv1);
  discard_send3(discard_recv1);

  discard_send1(discard_recv2);
  // discard_send2(discard_recv2);
  // discard_send3(discard_recv2);
  discard_send1(discard_recv3);
  // discard_send2(discard_recv3);
  // discard_send3(discard_recv3);
}

