libgochannel
============

*under development.*

# About

Code is based on https://github.com/ahorn/cpp-channel.

# How to use

# Internal

In libgochannel, 'channel' is constructed by

- a std::deque
- a std::mutex control the deque
- std::condition\_variable(s) control race cases

Below r1, r2, ..., represents receivers, s1, s2, ..., represents senders.

1) v0.1: 3 condition variables

cv\_data\_ready\_: data-ready, receivers wait for this. !queue.empty() is the sign.

- N=0, receivers wait for s1's data. Receivers are not ordered.
- N>0, receivers wait for a non-empty queue. Receivers are not ordered.

cv\_data\_received\_: data-received, queue.empty() is the sign. 

- N=0, s1 sent data, waits for received signal. r1 received data, notify s1. 

cv\_queue\_writable\_: queue-writable, queue.size() < N or ==0(for N=0) is the sign.

- N=0, s1 owns the channel, s2 has to wait for queue.empty().
- N>0, senders have to wait for queue.size() < N.

Flaw of design: 

0. set N=0,
1. s1 sent data, waiting for r1 receiving data. Sign is queue.empty(),
2. s2 has to wait for queue-writable, Sign is also queue.empty(),
3. r1 received data and send notification, should s1 or s2 be waken up?

Lesson: condition variables control different cases.


