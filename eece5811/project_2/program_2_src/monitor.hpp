/*

monitor.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses object and method definitions for the Monitor object

*/

// Stdlib includes
#include <deque>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <sstream>


class Monitor
{
    /* Monitor object: houses mutex, condition variable, buffer
    to be shared by producer and consumer threads
    */

    public:
    Monitor(int max_len);
    int max_length();
    void push(int num, int id);
    void pop(int id);

    private:
    int maximum_length;
    std::mutex monitor_lock;
    std::condition_variable monitor_cond;
    std::deque<int> buffer;
};
