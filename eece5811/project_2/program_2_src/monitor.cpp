/*

monitor.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses Monitor object methods

*/

// custom includes
#include "monitor.hpp"


Monitor::Monitor(int max_len)
{
    /* Monitor constructor: initializes the Monitor object with
    a supplied maximum buffer/queue length

    Args:
        max_len (int): maximum length of buffer/queue

    Returns:
        None (constructor)
    */

    maximum_length = max_len;
}


int Monitor::max_length()
{
    /* Monitor::max_length: returns the maximum length of the
    monitor's buffer/queue

    Args:
        None

    Returns:
        int: maximum length of buffer/queue
    */

    return maximum_length;
}


void Monitor::push(int num, int id)
{
    /* Monitor::push: places an integer in the monitor's
    buffer/queue; if the buffer/queue is full, the producer
    will wait until it is not full

    Args:
        num (int): randomly generated number to place in the
            buffer/queue
        id (int): ID of the producer

    Returns:
        None
    */

    bool must_wait = false;
    std::unique_lock<std::mutex> lock(monitor_lock);
    monitor_cond.wait(
        lock,
        [this, id, &must_wait]()
        {
            if (buffer.size() >= maximum_length)
            {
                std::stringstream full_msg;
                full_msg
                    << "P" << id
                    << ": Blocked due to full buffer"
                    << std::endl;
                std::cout << full_msg.str();
                must_wait = true;
                return false;
            }
            else
            {
                return true;
            }
            // return buffer.size() < maximum_length;
        }
    );
    if (must_wait)
    {
        std::stringstream done_waiting;
        done_waiting
            << "P" << id
            << ": Done waiting on full buffer"
            << std::endl;
        std::cout << done_waiting.str();
    }
    int curr_size = buffer.size();
    buffer.push_back(num);
    std::stringstream message;
    message
        << "P"
        << id
        << ": Writing "
        << num
        << " to position "
        << curr_size
        << std::endl;
    std::cout << message.str();
    lock.unlock();
    monitor_cond.notify_one();
    return;
}


void Monitor::pop(int id)
{
    /* Monitor::pop: obtains and removes the integer at the
    front of the buffer/queue; if the buffer/queue is empty,
    the consumer will wait until it is populated

    Args:
        id (int): ID of the consumer

    Returns:
        None
    */

    bool must_wait = false;
    std::unique_lock<std::mutex> lock(monitor_lock);
    monitor_cond.wait(
        lock,
        [this, id, &must_wait]()
        {
            if (buffer.empty())
            {
                std::stringstream empty_msg;
                empty_msg
                    << "C" << id
                    << ": Blocked due to empty buffer"
                    << std::endl;
                std::cout << empty_msg.str();
                must_wait = true;
                return false;
            }
            else
            {
                return true;
            }
            // return !buffer.empty();
        }
    );
    if (must_wait)
    {
        std::stringstream done_waiting;
        done_waiting
            << "C" << id
            << ": Done waiting on empty buffer"
            << std::endl;
        std::cout << done_waiting.str();
    }
    int result = buffer.front();
    buffer.pop_front();
    std::stringstream message;
    message
        << "C"
        << id
        << ": Reading "
        << result
        << " from position 0"
        << std::endl;
    std::cout << message.str();
    lock.unlock();
    monitor_cond.notify_one();
    return;
}
