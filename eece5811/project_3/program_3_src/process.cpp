/*

process.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses methods for Process object

*/

// custom includes
#include "process.hpp"

#include <iostream>


Process::Process(int id, int t_burst, int priority, int t_arrival)
{
    /* Process constructor: initializes the process

    Args:
        id (int): ID for the process
        t_burst (int): time it takes for the process to finish
        priority (int): priority of the process
        t_arrival (int): arrival time for the process

    Returns:
        New Process object
    */

    this->p_id = id;
    this->t_burst = t_burst;
    this->priority = priority;
    this->t_arrival = t_arrival;
    this->rr_time = t_arrival;
}


void Process::tick()
{
    /* Process::tick: increases the time the processes as processed by 1; if the
    process is completed, mark its completion time

    Args:
        None

    Returns:
        None
    */

    this->__t_processed += 1;
    if (this->__t_processed >= this->t_burst)
        this->__t_finished = this->t_arrival + this->__t_wait + this->t_burst;
    return;
}


void Process::wait()
{
    /* Process::wait: increases the time the processes has been waiting in the
    ready queue by 1; for every 25 cycles the process waits, its priority
    increases by 1 (priority -= 1)

    Args:
        None

    Returns:
        None
    */

    this->__t_wait += 1;
    if (this->__t_wait % 25 == 0 && this->priority > 1)
        this->priority -= 1;
    return;
}


bool Process::done()
{
    /* Process::done: determines whether the processes has completed processing
    (if the total processed time is equal to or greater than its burst time)

    Args:
        None

    Returns:
        bool: true if complete, false otherwise
    */

    return this->__t_processed >= this->t_burst;
}


int Process::remaining()
{
    /* Process::remaining: determines how much processing time the process
    requires before completing

    Args:
        None

    Returns:
        int: how much time the process requires to complete
    */

    return this->t_burst - this->__t_processed;
}


int Process::turnaround_time()
{
    /* Process::turnaround_time: returns the turnaround time for the process

    Args:
        None

    Returns:
        int: turnaround time
    */

    return this->__t_finished - this->t_arrival;
}


int Process::wait_time()
{
    /* Process::wait_time: returns the wait time for the process

    Args:
        None

    Returns:
        int: wait time
    */

    return this->__t_wait;
}
