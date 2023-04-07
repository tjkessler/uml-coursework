/*

schemes.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses methods for Scheme object and child objects

*/

// custom includes
#include "schemes.hpp"


Scheme::Scheme(std::vector<Process> procs)
{
    /* Scheme constructor: initializes the Scheme

    Args:
        procs (std::vector<Process>): vector of process objects

    Returns:
        New Scheme object
    */

    this->__processes = procs;
}


bool Scheme::update()
{
    /* Scheme::update: increments the scheme's internal time by 1, places any
    arriving processes into the ready queue, and determines if an interrupt
    needs to occur (if the scheduling scheme is preemptive)

    Args:
        None

    Returns:
        bool: true if a new process has arrived and the scheme is preemptive,
            false otherwise
    */

    this->__time += 1;
    bool new_proc = false;
    for (auto& p: this->__processes)
        if (p.t_arrival == this->__time)
        {
            this->__ready_queue.push_back(&p);
            new_proc = true;
        }
    return (new_proc && this->__is_preemptive);
}


bool Scheme::all_done()
{
    /* Scheme::all_done: determines if all processes have finished processing

    Args:
        None

    Returns:
        bool: true if all processes have finished, false otherwise
    */

    bool all_done = true;
    for (auto& p: this->__processes)
        if (!p.done())
        {
            all_done = false;
            break;
        }
    return all_done;
}


int Scheme::curr_time()
{
    /* Scheme::curr_time: returns the current internal time of the scheme

    Args:
        None

    Returns:
        int: internal time of the scheme
    */

    return this->__time;
}


void Scheme::wait_for(Process* currently_running)
{
    /* Scheme::wait_for: increments the wait time for all processes in the ready
    queue if they are not currently running

    Args:
        currently_running (Process*): the currently running process

    Returns:
        None
    */

    for (auto& p: this->__ready_queue) if (currently_running->p_id != p->p_id)
        p->wait();
    return;
}


std::vector<int> Scheme::in_ready_queue()
{
    /* Scheme::in_ready_queue: returns a list of ID's for all processes
    currently in the ready queue

    Args:
        None

    Returns:
        std::vector<int>: all processes currently in the ready queue
    */

    std::vector<int> p_ids;
    std::vector<Process*> rq = this->__get_queue();
    for (auto& p: rq) if (!p->is_running) p_ids.push_back(p->p_id);
    return p_ids;
}


Process* Scheme::to_run()
{
    /* Scheme::to_run: returns the processes at the front of the ready queue

    Args:
        None

    Returns:
        Process*: process at the front of the ready queue
    */

    int i;
    for (i = 0; i < this->__ready_queue.size(); i++)
        if (this->__ready_queue[i]->done())
            this->__ready_queue.erase(this->__ready_queue.begin() + i);
    if (this->__ready_queue.size() == 0) return nullptr;
    else return this->__get_queue()[0];
}


std::vector<int> Scheme::wait_times()
{
    /* Scheme::wait_times: obtains wait times for all completed processes

    Args:
        None

    Returns:
        std::vector<int>: wait times for all completed processes
    */

    if (!this->all_done()) exit(-1);
    std::vector<int> times;
    for (auto& p: this->__processes) times.push_back(p.wait_time());
    return times;
}


std::vector<int> Scheme::turnaround_times()
{
    /* Scheme::turnaround_times: obtains turnaround times for all completed
    processes

    Args:
        None

    Returns:
        std::vector<int> turnaround times for all completed processes
    */

    if (!this->all_done()) exit(-1);
    std::vector<int> times;
    for (auto& p: this->__processes) times.push_back(p.turnaround_time());
    return times;
}


void Scheme::reset_rr()
{
    /* Scheme::reset_rr: if not RR scheme, this does nothing (just here so my
    compiler doesn't yell at me for missing a method)
    */

    return;
}


std::vector<Process*> Scheme::__get_queue()
{
    /* Scheme::__get_queue: as this is a method of the base class, it only
    returns the ready queue in the order that processes arrive (FCFS)

    Args:
        None

    Returns:
        std::vector<Process*>: the ready queue
    */

    return this->__ready_queue;
}


std::vector<Process*> FCFS::__get_queue()
{
    /* FCFS::__get_queue: returns the ready queue sorted by FCFS (ties are
    awarded to the process with the higher priority)

    Args:
        None

    Returns:
        std::vector<Process*>: ready queue sorted with FCFS
    */

    std::sort(
        this->__ready_queue.begin(),
        this->__ready_queue.end(),
        SortFCFS()
    );
    return this->__ready_queue;
}


std::vector<Process*> SJF::__get_queue()
{
    /* SJF::__get_queue: returns the ready queue sorted by SJF (ties are awarded
    to the process with the higher priority)

    Args:
        None

    Returns:
        std::vector<Process*>: ready queue sorted with SJF
    */

    std::sort(
        this->__ready_queue.begin(),
        this->__ready_queue.end(),
        SortSJF()
    );
    return this->__ready_queue;
}


std::vector<Process*> STCF::__get_queue()
{
    /* STCF::__get_queue: returns the ready queue sorted by STCF (ties are
    awarded to the process with the higher priority)

    Args:
        None

    Returns:
        std::vector<Process*>: ready queue sorted with STCF
    */

    std::sort(
        this->__ready_queue.begin(),
        this->__ready_queue.end(),
        SortSTCF()
    );
    return this->__ready_queue;
}


std::vector<Process*> Priority::__get_queue()
{
    /* Priority::__get_queue: returns the ready queue sorted by priority (ties
    are awarded to the process that showed up first)

    Args:
        None

    Returns:
        std::vector<Process*>: ready queue sorted by priority
    */

    std::sort(
        this->__ready_queue.begin(),
        this->__ready_queue.end(),
        SortPriority()
    );
    return this->__ready_queue;
}


bool RR::update()
{
    /* RR::update: update method for RR scheme child object, increasing time and
    current time quantum iteration; if the time quantum iteration is equal to
    the global time quantum, reset it and force a context switch

    Args:
        None

    Returns:
        bool: true if forcing switch, false otherwise
    */

    this->__time += 1;
    this->__curr_tq += 1;
    for (auto& p: this->__processes)
        if (p.t_arrival == this->__time)
            this->__ready_queue.push_back(&p);
    if (this->__curr_tq == this->__time_quantum)
    {
        this->__curr_tq = 0;
        return true;
    }
    else
        return false;
}


void RR::set_time_quantum(int tq)
{
    /* RR::set_time_quantum: sets the scheme's time quantum to a specified
    number

    Args:
        tq (int): number to set the TQ to

    Returns:
        None
    */

    this->__time_quantum = tq;
    return;
}


void RR::reset_rr()
{
    /* RR::reset_rr: resets the TQ counter for the RR scheme

    Args:
        None

    Returns:
        None
    */

    this->__curr_tq = 0;
}


std::vector<Process*> RR::__get_queue()
{
    /* RR::__get_queue: returns the ready queue sorted by RR

    TODO: this.

    Args:
        None

    Returns:
        std::vector<Process*>: ready queue sorted with RR
    */

    std::sort(
        this->__ready_queue.begin(),
        this->__ready_queue.end(),
        SortRR()
    );
    return this->__ready_queue;
}
