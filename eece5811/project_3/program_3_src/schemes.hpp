/*

schemes.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses definitions for Scheme and child objects, sorting functions

*/

#pragma once

// custom includes
#include "process.hpp"

// stdlib includes
#include <vector>
#include <algorithm>
#include <string>


/* BASE SCHEME OBJECT */
class Scheme
{
    public:
        Scheme(std::vector<Process> procs);
        virtual bool update();
        bool all_done();
        Process* to_run();
        int curr_time();
        void wait_for(Process* currently_running);
        std::vector<int> in_ready_queue();
        virtual std::string repr(){return "";}
        std::vector<int> wait_times();
        std::vector<int> turnaround_times();
        virtual void reset_rr();

    protected:
        int __time = -1;
        bool __is_preemptive = false;
        std::vector<Process> __processes;
        std::vector<Process*> __ready_queue;
        virtual std::vector<Process*> __get_queue();
};


/* FCFS SCHEME CHILD OBJECT */
class FCFS : public Scheme
{
    public:
        FCFS(std::vector<Process> procs) : Scheme(procs){}
        virtual std::string repr(){return "FCFS";}

    protected:
        virtual std::vector<Process*> __get_queue();
};


/* SJF SCHEME CHILD OBJECT */
class SJF : public Scheme
{
    public:
        SJF(std::vector<Process> procs) : Scheme(procs){}
        virtual std::string repr(){return "SJF";}

    protected:
        virtual std::vector<Process*> __get_queue();
};


/* STCF SCHEME CHILD OBJECT */
class STCF : public Scheme
{
    public:
        STCF(std::vector<Process> procs) : Scheme(procs)
        {
            this->__is_preemptive = true;
        }
        virtual std::string repr(){return "STCF";}

    protected:
        virtual std::vector<Process*> __get_queue();
};


/* PRIORITY SCHEME CHILD OBJECT */
class Priority : public Scheme
{
    public:
        Priority(std::vector<Process> procs) : Scheme(procs){}
        virtual std::string repr(){return "Priority";}

    protected:
        virtual std::vector<Process*> __get_queue();
};


/* RR SCHEME CHILD OBJECT */
class RR : public Scheme
{
    public:
        RR(std::vector<Process> procs) : Scheme(procs){}
        virtual bool update();
        virtual std::string repr(){return "Round Robin";}
        void set_time_quantum(int tq);
        virtual void reset_rr();

    protected:
        virtual std::vector<Process*> __get_queue();

    private:
        std::vector<Process*> __rr_order;
        int __time_quantum = 2;
        int __curr_tq = 0;
};


class SortFCFS
{
    public:
        bool operator() (Process* a, Process* b)
        {
            /* SortFCFS::operator: used by std::sort to sort Process objects by
            their arrival time; ties are awarded to the process with the higher
            priority

            Args:
                a (Process*): first Process to compare
                b (Process*): second Process to compare

            Returns:
                bool: true if a < b, false otherwise
            */

            if (a->t_arrival == b->t_arrival)
                return a->priority < b->priority;
            else return a->t_arrival < b->t_arrival;
        }
};


class SortSJF
{
    public:
        bool operator() (Process* a, Process* b)
        {
            /* SortSJF::operator: used by std::sort to sort Process objects by
            their burst time; ties are awarded to the process with the higher
            priority

            Args:
                a (Process*): first Process to compare
                b (Process*): second Process to compare

            Returns:
                bool: true if a < b, false otherwise
            */

            if (a->t_burst == b->t_burst)
                return a->priority < b->priority;
            else return a->t_burst < b->t_burst;
        }
};


class SortSTCF
{
    public:
        bool operator() (Process* a, Process* b)
        {
            /* SortSTCF::operator: used by std::sort to sort Process objects by
            their remaining processing time; ties are awarded to the process
            with the higher priority

            Args:
                a (Process*): first Process to compare
                b (Process*): second Process to compare

            Returns:
                bool: true if a < b, false otherwise
            */

            if (a->remaining() == b->remaining())
                return a->priority < b->priority;
            else return a->remaining() < b->remaining();
        }
};


class SortPriority
{
    public:
        bool operator() (Process* a, Process* b)
        {
            /* SortPriority::operator: used by std::sort to sort Process objects
            by their priority; ties are awarded to the process that arrived
            first

            Args:
                a (Process*): first Process to compare
                b (Process*): second Process to compare

            Returns:
                bool: true if a < b, false otherwise
            */

            if (a->priority == b->priority)
                return a->t_arrival < b->t_arrival;
            else return a->priority < b->priority;
        }
};


class SortRR
{
    public:
        bool operator() (Process* a, Process* b)
        {
            /* SortRR::operator: used by std::sort to sort Process objects by
            their position in the RR scheme (combination of arrival time and
            processed count); ties are awarded to the process with higher
            priority

            Args:
                a (Process*): first Process to compare
                b (Process*): second Process to compare

            Returns:
                bool: true if a < b, false otherwise
            */

            if (a->rr_time == b->rr_time)
                return a->priority < b->priority;
            else return a->rr_time < b->rr_time;
        }
};
