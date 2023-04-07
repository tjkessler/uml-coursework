/*

process.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses definitions for Process object

*/

#pragma once

class Process
{
    public:
        int p_id;
        int t_burst;
        int priority;
        int t_arrival;
        int rr_time;
        bool is_running = false;
        Process(
            int id,
            int t_burst,
            int priority,
            int t_arrival
        );
        void tick();
        void wait();
        bool done();
        int remaining();
        int turnaround_time();
        int wait_time();

    private:
        int __t_processed = 0;
        int __t_wait = 0;
        int __t_finished;
};
