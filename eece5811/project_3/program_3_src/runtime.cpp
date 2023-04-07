/*

runtime.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses functions for runtime operations

*/

// custom includes
#include "runtime.hpp"


Runtime::Runtime(int snapshot_time, std::string output_file)
{
    /* Runtime constructor: initializes the runtime

    Args:
        snapshot_time (int): prints info every *this* iterations
        output_file (std::string): saves info to this file

    Returns:
        New Runtime object
    */

    this->__ss_time = snapshot_time;
    this->__out_file = output_file;
}


Summary Runtime::run_scheme(Scheme* scheme)
{
    /* Runtime::run_scheme: runs the scheduling runtime for the supplied scheme

    Args:
        scheme (Scheme*): processing scheme object (e.g. FCFS*, STCF*)

    Returns:
        Summary: summary object, outlining average wait time, average turnaround
            time and number of context switches
    */

    this->__schm = scheme;

    // print header
    std::stringstream header;
    header
        << "***** "
        << this->__schm->repr()
        << " Scheduling *****"
        << std::endl;
    to_console_and_file(this->__out_file, header.str());

    // MAIN RUNTIME
    bool switch_process = false;
    std::vector<int> process_sequence;
    int cs = 0;
    // until all processes have completed
    while (!this->__schm->all_done())
    {
        // obtain the process at the front of the ready queue, remove processes
        //  that have completed their processing
        this->__currently_running = this->__schm->to_run();

        // if the queue wasn't empty
        if (this->__currently_running)
        {
            // reset TQ counter if RR scheme
            this->__schm->reset_rr();

            // set the process to "running"
            this->__currently_running->is_running = true;

            // add proccess id to sequence, increment context switches
            if (process_sequence.empty() ||
                process_sequence[process_sequence.size() - 1]
                != this->__currently_running->p_id)
            {
                process_sequence.push_back(this->__currently_running->p_id);
                cs += 1;
            }

            // until the process has completed (or loop broken)
            while (!this->__currently_running->done())
            {
                // print info, if applicable snapshot time
                this->__create_snapshot();

                // increment the processes process time
                this->__currently_running->tick();

                // clear finished process integer
                this->__just_finished = -1;

                // increment wait times for all processes in the ready queue
                //  (except for the one currently running)
                this->__schm->wait_for(this->__currently_running);

                // update the scheme's internal time, add processes to queue if
                //  they arrive
                switch_process = this->__schm->update();

                // if a new processes has arrived and the scheme is preemptive
                //  (or RR time quantum completed), break
                if (switch_process) break;
            }

            // record if a process finished
            if (this->__currently_running->done())
                this->__just_finished = this->__currently_running->p_id;

            // for round-robin scheme, adjust the processes ready queue order
            //  (basically put it in the queue after newly arrived processes)
            //  NOTE: until this value is set here, it is equal to t_arrival
            this->__currently_running->rr_time = this->__schm->curr_time() + 1;

            // process done running (either finished or interrupted)
            this->__currently_running->is_running = false;
        }

        // nothing in the ready queue, t += 1, add arriving procs to RQ
        else
        {
            if (this->__schm->curr_time() != -1)
                this->__create_snapshot();
            this->__schm->update();
        }
    }

    // clear finished process flag
    this->__just_finished = -1;

    // obtain wait and turnaround times, calculate averages
    std::vector<int> wait_times = this->__schm->wait_times();
    std::vector<int> turnaround_times = this->__schm->turnaround_times();
    float average_wait = std::accumulate(
        wait_times.begin(),
        wait_times.end(),
        0.0
    ) / wait_times.size();
    float average_turnaround = std::accumulate(
        turnaround_times.begin(),
        turnaround_times.end(),
        0.0
    ) / turnaround_times.size();

    // create scheduling summary
    std::stringstream summary;
    summary
        << "*********************************************************"
        << std::endl << this->__schm->repr() << " Summary "
        << "(WT = wait time, TT = turnaround time)" << std::endl << std::endl;
    summary << "PID\tWT\tTT" << std::endl;
    int i;
    for (i = 0; i < wait_times.size(); i++)
        summary
            << i << "\t"
            << wait_times[i] << "\t"
            << turnaround_times[i]
            << std::endl;
    summary
        << "AVG\t" << average_wait << "\t" << average_turnaround
        << std::endl << std::endl << "Process sequence: ";
    for (i = 0; i < process_sequence.size(); i++)
    {
        summary << process_sequence[i];
        if (i != process_sequence.size() - 1)
            summary << "-";
    }
    summary
        << std::endl
        << "Context switches: " << cs
        << std::endl << std::endl << std::endl;
    to_console_and_file(this->__out_file, summary.str());

    return Summary(this->__schm->repr(), average_wait, average_turnaround, cs);
}


void Runtime::write_overall_sum(std::vector<Summary> summaries)
{
    /* Runtime::write_overall_sum: ranks wait times, turnaround times and
    context switches (least to greatest), prints to console and saves to file

    Args:
        summaries (std::vector<Summary>): vector of scheme Summary objects

    Returns:
        None
    */

    int i;

    std::stringstream overall_summary;
    overall_summary
        << "***** OVERALL SUMMARY *****" << std::endl << std::endl
        << "Wait Time Comparison" << std::endl;

    std::sort(
        summaries.begin(),
        summaries.end(),
        SortWait()
    );

    for (i = 0; i < summaries.size(); i++)
    {
        overall_summary
            << i + 1 << " " << summaries[i].repr;
        if (summaries[i].repr.length() < 5)
            overall_summary << "\t\t";
        else
            overall_summary << "\t";
        overall_summary << summaries[i].t_wait << std::endl;
    }

    overall_summary << std::endl << "Turnaround Time Comparison" << std::endl;

    std::sort(
        summaries.begin(),
        summaries.end(),
        SortTurnaround()
    );

    for (i = 0; i < summaries.size(); i++)
    {
        overall_summary
            << i + 1 << " " << summaries[i].repr;
        if (summaries[i].repr.length() < 5)
            overall_summary << "\t\t";
        else
            overall_summary << "\t";
        overall_summary << summaries[i].t_turn << std::endl;
    }

    overall_summary << std::endl << "Context Switch Comparison" << std::endl;

    std::sort(
        summaries.begin(),
        summaries.end(),
        SortContextSwitch()
    );

    for (i = 0; i < summaries.size(); i++)
    {
        overall_summary
            << i + 1 << " " << summaries[i].repr;
        if (summaries[i].repr.length() < 5)
            overall_summary << "\t\t";
        else
            overall_summary << "\t";
        overall_summary << summaries[i].cont_sw << std::endl;
    }

    to_console_and_file(this->__out_file, overall_summary.str());
}


void Runtime::__create_snapshot()
{
    /* Runtime::__create_snapshot: creates an information snapshot at the
    specified snapshot interval

    Args:
        None

    Returns:
        None
    */

    if (this->__schm->curr_time() % this->__ss_time != 0)
        return;

    std::stringstream to_write;

    if (!this->__currently_running)
        to_write
            << "t = " << this->__schm->curr_time() << std::endl
            << "CPU: Nothing being processed/in ready queue"
            << std::endl << std::endl;

    else
    {
        to_write << "t = " << this->__schm->curr_time() << std::endl;
        if (this->__currently_running->remaining()
            == this->__currently_running->t_burst)
        {
            to_write
                << "CPU: ";
            if (this->__just_finished >= 0)
                to_write
                    << "Finished process " << this->__just_finished << "; ";
            to_write
                << "Loading process "
                << this->__currently_running->p_id
                << " (CPU burst = "
                << this->__currently_running->t_burst
                << ")" << std::endl;
        }
        else
        {
            to_write
                << "CPU: Running process "
                << this->__currently_running->p_id
                << " (remaining CPU burst = "
                << this->__currently_running->remaining()
                << ")" << std::endl;
        }
        std::vector<int> in_queue = this->__schm->in_ready_queue();
        to_write << "Ready queue: ";
        int i;
        if (in_queue.empty())
            to_write << " empty";
        else
            for (i = 0; i < in_queue.size(); i++)
            {
                to_write << in_queue[i];
                if (i < in_queue.size() - 1 && in_queue.size() > 1)
                    to_write << "-";
            }
        to_write << std::endl << std::endl;
    }

    to_console_and_file(this->__out_file, to_write.str());
    return;
}
