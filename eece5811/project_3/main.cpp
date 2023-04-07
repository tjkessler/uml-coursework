/*

main.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses main function, which reads process information from a user-supplied file,
runs FCFS, SJF, STCF, RR and Priority scheduling schemes on them, and saves
information about their runtimes to a user-supplied file.

*/

// custom includes
#include "./program_3_src/schemes.hpp"
#include "./program_3_src/file_io.hpp"
#include "./program_3_src/process.hpp"
#include "./program_3_src/runtime.hpp"

// stdlib includes
#include <iostream>


// MAIN
int main(int argc, char* argv[])
{
    if (argc < 4)
    {
        std::cout
            << "Usage: ./main input_file output_file snapshot_time"
            << std::endl;
        exit(-1);
    }

    // read processes from file
    std::vector<Process> procs = from_file(argv[1]);

    // FCFS process scheduling
    FCFS* fcfs = new FCFS(procs);

    // SJF process scheduling
    SJF* sjf = new SJF(procs);

    // STCF process scheduling (priority)
    STCF* stcf = new STCF(procs);

    // Round robin scheduling
    RR* rr = new RR(procs);

    // Non-preemptive priority scheduling
    Priority* prior = new Priority(procs);

    // setup runtime
    Runtime* rt = new Runtime(atoi(argv[3]), argv[2]);
    erase_file(argv[2]);

    // run scheduling schemes
    std::vector<Summary> summaries;
    summaries.push_back(rt->run_scheme(fcfs));
    summaries.push_back(rt->run_scheme(sjf));
    summaries.push_back(rt->run_scheme(stcf));
    summaries.push_back(rt->run_scheme(rr));
    summaries.push_back(rt->run_scheme(prior));

    // write overall summary
    rt->write_overall_sum(summaries);

    return 0;
}
