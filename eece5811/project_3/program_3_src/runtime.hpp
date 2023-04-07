/*

runtime.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses method definitions for Runtime and Summary objects, functions for
Summary sorting

*/

#pragma once

// custom includes
#include "schemes.hpp"
#include "process.hpp"
#include "file_io.hpp"

// stdlib includes
#include <iostream>
#include <string>
#include <sstream>
#include <numeric>


class Summary
{
    public:
        Summary(std::string r, float tw, float tt, int cs)
        {
            this->repr = r;
            this->t_wait = tw;
            this->t_turn = tt;
            this->cont_sw = cs;
        }
        std::string repr;
        float t_wait, t_turn;
        int cont_sw;
};


class Runtime
{
    public:
        Runtime(int snapshot_time, std::string output_file);
        Summary run_scheme(Scheme* scheme);
        void write_overall_sum(std::vector<Summary> summaries);

    private:
        Scheme* __schm;
        int __ss_time;
        std::string __out_file;
        Process* __currently_running;
        int __just_finished = -1;
        void __create_snapshot();
};


class SortWait
{
    public:
        bool operator() (Summary& a, Summary& b)
        {
            return a.t_wait < b.t_wait;
        }
};


class SortTurnaround
{
    public:
        bool operator() (Summary& a, Summary& b)
        {
            return a.t_turn < b.t_turn;
        }
};


class SortContextSwitch
{
    public:
        bool operator() (Summary& a, Summary& b)
        {
            return a.cont_sw < b.cont_sw;
        }
};
