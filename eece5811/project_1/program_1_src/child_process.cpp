/*

child_process.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses constructor for the ChildProcess class

*/

// custom includes
#include "child_process.hpp"

ChildProcess::ChildProcess(int cn, pid_t pd)
{
    /* ChildProcess constructor: assigns child number, PID to ChildProcess
    class

    Args:
        cn (int): child number to assign
        pd (pid_t): PID to assign
    */

    child_num = cn;
    pid = pd;
}
