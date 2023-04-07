/*

child_process.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses the ChildProcess class declaration

*/

// stdlib includes
#include <sys/types.h>


class ChildProcess
{
    /* ChildProcess object: houses child process child number, PID */

    public:
    int child_num;
    pid_t pid;
    ChildProcess(int cn, pid_t pd);
};
