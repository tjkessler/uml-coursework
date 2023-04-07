/*

child_management.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses createChildren and waitForChildren function definitions

*/

// stdlib includes
#include <array>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

// custom includes
#include "child_management.hpp"


using namespace std;


bool createChildren(int num_children, std::vector<ChildProcess>& children)
{
    /* createChildren function: populates supplied ChildProcess vector with
    ChildProcess objects

    Args:
        num_children (int): number of child processes to create
        children (vector<ChildProcess>&): vector.push(new child process)

    Returns:
        bool: true if parent, false if child
    */

    pid_t fork_id, my_id;
    int iter;

    std::array<const char*, 5> programs = {
        "./program_1_src/tests/test1",
        "./program_1_src/tests/test2",
        "./program_1_src/tests/test3",
        "./program_1_src/tests/test4",
        "./program_1_src/tests/test5"
    };

    for (iter = 0; iter < num_children; iter++)
    {
        fork_id = fork();
        my_id = getpid();

        if (fork_id < 0)
        {
            /* ERROR IN FORK */
            exit(-1);
        }

        else if (fork_id == 0)
        {
            /* CHILD PROCESS */
            cout
                << "Child "
                << iter + 1
                << " started with PID "
                << my_id
                << endl;
            execlp(programs[iter % 5], "", NULL);
        }

        else if (fork_id > 0)
        {
            /* PARENT PROCESS */
            children.push_back(ChildProcess(iter + 1, fork_id));
            if (iter == 0)
            {
                cout << "Parent PID is " << my_id << endl;
            }
        }
    }

    if (fork_id > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}


void waitForChildren(int num_children, const std::vector<ChildProcess>& children)
{
    /*  waitForChildren function: waits for all child processes to finish

    Args:
        num_children (int): number of child processes to create
        children (vector<ChildProcess>&): vector of ChildProcess objects

    Returns:
        None
    */

    pid_t finished_id;

    // until no more child processes are running
    while ((finished_id = wait(NULL)) != -1)
    {
        for (auto cld: children)
        {
            if (cld.pid == finished_id)
            {
                cout
                    << "Child "
                    << cld.child_num
                    << " (PID "
                    << cld.pid
                    << ") finished"
                    << endl;
                break;
            }
        }
    }
}
