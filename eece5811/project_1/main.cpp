/*

main.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Executes test programs (supplied by Dr. Geiger) from N forks (N, number of
children, specified on the command line; defaults to 1)

*/

// stdlib includes
#include <iostream>
#include <vector>

// custom includes
#include "./program_1_src/child_management.hpp"


using namespace std;


int main(int argc, char* argv[])
{
    /* Main function: called from command line

    Args:
        argc (int): number of arguments supplied
        *argv (arr): supplied command line arguments; program parses
            first arg, N, number of child processes to generate

    Returns:
        int: 0 if successful execution, -1 otherwise
    */

    // default number of children == 1, unless specified in cmd line
    int num_children = 1;
    if (argc > 1)
    {
        num_children = atoi(argv[1]);
        if (num_children < 1)
        {
            cout
                << "Cannot have less than one child: supplied "
                << num_children
                << endl;
            return -1;
        }
    }

    // create children
    std::vector<ChildProcess> children;
    bool is_parent = createChildren(num_children, children);
    
    // if parent, wait for children to complete
    if (is_parent)
    {
        waitForChildren(num_children, children);
    }

    return 0;
}
