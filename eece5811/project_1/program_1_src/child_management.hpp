/*

child_management.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses createChildren and waitForChildren function declarations

*/

// stdlib includes
#include <vector>

// custom includes
#include "child_process.hpp"


bool createChildren(
    int num_children,
    std::vector<ChildProcess>& children
);
void waitForChildren(
    int num_children,
    const std::vector<ChildProcess>& children
);
