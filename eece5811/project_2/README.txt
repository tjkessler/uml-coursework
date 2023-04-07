################################################################################
#                                                                              #
#                             Program/Assignment 2                             #
#                Travis Kessler <Travis_Kessler@student.uml.edu>               #
#                                                                              #
################################################################################

1.0 - Compilation:
    Navigate to "kessler_travis_prog2/" on the command line/terminal,
    execute "make" to compile the following files:
        - ./program_2_src/monitor.cpp/hpp -> ./program_2_src/monitor.o
        - ./program_2_src/roles.cpp/hpp -> ./program_2_src/roles.o
        - ./program_2_src/thread_handling.cpp/hpp
            -> ./program_2_src/thread_handling.o
        - ./main.cpp, ./program_2_src/monitor.o, ./program_2_src/roles.o,
            ./program_2_src/thread_handling.o -> ./main

2.0 - Usage:
    Run "./main" by supplying three arguments:
        - the maximum size of the buffer/queue
        - the number of producer threads
        - the number of consumer threads
    E.g. "./main 4 1 1" for a buffer/queue size of 4, 1 producer and 1 consumer

*NOTE: the buffer design that was implemented uses FIFO - producers will place
    integers at the end of the queue, and consumers will always remove integers
    from the beginning of the queue ("position 0"). Don't be alarmed that all
    consumers read from position 0.
