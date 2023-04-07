################################################################################
#                                                                              #
#                             Program/Assignment 3                             #
#                Travis Kessler <Travis_Kessler@student.uml.edu>               #
#                                                                              #
################################################################################

*NOTE: Current implementation includes extra-credit part 1, increasing process
    priority by one for every 25 cycles it must wait in the ready queue
    (process.cpp, in function void Process::wait(), line 58)

1.0 - Compilation:
    Navigate to "kessler_travis_prog3/" on the command line/terminal,
    execute "make" to compile the following files:
        - ./program_3_src/file_io.cpp/hpp -> ./program_3_src/file_io.o
        - ./program_3_src/process.cpp/hpp -> ./program_3_src/process.o
        - ./program_3_src/runtime.cpp/hpp -> ./program_3_src/runtime.o
        - ./program_3_src/schemes.cpp/hpp -> ./program_3_src/schemes.o
        - ./main.cpp (with all object files) -> ./main

2.0 - Usage:
    Run "./main" by supplying three arguments:
        - the input file containing process information
        - the output file to save snapshots and summaries
        - the time interval to take snapshots
    E.g. "./main testin1.dat out.txt 5" to load processes from "testin1.dat",
        save snapshots/summaries to "out.txt", and take a snapshot every 5
        iterations
