Program/Assignment 1
Travis Kessler <Travis_Kessler@student.uml.edu>

Compilation:
    Navigate to "kessler_travis_prog1/" on the command line/terminal,
    execute "make" to compile the following files:
        - ./program_1_src/tests/test1.c -> ./program_1_src/tests/test1
        - ./program_1_src/tests/test2.c -> ./program_1_src/tests/test2
        - ./program_1_src/tests/test3.c -> ./program_1_src/tests/test3
        - ./program_1_src/tests/test4.c -> ./program_1_src/tests/test4
        - ./program_1_src/tests/test5.c -> ./program_1_src/tests/test5
        - ./program_1_src/child_management.cpp/hpp -> ./program_1_src/child_management.o
        - ./program_1_src/child_process.cpp/hpp -> ./program_1_src/child_process.o
        - ./main.cpp, ./program_1_src/child_management.o, ./program_1_src/child_process.o -> ./program_1

Usage:
    Default usage of "program_1" (without command line arguments):
        - exeute "./program_1"
        - default number of child processes == 1
    Usage of "program_1" with command line arguments:
        - supply one argument, int, number of child processes
        - "./program_1 N", where N == number of child processes
