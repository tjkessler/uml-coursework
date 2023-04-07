all: child_process.o child_management.o program_1 test1 test2 test3 test4 test5

child_process.o: ./program_1_src/child_process.cpp ./program_1_src/child_process.hpp
	g++ -c -o ./program_1_src/child_process.o ./program_1_src/child_process.cpp

child_management.o: ./program_1_src/child_management.cpp ./program_1_src/child_management.hpp
	g++ -c -o ./program_1_src/child_management.o ./program_1_src/child_management.cpp

program_1: ./main.cpp
	g++ -o ./program_1 ./main.cpp ./program_1_src/child_management.o ./program_1_src/child_process.o

test1: ./program_1_src/tests/test1.c
	gcc -o ./program_1_src/tests/test1 ./program_1_src/tests/test1.c

test2: ./program_1_src/tests/test2.c
	gcc -o ./program_1_src/tests/test2 ./program_1_src/tests/test2.c -lm

test3: ./program_1_src/tests/test3.c
	gcc -o ./program_1_src/tests/test3 ./program_1_src/tests/test3.c

test4: ./program_1_src/tests/test4.c
	gcc -o ./program_1_src/tests/test4 ./program_1_src/tests/test4.c

test5: ./program_1_src/tests/test5.c
	gcc -o ./program_1_src/tests/test5 ./program_1_src/tests/test5.c
