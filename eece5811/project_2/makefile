all: monitor.o roles.o thread_handling.o main

monitor.o: ./program_2_src/monitor.cpp ./program_2_src/monitor.hpp
	g++ -c -o ./program_2_src/monitor.o ./program_2_src/monitor.cpp

roles.o: ./program_2_src/roles.cpp ./program_2_src/roles.hpp
	g++ -c -o ./program_2_src/roles.o ./program_2_src/roles.cpp

thread_handling.o: ./program_2_src/thread_handling.cpp ./program_2_src/thread_handling.hpp
	g++ -c -o ./program_2_src/thread_handling.o ./program_2_src/thread_handling.cpp

main: ./main.cpp
	g++ -o ./main ./main.cpp ./program_2_src/monitor.o ./program_2_src/roles.o ./program_2_src/thread_handling.o -pthread
