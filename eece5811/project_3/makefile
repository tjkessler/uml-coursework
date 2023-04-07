all: process.o schemes.o file_io.o runtime.o main

process.o: ./program_3_src/process.cpp ./program_3_src/process.hpp
	g++ -c -o ./program_3_src/process.o ./program_3_src/process.cpp

schemes.o: ./program_3_src/schemes.cpp ./program_3_src/schemes.hpp
	g++ -c -o ./program_3_src/schemes.o ./program_3_src/schemes.cpp

file_io.o: ./program_3_src/file_io.cpp ./program_3_src/file_io.hpp
	g++ -c -o ./program_3_src/file_io.o ./program_3_src/file_io.cpp

runtime.o: ./program_3_src/runtime.cpp ./program_3_src/runtime.hpp
	g++ -c -o ./program_3_src/runtime.o ./program_3_src/runtime.cpp

main: ./main.cpp
	g++ -o ./main ./main.cpp ./program_3_src/process.o ./program_3_src/schemes.o ./program_3_src/file_io.o ./program_3_src/runtime.o
