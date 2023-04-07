/*

file_io.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses function definitions for file I/O functions

*/

#pragma once

// custom includes
#include "process.hpp"

// stdlib includes
#include <sstream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <iostream>


std::vector<Process> from_file(std::string filename);
void to_console_and_file(std::string filename, std::string content);
void erase_file(std::string filename);
