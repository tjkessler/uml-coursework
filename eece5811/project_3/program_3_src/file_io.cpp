/*

file_io.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses functions for file I/O operations

*/

// custom includes
#include "file_io.hpp"


std::vector<Process> from_file(std::string filename)
{
    /* from_file: reads process information from a supplied file, returns them
    as a vector of Process objects

    Args:
        filename (std::string): location of the file to read from

    Returns:
        std::vector<Process>: vector of Process objects derived from file
    */

    std::vector<Process> input_processes;
    std::string line;
    std::ifstream infile(filename);
    int process_id = 0;
    while (std::getline(infile, line))
    {
        std::istringstream buf(line);
        std::istream_iterator<std::string> beg(buf), end;
        std::vector<std::string> items(beg, end);
        std::vector<int> attributes;
        for (auto& s: items)
            attributes.push_back(atoi(s.c_str()));
        input_processes.push_back(Process(
            process_id,
            attributes[0],
            attributes[1],
            attributes[2]
        ));
        process_id += 1;
    }
    return input_processes;
}


void to_console_and_file(std::string filename, std::string content)
{
    /* to_console_and_file: prints supplied message to console and saves to file

    Args:
        filename (std::str): file to save to
        content (std::string): message to print/save

    Returns:
        None
    */

    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::app);
    ofs << content;
    ofs.close();
    std::cout << content;
    return;
}


void erase_file(std::string filename)
{
    /* erase_file: clears the contents of the supplied file

    Args:
        filename (std::string): file to erase

    Returns:
        None
    */

    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    return;
}
