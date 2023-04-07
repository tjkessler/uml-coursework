/*

main.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses main function, which initializes a monitor for a shared buffer,
executes functions for creating/joining producer/consumer threads

*/

// custom includes
#include "./program_2_src/thread_handling.hpp"

// stdlib includes
#include <time.h>


// print_info declaration
void print_info(int buffer_size, int num_prods, int num_cons, int rpc, int rem);


/*  MAIN  */
int main(int argc, char* argv[])
{
    // ensure correct number of arguments are supplied
    if (argc < 4)
    {
        std::cout
            << "Usage: ./main buffer_size num_prods num_cons"
            << std::endl;
        exit(-1);
    }

    // parse arguments
    int buffer_size = atoi(argv[1]);
    int num_prods = atoi(argv[2]);
    int num_cons = atoi(argv[3]);

    // determine number of reads per consumer and remainder
    int reads_per_con = buffer_size * 2 * num_prods / num_cons;
    int rem_reads = buffer_size * 2 * num_prods % num_cons;

    // print program information + runtime parameters
    print_info(buffer_size, num_prods, num_cons, reads_per_con, rem_reads);

    // random seed for integer generation
    srand(time(0));

    // initialize monitor
    Monitor* monitor = new Monitor(buffer_size);

    // initialize producers
    std::vector<pthread_t> prod_threads = create_producers(num_prods, monitor);

    // initialize consumers
    std::vector<pthread_t> con_threads = create_consumers(
        num_cons,
        reads_per_con,
        rem_reads,
        monitor
    );

    // join producer threads
    join_producers(prod_threads);

    // join consumer threads
    join_consumers(con_threads);

    std::cout << "Main: program completed" << std::endl;
    return 0;
}


void print_info(int buffer_size, int num_prods, int num_cons, int rpc, int rem)
{
    /* print_info: print program information + runtime parameters

    Args:
        buffer_size (int): size of the shared buffer/queue
        num_prods (int): number of producers to generate
        num_cons (int): number of consumers to generate
        rpc (int): number of reads per consumer
        rem (int): number of remaining buffer items

    Returns:
        None
    */

    std::cout
        << std::endl
        << "################################################" << std::endl
        << "#                                              #" << std::endl
        << "#    Program #2 - Written by Travis Kessler    #" << std::endl
        << "#       <Travis_Kessler@student.uml.edu>       #" << std::endl
        << "#                                              #" << std::endl
        << "################################################"
        << std::endl << std::endl
        << "Running with arguments:" << std::endl
        << "   - Buffer size: " << buffer_size << std::endl
        << "   - Num producers: " << num_prods << std::endl
        << "   - Num consumers: " << num_cons << std::endl
        << std::endl
        << "Each producer will produce "
        << buffer_size * 2 << " integers (total of "
        << buffer_size * 2 * num_prods << " integers)" << std::endl
        << "Each consumer will consume "
        << rpc << " integers (total of "
        << rpc * num_cons << " integers)"
        << std::endl;
    if (rem > 0)
    {
        std::cout
            << "The "
            << rem
            << " remaining integers will be split evenly among consumers"
            << std::endl;
    }
    std::cout
        << std::endl
        << "                     START                      "
        << std::endl
        << "------------------------------------------------"
        << std::endl << std::endl;

    return;
}
