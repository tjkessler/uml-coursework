/*

thread_handling.hpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses function definitions for creating/joining threads

*/

// stdlib includes
#include <iostream>
#include <pthread.h>
#include <vector>

// custom includes
#include "roles.hpp"


// Structure for Producer Thread function arguments
struct prod_args
{
    int threadid;
    Monitor* monitor;
};


// Structure for Consumer Thread function arguments
struct con_args
{
    int threadid;
    int num_iter;
    Monitor* monitor;
};


// function declarations
void *run_producer(void* args);
std::vector<pthread_t> create_producers(int num_prod, Monitor* monitor);
void *run_consumer(void* args);
std::vector<pthread_t> create_consumers(
    int num_con,
    int reads_per_con,
    int remainder,
    Monitor* monitor
);
void join_producers(std::vector<pthread_t> producer_threads);
void join_consumers(std::vector<pthread_t> consumer_threads);
