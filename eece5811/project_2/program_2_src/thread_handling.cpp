/*

thread_handling.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses functions for creating/joining threads

*/

// custom includes
#include "thread_handling.hpp"


void *run_producer(void* args)
{
    /* run_producer: spawned by pthread_create to create a producer.
    The producer populates a shared queue with random numbers for a
    pre-determined number of iterations.

    Args:
        args (void*): a "prod_args" struct, containing the thread
            id, and the shared monitor object

    Returns:
        None (pthread_exit)
    */

    // obtain arguments from struct
    int threadid = ((struct prod_args*)args)->threadid;
    Monitor* monitor = ((struct prod_args*)args)->monitor;

    // e.g. if buffer max length is 5, producer will populate with
    //  10 integers
    int num_iter = monitor->max_length() * 2;

    // set up Producer object
    Producer* prod = new Producer(threadid, monitor);

    std::stringstream num_writes;
    num_writes
        << "P" << threadid
        << ": Producing " << num_iter
        << " values" << std::endl;
    std::cout << num_writes.str();

    // write to shared buffer
    int i;
    for (i = 0; i < num_iter; i++)
    {
        prod->write_to_buffer();
    }

    // exit thread
    std::stringstream done;
    done << "P" << threadid << ": Exiting" << std::endl;
    std::cout << done.str();
    pthread_exit(NULL);
}


std::vector<pthread_t> create_producers(int num_prod, Monitor* monitor)
{
    /* create_producers: spawns a pre-determined number of producer
    threads

    Args:
        num_prod (int): number of producers to spawn
        monitor (Monitor): monitor object (see monitor.cpp/hpp)

    Returns:
        vector (type pthread_t): vector of spawned pthread_t threads
    */

    // create pthread_t vector to house producers
    std::vector<pthread_t> prod_threads(num_prod);

    int i, res;
    // spawn producers
    for (i = 0; i < num_prod; i++)
    {
        // set up producer arguments
        struct prod_args *Args = (struct prod_args*)malloc(
            sizeof(struct prod_args)
        );
        Args->threadid = i;
        Args->monitor = monitor;

        // spawn thread
        res = pthread_create(
            &prod_threads[i],
            NULL,
            run_producer,
            (void*)Args
        );

        // check for error
        if (res)
        {
            std::stringstream error_msg;
            error_msg
                << "Error in creating producer thread "
                << i
                << ": return code "
                << res
                << std::endl;
            std::cout << error_msg.str();
            exit(-1);
        }

        // no error
        else
        {
            std::stringstream message;
            message << "Main: started producer " << i << std::endl;
            std::cout << message.str();
        }
    }
    return prod_threads;
}


void *run_consumer(void* args)
{
    /* run_consumer: spawned by pthread_create to create a consumer.
    The consumer reads a shared queue, removing integers in the front
    of the queue.

    Args:
        args (void*): a "con_args" struct, containing the thread id,
            the number of iterations to run, and the shared monitor
            object

    Returns:
        None (pthread_exit)
    */

    // obtain arguments from struct
    int threadid = ((struct con_args*)args)->threadid;
    int num_iter = ((struct con_args*)args)->num_iter;
    Monitor* monitor = ((struct con_args*)args)->monitor;

    // set up consumer object
    Consumer* con = new Consumer(threadid, monitor);

    std::stringstream num_reads;
    num_reads
        << "C" << threadid
        << ": Consuming " << num_iter
        << " values" << std::endl;
    std::cout << num_reads.str();

    int i;
    // consumer reads from buffer
    for (i = 0; i < num_iter; i++)
    {
        con->read_from_buffer();
    }

    // exit thread
    std::stringstream done;
    done << "C" << threadid << ": Exiting" << std::endl;
    std::cout << done.str();
    pthread_exit(NULL);
}


std::vector<pthread_t> create_consumers(int num_con, int reads_per_con,
                                        int remainder, Monitor* monitor)
{
    /* create_consumers: spawns a pre-determined number of consumer
    threads

    Args:
        num_con (int): number of consumers to spawn
        reads_per_con (int): number of times each consumer will read
            from the shared buffer
        remainder (int): any remaining reads, given to last consumer
        monitor (Monitor): monitor object (see monitor.cpp/hpp)

    Returns:
        vector (type pthread_t): vector of spawned pthread_t threads
    */

    // create pthread_t vector to house consumers
    std::vector<pthread_t> con_threads(num_con);

    int i, res;
    // spawn consumers
    for (i = 0; i < num_con; i++)
    {
        // set up consumer thread arguments
        struct con_args *Args = (struct con_args*)malloc(
            sizeof(struct con_args)
        );
        Args->threadid = i;
        Args->monitor = monitor;

        if (i < remainder)
        {
            Args->num_iter = reads_per_con + 1;
        }
        else
        {
            Args->num_iter = reads_per_con;
        }

        // create consumer thread
        res = pthread_create(
            &con_threads[i],
            NULL,
            run_consumer,
            (void*)Args
        );

        // check for error
        if (res)
        {
            std::cout
                << "Error in creating consumer thread "
                << i
                << ": return code "
                << res
                << std::endl;
            exit(-1);
        }

        // no error
        else
        {
            std::stringstream message;
            message << "Main: started consumer " << i << std::endl;
            std::cout << message.str();
        }
        
    }
    return con_threads;
}


void join_producers(std::vector<pthread_t> producer_threads)
{
    /* join_producers: join producer threads

    Args:
        producer_threads (vector, pthread_t): producer threads

    Returns:
        None
    */

    int i, res;
    void* status;

    for (i = 0; i < producer_threads.size(); i++)
    {
        res = pthread_join(producer_threads[i], &status);
        if (res)
        {
            std::cout
                << "Error in joining producer thread "
                << i
                << ": return code "
                << res
                << std::endl;
            exit(-1);
        }
        else
        {
            std::stringstream joined;
            joined << "Main: producer " << i << " joined" << std::endl;
            std::cout << joined.str();
        }
        
    }

    return;
}


void join_consumers(std::vector<pthread_t> consumer_threads)
{
    /* join_consumers: join consumer threads

    Args:
        consumer_threads (vector, pthread_t): consumer threads

    Returns:
        None
    */

    int i, res;
    void* status;

    for (i = 0; i < consumer_threads.size(); i++)
    {
        res = pthread_join(consumer_threads[i], &status);
        if (res)
        {
            std::cout
                << "Error in joining consumer thread "
                << i
                << ": return code "
                << res
                << std::endl;
            exit(-1);
        }
        else
        {
            std::stringstream joined;
            joined << "Main: consumer " << i << " joined" << std::endl;
            std::cout << joined.str();
        }
    }

    return;
}
