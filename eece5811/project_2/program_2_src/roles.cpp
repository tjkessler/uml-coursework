/*

roles.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses Producer and Consumer object methods

*/

// custom includes
#include "roles.hpp"


Producer::Producer(int id, Monitor* monitor)
{
    /* Producer constructor: assigns a new Producer obect
    an ID and a reference to a shared monitor

    Args:
        id (int): ID of the producer
        monitor (Monitor): shared Monitor object

    Returns:
        None (constructor)
    */

    p_id = id;
    p_monitor = monitor;
    return;
}


void Producer::write_to_buffer()
{
    /* Producer::write_to_buffer: the Producer generates
    a random integer between 0 and 10, and places it in
    the shared buffer/queue via the shared Monitor

    Args:
        None

    Returns:
        None
    */

    int rand_num = rand() % (11);
    p_monitor->push(rand_num, p_id);
    return;
}


int Producer::id()
{
    /* Producer::id: returns the ID of the producer

    Args:
        None

    Returns:
        int: ID of the producer
    */

    return p_id;
}


Consumer::Consumer(int id, Monitor* monitor)
{
    /* Consumer constructor: assigns the consumer an
    ID and a reference to a shared monitor

    Args:
        id (int): ID of the consumer
        monitor (Monitor): shared Monitor object

    Returns:
        None (constructor)
    */

    c_id = id;
    c_monitor = monitor;
    return;
}


void Consumer::read_from_buffer()
{
    /* Consumer::read_from_buffer: reads an integer
    from the shared buffer/queue

    Args:
        None

    Returns:
        None
    */

    c_monitor->pop(c_id);
    return;
}


int Consumer::id()
{
    /* Returns the ID of the consumer

    Args:
        None

    Returns:
        int: ID of the consumer
    */

    return c_id;
}
