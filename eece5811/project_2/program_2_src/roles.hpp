/*

roles.cpp

Written by Travis Kessler <Travis_Kessler@student.uml.edu>

Houses Producer and Consumer object/method definitions

*/

// Stdlib includes
#include "monitor.hpp"
#include <stdlib.h>
#include <iostream>


class Producer
{
    /* Producer object: writes to shared buffer/queue */

    public:
    Producer(int id, Monitor* monitor);
    void write_to_buffer();
    int id();

    private:
    int p_id;
    Monitor* p_monitor;
};


class Consumer
{
    /* Consumer object: reads from shared buffer/queue */

    public:
    Consumer(int id, Monitor* monitor);
    void read_from_buffer();
    int id();

    private:
    int c_id;
    Monitor* c_monitor;
};
