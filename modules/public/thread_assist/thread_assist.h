#ifndef __THREAD_ASSIST__
#define __THREAD_ASSIST__

/* -----------------------------------------------
 *
 *  Public Library
 *
 * --------------------------------------------- */

#include <iostream>
#include <vector>
#include <ctime>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#define INTERRUPT_LOOP boost::this_thread::interruption_point

using namespace boost;
using namespace std;

/*
-Thread Example

int mThread(void)
{
    while(xxxx)
    {
        INTERRUPT_LOOP();
        xxxxx();
        xxxxx;
    }
}
*/

class ThreadContainer
{
private:
    string id;
    string info;
    boost::shared_ptr<boost::thread> pthread;
public:
    enum Status
    {
        New,
        Run,
        Stop
    }status;

    ThreadContainer():
        status(New),
        id(""),
        info("")
    {}

    template<typename ObjT>
    int regist(int (ObjT::*task)(void), ObjT& obj, char* description);

    int regist(int (*task)(void), char* description);

    int join_able();

    int interrupt();

    inline string id_get(){return id;}
    inline string info_get(){return info;}
    string status_get();
};


class ThreadAssist
{
public:
    ~ThreadAssist()

    template<typename ObjT>
    int add_tables(char* description, int (ObjT::*func)(void), ObjT& obj);

    int add_tables(char* description, int (*MyTask)(void));

    void show_tables();

    void shutdown_all();

    int size();
private:
    std::vector<ThreadContainer*> thread_tables;
};

#endif
