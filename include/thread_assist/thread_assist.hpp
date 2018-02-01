#ifndef __THREAD_ASSIST__
#define __THREAD_ASSIST__

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#define INTERRUPT_LOOP boost::this_thread::interruption_point

using namespace boost;
using namespace std;

class ThreadContainer
{
private:
    string id;
    string info;
    boost::shared_ptr<boost::thread> pthread;
public:
    enum Status
    {
        Run,
        Stop
    }status;

    ThreadContainer():
        status(Stop),
        id(""),
        info("")
    {}

    int regist(int (*task)(void), const char* description)
    {
        if (!task) return 1;

        info = description;
        pthread.reset(new boost::thread(task));
        status = Run;
        id = boost::lexical_cast<std::string>(pthread->get_id());
        return 0;
    }

    template<typename ObjT>
    int regist(int (ObjT::*task)(void), ObjT& obj, const char* description)
    {
        if (!task) return 1;

        info = description;
        pthread.reset(new boost::thread(boost::bind(task, obj)));
        status = Run;
        id = boost::lexical_cast<std::string>(pthread->get_id());
        return 0;
    }

    bool joinable()
    {
        if (!pthread)
        {
            return 0;
        }
        return pthread->joinable();
    }

    int interrupt()
    {
        // boost::this_thread::interruption_point();
        // every while(true) need add below line.
        if (joinable())
        {
            clock_t start, end;
            start = time(NULL);
            pthread->interrupt();
            pthread->timed_join(boost::posix_time::seconds(10));
            end = time(NULL);
            double duration = difftime(end,start);
            if(duration >= 9.0)
            {
                // if 5 seconds pass, thread still can't close
                pthread->interrupt();
                pthread->join();
                printf("[%s] can't closed normally.\r\n",info.c_str());
                status = Stop;
            }
            else
            {
                status = Stop;
                printf("[%s] has been closed normally, use %3.1f second\r\n",info.c_str(), duration);
            }
            fflush(stdout);
        }
    }

    inline string getID(){return id;}
    inline string getInfo(){return info;}
    inline void setStatus(int st){status =(Status)st;}
    inline string getStatus()
    {
        std::string str;
        switch(status)
        {
        case Run:  str = "Run"; break;
        case Stop: str = "Stop"; break;
        default: str = "Err";
        }
        return str;
    }
};

class ThreadAssist
{
public:
    enum Status
    {
        Run,
        Stop
    }status;

    ThreadAssist()
    {}

    ~ThreadAssist()
    {
        for(int i=0; i<thread_tables.size(); i++)
        {
            delete thread_tables[i];
        }
    }

    template<typename ObjT>
    int add_tables(const char* description, int (ObjT::*func)(void), ObjT& obj)
    {
        ThreadContainer* thread_container = new ThreadContainer();
        thread_container->regist(func, obj, description);
        thread_tables.push_back(thread_container);
        return 0;
    }

    int add_tables(const char* description, int (*MyTask)(void) )
    {
        ThreadContainer* thread_container = new ThreadContainer();
        thread_container->regist(MyTask, description);
        thread_tables.push_back(thread_container);
        return 0;
    }

    int setStatus(string id, Status status)
    {
        for(int i=0; i < thread_tables.size(); i++)
        {
            if (thread_tables[i]->getID() == id)
            {
                thread_tables[i]->setStatus(status);
            }
        }
        return 0;
    }

    void show_tables()
    {
        printf("***************** Thread Tables *******************\n");
        for(int i=0; i<thread_tables.size(); i++)
        {
            printf("idx:%d, id: %s is %s, info: %s\r\n",
                   i,
                   thread_tables[i]->getID().c_str(),
                   thread_tables[i]->getStatus().c_str(),
                   thread_tables[i]->getInfo().c_str()
                   );
        }
        printf("**************************************************\n");
        fflush(stdout);
    }

    void shutdown_all()
    {
        for(int i=0; i<thread_tables.size(); i++)
        {
            if (thread_tables[i]->joinable())
            {
                thread_tables[i]->interrupt();
                delete thread_tables[i];
            }
        }
        thread_tables.clear();
    }

    void fresh_tables()
    {
        std::cout<<"---------------REFRESH--------------"<<std::endl;
        for (auto p = thread_tables.begin(); p!= thread_tables.end(); p++)
        {
            std::string state((*p)->getStatus());
            if(state == string("Stop"))
            {
                (*p)->interrupt();
                std::cout<<(*p)->getID()<<" "<<(*p)->getStatus()<<std::endl;
                //delete *p;
                thread_tables.erase(p);
            }
        }
        std::cout<<"-----------------------------------"<<std::endl;
    }

    int size()
    {
        return thread_tables.size();
    }
private:
    std::vector<ThreadContainer*> thread_tables;
};

#endif
