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

//     Thread Example
//    int mThread(void)
//    {
//        while(xxxx)
//        {
//            INTERRUPT_LOOP();
//            xxxxx();
//            xxxxx;
//        }
//    }

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

    int regist(int (*task)(void), const char* description)
    {
        if (!task || pthread)
        {
           return 1;
        }
        info = description;
        pthread = boost::shared_ptr<boost::thread>(new boost::thread(task));
        status = Run;
        if (!pthread)
        {
            status = New;
            cerr << info << " thread create fail"<<endl;
            return 1;
        }
        id = boost::lexical_cast<std::string>(pthread->get_id());
        return 0;
    }

    template<typename ObjT>
    int regist(int (ObjT::*task)(void), ObjT& obj, const char* description)
    {
        if (!task || pthread) return 1;

        info = description;
        pthread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(task, obj)));
        status = Run;
        if (!pthread)
        {
            status = New;
            cerr << info << " thread create fail"<<endl;
            return 1;
        }
        id = boost::lexical_cast<std::string>(pthread->get_id());
        return 0;
    }

    bool join_able()
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
        if (join_able())
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
                printf("[%s] can't closed normally.\r\n",info.c_str());
                status = Stop;
            }
            else
            {
                status = Stop;
                printf("[%s] has been closed normally, use %3.1f second\r\n",info.c_str(), duration);
            }
        }
    }

    inline string id_get(){return id;}
    inline string info_get(){return info;}
    inline void status_set(Status st){status = st;}
    string status_get()
    {
        std::string str;
        switch(status)
        {
        case New: str = "New"; break;
        case Run: str = "Run"; break;
        case Stop:str = "Stop"; break;
        default: str = "Err";
        }
        return str;
    }
};

class ThreadAssist
{
public:
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

    void fresh_tables(void)
    {
        for(int i=0; i<thread_tables.size(); i++)
        {
            // thread started
            if (thread_tables[i]->join_able())
            {
                thread_tables[i]->status_set(ThreadContainer::Run);
            }
            // thread has benn stopped
            else
            {
                thread_tables[i]->status_set(ThreadContainer::Stop);
            }
        }
    }

    void show_tables()
    {
        printf("***************** Thread Tables *******************\n");
        for(int i=0; i<thread_tables.size(); i++)
        {
            printf("idx:%d, id: %s is %s, info: %s\r\n",
                   i,
                   thread_tables[i]->id_get().c_str(),
                   thread_tables[i]->status_get().c_str(),
                   thread_tables[i]->info_get().c_str()
                   );
        }
        printf("**************************************************\n");
        fflush(stdout);
    }

    void shutdown_all()
    {
        for(int i=0; i<thread_tables.size(); i++)
        {
            thread_tables[i]->interrupt();
            delete thread_tables[i];
        }
        thread_tables.clear();
    }

    int size()
    {
        return thread_tables.size();
    }
private:
    std::vector<ThreadContainer*> thread_tables;
};

#endif
