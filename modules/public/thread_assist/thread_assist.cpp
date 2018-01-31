#include "thread_assist.h"

/* -----------------------------------------------
 *
 *  Public Library
 *
 * --------------------------------------------- */

using namespace boost;
using namespace std;


int ThreadContainer::regist(int (*task)(void), char* description)
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
int ThreadContainer::regist(int (ObjT::*task)(void), ObjT& obj, char* description)
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

int ThreadContainer::join_able()
{
    if (!pthread)
    {
        return 0;
    }
    return pthread->joinable();
}

int ThreadContainer::interrupt()
{
    // boost::this_thread::interruption_point();
    // every while(true) need add below line.
    if (join_able())
    {
        time_t t_start, t_end;
        t_start = time(NULL) ;
        pthread->interrupt();
        pthread->timed_join(boost::posix_time::seconds(10));
        t_end = time(NULL) ;
        if((t_end,t_start) >= 8)
        {
            // if 5 seconds pass, thread still can't close
            status = Run;
        }
        else // normally interrupt thread
        {
            status = Stop;
        }
    }
    status = Stop;
}

string ThreadContainer::status_get()
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

/*---------------------------------------------------------------------------------*/
~ThreadAssist::ThreadAssist()
{
    for(int i=0; i<thread_tables.size(); i++)
    {
        delete thread_tables[i];
    }
}

template<typename ObjT>
int ThreadAssist::add_tables(char* description, int (ObjT::*func)(void), ObjT& obj)
{
    ThreadContainer* thread_container = new ThreadContainer();
    thread_container->regist(func, obj, description);
    thread_tables.push_back(thread_container);
    return 0;
}

int ThreadAssist::add_tables(char* description, int (*MyTask)(void) )
{
    ThreadContainer* thread_container = new ThreadContainer();
    thread_container->regist(MyTask, description);
    thread_tables.push_back(thread_container);
    return 0;
}

void ThreadAssist::show_tables()
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

void ThreadAssist::shutdown_all()
{
    for(int i=0; i<thread_tables.size(); i++)
    {
        thread_tables[i]->interrupt();
        delete thread_tables[i];
    }
    thread_tables.clear();
}

int ThreadAssist::size()
{
    return thread_tables.size();
}

#endif
