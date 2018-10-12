#ifndef __VS_DATA_SAVER_H__
#define __VS_DATA_SAVER_H__
#include <deque>
#include <thread>
#include <mutex>
#include <unistd.h>

namespace vs
{

template<class T>
class DataSaver
{
public:
    typedef std::function<void(FILE* fp, const T & t)> WriteFunction;

    DataSaver(const char* save_file, WriteFunction write_function)
        : m_exit(false), m_out_file(save_file), m_write_fun(write_function)
        , m_thread_ptr(new std::thread(std::bind(&DataSaver<T>::run, this)))
    {}

    ~DataSaver()
    {
        m_exit = true;
        join();
    }

    void push(const T& data)
    {
        m_mtx.lock();
        m_buffer.push_back(data);
        m_mtx.unlock();
    }

    void join() {m_thread_ptr->join();}

private:
    bool                                        m_exit;
    std::string                                 m_out_file;
    std::mutex                                  m_mtx;
    std::deque<T>                               m_buffer;
    std::function<void(FILE* fp, const T & t)>  m_write_fun;
    std::shared_ptr<std::thread>                m_thread_ptr;

    void run()
    {
        FILE* fp = NULL;
        while(!m_exit)
        {
            if(!fp) //open file
            {
                m_mtx.lock();
                bool has_data = !m_buffer.empty();
                m_mtx.unlock();
                if(has_data)
                {
                    fp = fopen(m_out_file.c_str(), "w");
                    if(!fp)
                    {
                        printf("[ERROR]DataSaver: Cannot open '%s', Exit.\n",
                                m_out_file.c_str());
                        m_exit = true;
                    }
                }
            }
            else //write data
            {
                m_mtx.lock();
                auto buffer = m_buffer;
                m_buffer.clear();
                m_mtx.unlock();
                if(!buffer.empty())
                {
                    for(const auto& t : buffer)
                    {
                        m_write_fun(fp, t);
                    }
                    fflush(fp);
                }
            }
            usleep(500000);
        }
        if(fp) fclose(fp);
    }
};

} /* namespace vs */
#endif//__VS_DATA_SAVER_H__