#include "vs_syslog.h"
#include "vs_os.h"
#include <fstream>
#include <mutex>
#include <algorithm>

namespace vs
{

SysLogger::SysLogger(): m_fp_sys(NULL) {}

bool SysLogger::init(const char* logdir, size_t max_dir_size,
                     const char* syslogdir, size_t max_syslog_size)
{
    if(logdir)
    {
        m_log_dir = std::string(logdir);
        m_fcnt = m_log_dir + "/log_count.txt";
    }
    makedirs(m_log_dir.c_str());
    cleanLogDir(max_dir_size);
    m_path = makeLogPath();

    if(syslogdir)
    {
        makedirs(syslogdir);
        // clean
        auto files = listdir(syslogdir, 1, 0);
        if(!files.empty())
        {
            std::sort(files.begin(), files.end());
            double sum_size = 0;
            for(int i = files.size() - 1; i >= 0; i--)
            {
                const auto& f = files[i];
                if(suffix(f.c_str()) != ".log") continue;
                if(sum_size > max_syslog_size)
                    std::remove(f.c_str());
                else
                    sum_size += filesize(f.c_str());
            }
        }
        int log_count = readLogCount();
        char t[128] = {0};
        snprintf(t, 128, "%s/system_%04d.log", syslogdir, log_count);
        m_fp_sys = fopen(t, "w");
    }
    else
    {
        m_fp_sys = fopen(join(m_path, "system.log").c_str(), "w");
    }
    return m_fp_sys;
}

int SysLogger::readLogCount()
{
    m_mtx_logcnt.lock();
    int log_count = -1;
    std::ifstream fin(m_fcnt.c_str());
    if(fin.is_open()) fin >> log_count;
    fin.close();
    m_mtx_logcnt.unlock();
    return log_count;
}

void SysLogger::writeLogCount(int log_count)
{
    m_mtx_logcnt.lock();
    std::ofstream fout(m_fcnt.c_str());
    fout << log_count;
    fout.close();
    m_mtx_logcnt.unlock();
}

std::string SysLogger::combileLogPath(int log_count)
{
    char t[256] = {0};
    snprintf(t, sizeof(t), "%s/%d/", m_log_dir.c_str(), log_count);
    return std::string(t);
}

std::string SysLogger::makeLogPath()
{
    makedirs(m_log_dir.c_str());
    int log_count = readLogCount();
    if(log_count < 0) log_count = 0;
    else log_count++;

    std::string new_path = combileLogPath(log_count);
    makedirs(new_path.c_str());
    if(exists(new_path.c_str()))
        writeLogCount(log_count);
    return new_path;
}

void SysLogger::cleanLogDir(size_t max_dir_size)
{
    if(!exists(m_log_dir.c_str())) return;
    size_t cur_dir_size = dirsize(m_log_dir.c_str());
    if(cur_dir_size > max_dir_size)
    {
        int cur_cnt = readLogCount();
        int start_cnt = 0;
        for(; start_cnt < cur_cnt; start_cnt++)
        {
            std::string t = combileLogPath(start_cnt);
            const char* tdir = t.c_str();
            if(exists(tdir))
                break;
        }
        size_t free_size = cur_dir_size - max_dir_size / 2;
        size_t sum_size = 0;
        for(int i = start_cnt; i < cur_cnt - 3; i++)
        {
            std::string t = combileLogPath(i);
            const char* tdir = t.c_str();
            if(exists(tdir))
            {
                size_t s = dirsize(tdir);
                char cmd[256] = {0};
                snprintf(cmd, sizeof(cmd), "rm -rf %s", tdir);
                int res = system(cmd);
                res = res;
                sum_size += s;
                if(sum_size > free_size)
                {
                    printf("[WARN] No enougn disk space. Remove data dir(%d - %d), "
                            "total free %.2f MB\n",
                             start_cnt, i, (double)sum_size / 1024.0 / 1024.0);
                    break;
                }
            }
        }
    }
}

} /* namespace vs */