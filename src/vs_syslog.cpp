#include "vs_syslog.h"
#include <fstream>
#include <mutex>
#include "vs_fileutils.h"

FILE* g_fp_sys = NULL;
std::string g_log_dir;

static std::string p_log_dir = std::string(getHomePath())+"/data/log/";
static std::string p_log_count_file = p_log_dir+"/log_count.txt";
static std::mutex p_mtx_logcount;

void setLogDir(const char* logdir)
{
    p_log_dir = std::string(logdir);
    p_log_count_file = p_log_dir+"/log_count.txt";
}

const char* getHomePath()
{
    return getenv("HOME");
}

/* read current log count from p_log_count_file */
static int readLogCount()
{
    p_mtx_logcount.lock();
    int log_count = -1;
    std::ifstream fin(p_log_count_file.c_str());
    if(fin.is_open())
        fin>>log_count;
    fin.close();
    p_mtx_logcount.unlock();

    if(log_count>1000000) {log_count = -1;}    
    return log_count;
}

/* write current log count to p_log_count_file */
static void writeLogCount(int log_count)
{
    p_mtx_logcount.lock();
    std::ofstream fout(p_log_count_file.c_str());
    fout<<log_count;
    fout.close();
    p_mtx_logcount.unlock();
}

/* combile log_count to log path */
static std::string combileLogPath(int log_count)
{
    char t[256] = {0};
    sprintf(t, "%s/%d/", p_log_dir.c_str(), log_count);
    return std::string(t);
}

/* make log path, read log_count and update it by +1, create new log dir*/
static std::string vsMakeLogPath()
{
    if(!existDir(p_log_dir.c_str()))
        createDir(p_log_dir.c_str());

    int log_count = readLogCount();
    if(log_count<0)
        log_count = 0;
    else
        log_count++;

    std::string new_path = combileLogPath(log_count);
    if(!existDir(new_path.c_str()))
        createDir(new_path.c_str());
    writeLogCount(log_count);
    return new_path;
}

/** \brief clean old data in log dir*/
static void vsCleanLogDir()
{
    if(!existDir(p_log_dir.c_str())) return;
    const unsigned long max_dir_size = 4e9; //4GB
    unsigned long cur_dir_size = dirSize(p_log_dir.c_str());
    if(cur_dir_size > max_dir_size)
    {
        int cur_cnt = readLogCount();
        int start_cnt = 0;
        for(; start_cnt<cur_cnt; start_cnt++)
        {
            std::string t = combileLogPath(start_cnt);
            const char* tdir = t.c_str();
            if(existDir(tdir))
                break;
        }
        unsigned long free_size = cur_dir_size - max_dir_size/2;
        unsigned long sum_size = 0;
        for(int i=start_cnt; i<cur_cnt-10; i++)
        {
            std::string t = combileLogPath(i);
            const char* tdir = t.c_str();
            if(existDir(tdir))
            {
                unsigned long s = dirSize(tdir);
                char cmd[256] = {0};
                sprintf(cmd, "rm -rf %s", tdir);
                int res = system(cmd);
                res = res;
                sum_size += s;
                if(sum_size > free_size)
                {
                    printf("[WARN] No enougn disk space. Remove data dir(%d - %d), total free %.2f MB\n",
                                    start_cnt, i, (double)sum_size/1024.0/1024.0);
                    break;
                }
            }
        }
    }
}

bool initLog(const char* logdir)
{
    if(logdir)
        setLogDir(logdir);
    g_log_dir = vsMakeLogPath();
    vsCleanLogDir();
    g_fp_sys = fopen((g_log_dir+"/system.log").c_str(),"w");
    return g_fp_sys;
}

const char* vsGetLogPath()
{
    return g_log_dir.c_str();
}

void writeFile(const char* file, const char* content, const char* mode)
{
    FILE* fp = fopen(file, mode);
    if(!fp)
    {
        printf("[ERROR] open file '%s' failed.\n", file);
        return;
    }
    fprintf(fp, "%s\n", content);
    fclose(fp);
}