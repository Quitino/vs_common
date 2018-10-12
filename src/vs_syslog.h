#ifndef __VS_SYSLOG_H__
#define __VS_SYSLOG_H__
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <mutex>
#include "vs_tictoc.h"
#include "vs_singleton.h"

namespace vs
{

class SysLogger
{
public:
    SysLogger();

    /** \brief init log, create log dir and log file*/
    bool init(const char* logdir = NULL, size_t max_dir_size = 4e9,
              const char* syslogdir = NULL, size_t max_syslog_size = 1e9);

    /** \brief get current log dir.*/
    const char* path() const {return m_path.c_str();}

    /** \brief get system log fp.*/
    FILE* fp() {return m_fp_sys;}

    bool ok() {return m_fp_sys;}

private:
    std::string m_log_dir;  // data log dir
    std::string m_fcnt;     // log index for next data
    std::string m_path;     // cur log dir, 'log_dir/xx' where 'xx' is a number index
    std::mutex m_mtx_logcnt;// mutex for log_count file
    FILE* m_fp_sys; // current log file fp

    /* read current log count from p_log_count_file */
    int readLogCount();

    /* write current log count to p_log_count_file */
    void writeLogCount(int log_count);

    /* combile log_count to log path */
    std::string combileLogPath(int log_count);

    /* make log path, read log_count and update it by +1, create new log dir*/
    std::string makeLogPath();

    /** \brief clean old data in log dir*/
    void cleanLogDir(size_t max_dir_size);
};

typedef Singleton<SysLogger> SysLog;

/** \brief init log, create log dir and log file*/
inline bool initSysLog(const char* logdir = NULL, size_t max_dir_size = 4e9,
                const char* syslogdir = NULL, size_t max_syslog_size = 1e9)
{
    return SysLog::instance()
            ->init(logdir, max_dir_size, syslogdir, max_syslog_size);
}

} /* namespace vs */

#define VS_SYSLOG(format, args...) do {\
                FILE* fp = vs::SysLog::instance()->fp();\
                if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                fprintf(fp, format, ##args); fflush(fp); } } while(0)

#define VS_SYSLOGP(format, args...) do { printf(format, ##args);\
                FILE* fp = vs::SysLog::instance()->fp();\
                if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                fprintf(fp, format, ##args); fflush(fp); } } while(0)

#ifdef __linux__
#define VS_SYSLOG_COLOR_RED_BEG "\033[49;31m"
#define VS_SYSLOG_COLOR_DARKGREEN_BEG "\033[49;36m"
#define VS_SYSLOG_COLOR_END "\033[0m"
#else
#define VS_SYSLOG_COLOR_RED_BEG
#define VS_SYSLOG_COLOR_DARKGREEN_BEG
#define VS_SYSLOG_COLOR_END
#endif

#define VS_SYSPRINTE(format, args...) do {\
                    printf("[" VS_SYSLOG_COLOR_RED_BEG "ERROR"\
                    VS_SYSLOG_COLOR_END "]" format, ##args); } while(0)

#define VS_SYSPRINTW(format, args...) do {\
                    printf("[" VS_SYSLOG_COLOR_DARKGREEN_BEG "WARN"\
                    VS_SYSLOG_COLOR_END "]" format, ##args); } while(0)


#define VS_SYSLOGE(format, args...) do {\
                        FILE* fp = vs::SysLog::instance()->fp();\
                        if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                        fprintf(fp, "[ERROR]" format, ##args); fflush(fp); }\
                    } while(0)

#define VS_SYSLOGEP(format, args...) do {\
                        VS_SYSPRINTE(format, ##args);\
                        FILE* fp = vs::SysLog::instance()->fp();\
                        if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                        fprintf(fp, "[ERROR]" format, ##args); fflush(fp); }\
                    } while(0)

#define VS_SYSLOGW(format, args...) do {\
                        FILE* fp = vs::SysLog::instance()->fp();\
                        if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                        fprintf(fp, "[WARN]" format, ##args); fflush(fp); }\
                    } while(0)

#define VS_SYSLOGWP(format, args...) do{\
                        VS_SYSPRINTW(format, ##args);\
                        FILE* fp = vs::SysLog::instance()->fp();\
                        if(fp) {fprintf(fp, "[%f]", vs::getSoftTs());\
                        fprintf(fp, "[WARN]" format, ##args); fflush(fp); }\
                    } while(0)

#endif//__VS_SYSLOG_H__