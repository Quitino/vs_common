#ifndef __VS_SYSLOG_H__
#define __VS_SYSLOG_H__
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "vs_tictoc.h"

extern std::string g_log_dir;
extern FILE* g_fp_sys;

/** \brief init log, create log dir and log file*/
bool initLog(const char* logdir = NULL);

/** \brief get home path in linux.*/
const char* getHomePath();

/** \brief get current log dir.*/
const char* vsGetLogPath();

/** \brief write content to a file.*/
void writeFile(const char* file, const char* content, const char* mode="w");


#define VS_SYSLOG(format, args...) do{if(g_fp_sys){fprintf(g_fp_sys,"[%f]",getCurTimestamp()); \
                                fprintf(g_fp_sys, format, ##args);fflush(g_fp_sys);}}while(0)

#define VS_SYSLOGP(format, args...) do{printf(format, ##args);if(g_fp_sys){\
                                fprintf(g_fp_sys,"[%f]",getCurTimestamp());\
                                fprintf(g_fp_sys, format, ##args);fflush(g_fp_sys);}}while(0)

#endif//__VS_SYSLOG_H__