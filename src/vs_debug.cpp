#include "vs_debug.h"
#include <signal.h>
#include <execinfo.h>
#include <stdio.h>
#include <cstring>
#include <string>
#include <stdlib.h>

namespace vs
{

static std::string p_log_file;

static void seg_fault_callback(int signal, siginfo_t *si, void *arg)
{
    printf("[ERROR]Catch a segmetation fault error.\n");
    void *array[10];
    char **strings;
    size_t i;
    size_t size = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    printf("************* VS_DEBUG: Catch Seg Fault **************\n");
    printf("Signal:%d Error Code:%d Address:%p\n", signal, si->si_errno, si->si_addr);
    printf("PID:%d UID:%d\n", (int)(si->si_pid), (int)(si->si_uid));
    printf("Calling Stack(%zd):\n", size);
    for(i = 0; i < size; i++) printf(">> %s\n", strings[i]);
    printf("*******************************************************\n");

    if(p_log_file.size() > 0)
    {
        FILE* fp = NULL;
        fp = fopen(p_log_file.c_str(), "w");
        if(fp)
        {
            fprintf(fp, "************* VS_DEBUG: Catch Seg Fault **************\n");
            fprintf(fp, "Signal:%d Error Code:%d Address:%p\n", signal, si->si_errno, si->si_addr);
            fprintf(fp, "PID:%d UID:%d\n", (int)(si->si_pid), (int)(si->si_uid));
            fprintf(fp, "Calling Stack(%zd):\n", size);
            for(i = 0; i < size; i++) fprintf(fp, ">> %s\n", strings[i]);
            fprintf(fp, "*******************************************************\n");
            fclose(fp);
        }
    }
    free(strings);
    exit(0);
}

void openSegFaultDebug(const char* log_file)
{
    if(log_file)
    {
        p_log_file = std::string(log_file);
    }
    struct sigaction sa;
    memset(&sa, 0, sizeof(struct sigaction));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = seg_fault_callback;
    sa.sa_flags   = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, NULL);
}

} /* namespace vs */