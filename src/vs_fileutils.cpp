#include "vs_fileutils.h"
#include <fstream>
#include <string>

bool exist(const char* file)
{
    std::ifstream fin(file);
    bool flag = fin.is_open();
    fin.close();
    return flag;
}


#ifdef WIN32

bool existDir(const char* path){
    return _access(path,0)!=-1;
}

bool writePermissionDir(const char* path){
    return _access(path,2)!=-1;
}

void createDir(const char* path){
    int t=-1;
    std::string s(path);
    while((t=s.find_first_of("\\/",t+1))!=s.npos){
        const char* sub_path = s.substr(0,t).c_str();
        if(_access(sub_path,0)==-1){
            _mkdir(sub_path);
        }
    }
}

unsigned long fileSize(const char* file)
{
    printf("TODO: fileSize\n");
    return -1;
}

unsigned long dirSize(const char* path)
{
    printf("TODO: dirSize\n");
    return -1;
}

#else //LINUX

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string.h>

bool existDir(const char* path){
    return access(path,F_OK)!=-1;
}

bool writePermissionDir(const char* path){
    return access(path,W_OK)!=-1;
}
#include <iostream>
void createDir(const char* path){
    size_t t = -1;
    std::string s(path);
    while((t = s.find_first_of("\\/",t+1)) != s.npos){
        std::string sub_path = s.substr(0,t);
        if(access(sub_path.c_str(),0) == -1){
            mkdir(sub_path.c_str(),S_IRWXU|S_IRWXG|S_IRWXO);
        }
    }
}

unsigned long fileSize(const char* file)
{
    struct stat statbuff;  
    if(stat(file, &statbuff) < 0)
        return -1;  
    else
        return statbuff.st_size;  
}

unsigned long dirSize(const char* path)
{
    DIR *dp;
    struct dirent *entry;
    struct stat statbuf;
    unsigned long dir_size = 0;

    if ((dp = opendir(path)) == NULL)
    {
        printf("[ERROR]: Cannot open dir '%s'\n", path);
        return -1;
    }
    
    lstat(path, &statbuf);
    dir_size += statbuf.st_size;

    while ((entry = readdir(dp)) != NULL)
    {
        char subdir[256] = {0};
        sprintf(subdir, "%s/%s", path, entry->d_name);
        lstat(subdir, &statbuf);
        if (S_ISDIR(statbuf.st_mode))
        {
            if (strcmp(".", entry->d_name) == 0 ||
                strcmp("..", entry->d_name) == 0)
            {
                continue;
            }
            dir_size += dirSize(subdir);
        }
        else
        {
            dir_size += statbuf.st_size;
        }
    }
    closedir(dp);    
    return dir_size;
}

bool copyFile(const char* source_file, const char* target_file)
{
    std::ifstream fin(source_file);
    std::ofstream fout(target_file);
    if(!fin.is_open() || !fout.is_open())
    {
        return false;
    }
    fout << fin.rdbuf();
    fin.close();
    fout.close();
    return true;
}
#endif