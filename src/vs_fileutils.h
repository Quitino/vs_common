#ifndef __VS_FILEUTILS_H__
#define __VS_FILEUTILS_H__

/** \brief judge a file whether exist.*/
bool exist(const char* file);

/** \brief judge a directory whether exist.*/
bool existDir(const char* path);

/** \brief judge a directory whether write permission.*/
bool writePermissionDir(const char* path);

/** \brief create a directory.*/
void createDir(const char* path);

/** \brief get file size [Byte].*/
unsigned long fileSize(const char* file);

/** \brief get dir size [Byte].*/
unsigned long dirSize(const char* path);

bool copyFile(const char* source_file, const char* target_file);

// std::vector<std::string> listDir(const std::string & path);

#endif