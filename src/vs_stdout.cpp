#include "vs_stdout.h"
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#ifdef WIN32
#include <Windows.h>

namespace vs
{

void coutColor( const std::string& info, int color/*=WHITE*/ )
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), color);
    std::cout<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_WHITE);
}

void coutError(const std::string& module, const std::string& info)
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_RED);
    printf("[ERROR] ");
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_DARKRED);
    std::cout<<module<<":"<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_WHITE);

}
void coutWarn(const std::string& module, const std::string& info)
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_GREEN);
    printf("[WARN] ");
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_DARKGREEN);
    std::cout<<module<<":"<<info<<std::endl;
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_WHITE);
}

}
#else
/*
    \033[0m close all attributes
    \033[1m highlight
    \033[2m half light
    \033[3m italic
    \033[4m underline
    \033[5m twinkle
    \033[6m quick twinkle
    \033[7m anti display
    \033[8m blanking
    \033[9m middle line
    10-19 font correlation
    21-29 opposite to 1-9
    30-37 set foreground color
    40-47 set background color
    30: black
    31: red
    32: green
    33: yellow
    34: blue
    35: purple
    36: dark green
    37: white
    38: enable underline, set default foreground color
    39: disable underline, set default foreground color
    40: black background color
    41: red background color
    42: green background color
    43: brown background color
    44: blue background color
    45: pinkish red background color
    46: peacock blue background color
    47: white background color
    49: set default background color
    50-89: reserved
    90-109: set foreground color
    \033[nA: cursor up n lines
    \033[nB: cursor down n lines
    \033[nC: cursor right n lines
    \033[nD: cursor left n lines
    \033[y;xH: set cursor position
    \033[2J: clear screen
    \033[K: clear content from cursor to end of line
    \033[s: save cursor position
    \033[u: revert cursor postion
    \033[?25l: hide cursor
    \033[?25h: show cursor*/

namespace vs
{

void coutColor(const std::string& info, int color/*=WHITE*/){
    switch(color){
        case COLOR_RED:         printf("\033[49;31m%s \033[0m", info.c_str()); break;
        case COLOR_GREEN:       printf("\033[49;32m%s \033[0m", info.c_str()); break;
        case COLOR_YELLOW:      printf("\033[49;33m%s \033[0m", info.c_str()); break;
        case COLOR_BLUE:        printf("\033[49;34m%s \033[0m", info.c_str()); break;
        case COLOR_PINK:        printf("\033[49;35m%s \033[0m", info.c_str()); break;
        case COLOR_DARKGREEN:   printf("\033[49;36m%s \033[0m", info.c_str()); break;
        case COLOR_WHITE:       printf("\033[49;37m%s \033[0m", info.c_str()); break;
        default: printf("%s", info.c_str()); break;
    }
    printf("\n");
}

void coutError(const std::string& module, const std::string& info)
{
    printf("\033[40;31m[ERROR] \033[0m");
    std::cout<<module<<":"<<info<<std::endl;

}
void coutWarn(const std::string& module, const std::string& info)
{
    printf("\033[40;32m[WARN] \033[0m");
    std::cout<<module<<":"<<info<<std::endl;
}

}
#endif

