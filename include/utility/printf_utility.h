#ifndef PRINTF_UTILITY_H
#define PRINTF__UTILITY_H
#include <iostream>
#include <list>
#include <string>

using namespace std;

// 颜色
#define RED "\033[0;1;31m"
#define GREEN "\033[0;1;32m"
#define YELLOW "\033[0;1;33m"
#define BLUE "\033[0;1;34m"
#define PURPLE "\033[0;1;35m"
#define DEEPGREEN "\033[0;1;36m"
#define WHITE "\033[0;1;37m"

// 指针
#define NO_POINTER "\033[?25l"
#define POINTER "\033[?25h"

/*  打印标题  */
void print_title(std::string title_name, std::vector<string> title_content)
{
    std::string title_sign = "*************************************************************";
    std::string title_space = "";
    std::string title;
    std::string choice;
    int choice_num = 0;
    int title_sign_size = title_sign.size();
    int title_name_size = title_name.size();
    float space_num = (title_sign_size - title_name_size) / 2.0;
    for (int i = 0; i < floor(space_num-1); i++)
    {
        title_space.append(" ");
    }
    std::string tmp_space = (space_num - floor(space_num) > 0.1) ? " " : "";
    title = title_sign + "\n" + "*" + title_space + title_name + title_space + tmp_space + 
        "*" + "\n" + title_sign + "\n";
    for (auto item = title_content.begin(); item != title_content.end(); ++item)
    {
        choice = choice + to_string(choice_num) + ". " + *item + "\n";
        choice_num++;
    }
    cout << GREEN << title << WHITE << endl;
    cout << choice << endl;
}

void Error(string msg)
{
    cout << RED << "[ERROR] " + msg << WHITE << endl;
}

void Warning(string msg)
{
    cout << YELLOW << "[WARNING] " + msg << WHITE << endl;
}

void Info(string msg)
{
    cout << GREEN << "[INFO] " + msg << WHITE << endl;
}
#endif