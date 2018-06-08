#ifndef __get_html_tags__h
#define __get_html_tags__h

#include <iostream>
#include <fstream>
#include <istream>
#include <string>
#include <Windows.h>
#include <locale.h>  
#include <vector>
#include <comutil.h>  

using namespace std;
#pragma comment(lib, "comsuppw.lib")  

#define _HTML			"HTML"


void preprocess(string & html);
vector<vector<string>> executeSql(string sql, string html, vector<string> & select_val);


bool isBlank(char ch);

bool readFile(string filename, string & html);
bool writeFile(string filename, string record);

bool isBlank(char ch);
string clearBlank(string str);

#endif // !__get_html_tags__h
