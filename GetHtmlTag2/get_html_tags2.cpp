#include "get_html_tags2.h"


/*
*	'<'、'>'和'='前后的空格符都去掉
*/
void eraseBlank(string &html, const char ch)
{
	int found = -1;
	while (true)
	{
		found = html.find(ch, found + 1);
		if (string::npos == found)
			return;
		int i = found + 1;
		while (i < html.length()){
			if (isBlank(html[i]))
				html.erase(i, 1);
			else
				break;
		}
		i = i - 2;
		while (i >= 0){
			if (isBlank(html[i]))
				html.erase(i--, 1);
			else
				break;
		}
	}
}
void preprocess(string & html)
{
	eraseBlank(html, '<');
	eraseBlank(html, '>');
	eraseBlank(html, '=');
	eraseBlank(html, ':');
	eraseBlank(html, ';');
	eraseBlank(html, ',');
}
int find_tag_end_pos(string html, int start, string tags)
{
	int found = start - 1;
	int cnt = 0;
	while (true)
	{
		//cout << html.substr(found, tags.length() + 20) << endl;
		int p1 = html.find("<" + tags + " ", found + 1);
		int p2 = html.find("<" + tags + ">", found + 1);
		int p3 = html.find("</" + tags + ">", found + 1);

		if (-1 == p1 && -1 == p2)
			break;
		else if (-1 == p1 && -1 != p2)
			found = p2;
		else if (-1 != p1 && -1 == p2)
			found = p1;
		else
			found = p1 < p2 ? p1 : p2;

		if (-1 == p3){
			int idx = html.find(">", found + 1);
			if (-1 == idx)
				return html.length();
			else
				return idx + 1;

		}

		if (-1 != p3 && p3 < found){
			//cout << html.substr(p3, tags.length() + 20) << endl;
			cnt--;
			found = p3;
			if (cnt <= 0)
				break;
		}
		else
			cnt++;
		found += tags.length() + 1;
	}
	while (cnt>0){
		found = html.find("</" + tags + ">", found + 1);
		if (-1 == found)
			return -1;
		found += tags.length() + 3;
		cnt--;
	}
	return found;
}
bool match_need(string scheck, string conditions)
{
	int found = -1;
	int pos = 0;
	while (true)//去除引号
	{
		found = conditions.find("&", found + 1);
		if (-1 == found)
			break;
		string str = conditions.substr(pos, found - pos);
		pos = found + 1;
		if (-1 == scheck.find(str))
			return false;
	}

	string str = conditions.substr(pos, conditions.length() - pos);
	if (-1 == scheck.find(str))
		return false;
	else
		return true;
}
vector<string> get_tags(string html, string tags, string _conditions){
	string conditions = _conditions;
	int found = -1;
	while (true)//去除引号
	{
		found = conditions.find("'", found + 1);
		if (-1 == found)
			break;
		conditions[found] = '\"';
	}

	vector<string> tags_html;
	found = -1;
	while (true)
	{
		int p1 = html.find("<" + tags + " ", found + 1);
		int p2 = html.find("<" + tags + ">", found + 1);

		if (-1 == p1 && -1 == p2)
			break;
		else if (-1 == p1 && -1 != p2)
			found = p2;
		else if (-1 != p1 && -1 == p2)
			found = p1;
		else
			found = p1 < p2 ? p1 : p2;

		int pos2 = html.find(">", found + tags.length() + 1);
		if (string::npos == pos2)
			break;
		string scheck = html.substr(found, pos2 - found + 1);
		int idx = -1;
		if (match_need(scheck, conditions))//如果满足要求，则查找结束位置
		{
			idx = find_tag_end_pos(html, found, tags);

			if (string::npos == idx)
				goto tail;
			else{
				string str = html.substr(found, idx - found);
				tags_html.push_back(str);
			}

		}
	tail:
		if (-1 == idx)
			found += tags.length() + 1;
		else
			found = idx;
	}
	return tags_html;
}

vector<vector<string>> get_target_value(vector<string> target, vector<string> condition)
{
	vector<vector<string>>v2;
	for (size_t k = 0; k < target.size(); k++)
	{
		string str = target[k];
		int idx = str.find(">");
		string tag_head = str.substr(0, idx + 1);
		vector<string>v;
		for (size_t i = 0; i < condition.size(); i++)
		{
			if ("inner_text" == condition[i]){
				int pos1 = str.find(">", 0);
				int pos2 = str.find_last_of("</");
				str.erase(pos2 - 1, str.length() - pos2 - 1);
				str.erase(0, pos1 + 1);

				pos1 = str.find("<", 0);
				pos2 = str.find_last_of(">");
				str.erase(pos1, pos2 - pos1 + 1);
				v.push_back(str);
			}
			else{
				int pos1 = tag_head.find(condition[i] + "=");
				if (-1 == pos1){
					v.push_back("");
					continue;
				}
				int pos2 = tag_head.find("\"", pos1 + condition[i].length() + 2);
				if (-1 == pos2){
					pos2 = tag_head.find(" ", pos1);
					if (-1 == pos2){
						pos2 = tag_head.find("/", pos1);
						if (-1 == pos2)
							pos2 = tag_head.find(">", pos1);
					}
				}
				if (-1 == pos2){
					v.push_back("");
					continue;
				}
				int pos3 = pos1 + condition[i].length() + 1;
				string value = tag_head.substr(pos3, pos2 - pos3);
				for (int j = value.size() - 1; j >= 0; j--)
				{
					if ('"' == value[j])
						value.erase(j, 1);
				}
				v.push_back(value);
			}
		}
		v2.push_back(v);
	}
	return v2;
}
void sql_preprocess(string & sql)
{
	for (size_t i = 0; i < sql.length(); i++)
	{
		if (sql[i] >= 'A' && sql[i] <= 'Z')
			sql[i] = sql[i] - 'A' + 'a';
	}
}
void sql2condition(string sql, vector<string> & select_val, string & from_val, string & where_val)
{
	// select action,class from form where name='pwd'&id='pwd' in select * from form where name="";
	eraseBlank(sql, ',');
	eraseBlank(sql, ' ');
	sql = " " + sql;
	int pos1 = sql.find(" select ");
	int pos2 = sql.find(" ", pos1 + 8);
	string s_select = sql.substr(pos1 + 8, pos2 - pos1 - 8);
	int found = -1;
	int pos = 0;
	while (true)
	{
		found = s_select.find(",", found + 1);
		if (-1 == found)
			break;
		string str = s_select.substr(pos, found);
		pos = found;
		select_val.push_back(str);
	}
	string str = s_select.substr(pos ? pos + 1 : pos, s_select.length() - pos);
	pos = found;
	select_val.push_back(str);

	pos1 = sql.find(" from ");
	pos2 = sql.find(" ", pos1 + 6);
	from_val = sql.substr(pos1 + 6, pos2 - pos1 - 6);

	pos1 = sql.find(" where ");
	if (-1 == pos1){
		where_val = "";
		for (size_t i = 0; i < select_val.size(); i++)
		{
			if (i>0)where_val += "&";
			where_val += select_val[i];
		}
	}
	else{
		pos2 = sql.find(" ", pos1 + 7);
		if (-1 == pos2)
			pos2 = sql.length();
		where_val = sql.substr(pos1 + 7, pos2 - pos1 - 7);
	}
}

vector<vector<string>> executeSql(string sql, string html, vector<string> & select_val)
{
	string from_val, where_val;
	sql_preprocess(sql);
	int idx = sql.find(" in ");
	if (-1 != idx){
		vector<string>select_val;

		string sql1 = sql.substr(0, idx);
		string sql2 = sql.substr(idx + 4, sql.length() - idx - 4);

		sql2condition(sql2, select_val, from_val, where_val);
		vector<string> v = get_tags(html, from_val, where_val);
		string tag_html = "";
		for (size_t i = 0; i < v.size(); i++)
		{
			tag_html += v[i];
		}
		html = tag_html;
		sql = sql1;
	}

	sql2condition(sql, select_val, from_val, where_val);
	vector<string> v = get_tags(html, from_val, where_val);
	return get_target_value(v, select_val);
}


bool isBlank(char ch)
{
	return ' ' == ch || '\t' == ch;
}



bool writeFile(string filename, string record)
{
	FILE * fp = fopen(filename.c_str(), "a+");
	if (!fp) {
		cout << "Fail to open file!" << endl;
		return false;
	}
	fprintf(fp, "%s\n", record.c_str());
	fclose(fp);
	return true;
}

//changeTxtEncoding修改字符串的编码  
char* changeTxtEncoding(char* szU8)
{
	int wcsLen = ::MultiByteToWideChar(CP_UTF8, NULL, szU8, strlen(szU8), NULL, 0);
	wchar_t* wszString = new wchar_t[wcsLen + 1];
	::MultiByteToWideChar(CP_UTF8, NULL, szU8, strlen(szU8), wszString, wcsLen);
	wszString[wcsLen] = '\0';
	//cout << wszString << endl;

	int ansiLen = ::WideCharToMultiByte(CP_ACP, NULL, wszString, wcslen(wszString), NULL, 0, NULL, NULL);
	char* szAnsi = new char[ansiLen + 1];
	::WideCharToMultiByte(CP_ACP, NULL, wszString, wcslen(wszString), szAnsi, ansiLen, NULL, NULL);
	szAnsi[ansiLen] = '\0';
	return szAnsi;
}

bool readFile(string filename, string & html)
{
	ifstream infile;
	string strLine = "";
	string strResult = "";
	infile.open(filename);
	if (!infile.is_open())
		return false;
	if (infile)
	{
		while (!infile.eof()) {
			getline(infile, strLine);
			strResult += strLine + "\n";
		}
	}
	infile.close();
	char* changeTemp = new char[strResult.length() + 6];
	memset(changeTemp, 0, strResult.length() + 6);
	strncpy(changeTemp, strResult.c_str(), strResult.length());
	char* changeResult = changeTxtEncoding(changeTemp);
	strResult = changeResult;
	html = strResult;
	return true;
}
//changeTextFromUtf8ToAnsi读取UTF-8格式的文件并将之保存为ANSI格式的文件  
void changeTextFromUtf8ToAnsi(const char* filename)
{
	ifstream infile; string strLine = ""; string strResult = "";
	infile.open(filename);
	if (infile)
	{
		while (!infile.eof()) {
			getline(infile, strLine);
			strResult += strLine + "\n";
		}
	}
	infile.close();
	char* changeTemp = new char[strResult.length()];
	strcpy(changeTemp, strResult.c_str());
	char* changeResult = changeTxtEncoding(changeTemp);
	strResult = changeResult;
	ofstream outfile;
	outfile.open("ANSI.txt");
	outfile.write(strResult.c_str(), strResult.length());
	outfile.flush();
	outfile.close();
}
wstring s2ws(const string& s)
{
	_bstr_t t = s.c_str();
	wchar_t* pwchar = (wchar_t*)t;
	wstring result = pwchar;
	return result;
}
string ws2s(const wstring& ws)
{
	string curLocale = setlocale(LC_ALL, NULL); // curLocale = "C";  

	setlocale(LC_ALL, "chs");

	const wchar_t* _Source = ws.c_str();
	size_t _Dsize = 2 * ws.size() + 1;
	char *_Dest = new char[_Dsize];
	memset(_Dest, 0, _Dsize);
	wcstombs(_Dest, _Source, _Dsize);
	string result = _Dest;
	delete[]_Dest;

	setlocale(LC_ALL, curLocale.c_str());

	return result;
}

string int2String(int a, bool flag)
{
	if (flag && a < 10 && a >= 0)
		return "0" + to_string(a);
	else
		return to_string(a);
}


//只拷贝低字节至string中
std::string WStringToString(const std::wstring &wstr)
{
	std::string str(wstr.length(), ' ');
	std::copy(wstr.begin(), wstr.end(), str.begin());
	return str;
}

