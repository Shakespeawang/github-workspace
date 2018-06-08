//#include "get_html_tags.h"
//
//
///*******************************************
//1)根据文件名读取文本文件，存储到string中
//2)从表格里提取一条信息
//3)添加到到csv文件中
//********************************************/
//
//
//
//string clearBlank(string str)
//{
//	string rst = "";
//	for (int i = 0; i < str.length(); i++) {
//		if (!isBlank(str[i]) && '\n' != str[i])
//			rst += str[i];
//	}
//	return rst;
//}
//bool isBlank(char ch)
//{
//	return ' ' == ch || '\t' == ch;
//}
//
//
//string getTag(string tag, string str, int * i)
//{
//	int state = 1;
//	string tagWord = "";
//	string tagCode = "";
//	bool endTag = false;
//	bool getWord = false;
//	while (*i < str.length())
//	{
//		switch (state)
//		{
//		case 1:
//			tagCode = "";
//			tagWord = "";
//			endTag = false;
//			getWord = false;
//			if ('<' == str[*i]) {
//				tagCode += str[*i];
//				state = 2;
//			}
//			break;
//		case 2:
//			tagCode += str[*i];
//			if (!getWord && isBlank(str[*i]))
//				state = 2;
//			else if ('/' == str[*i]) {
//				endTag = true;
//				state = 3;
//				tagWord = "";
//			}
//			else if (!getWord) {
//				tagWord += str[*i];
//				state = 3;
//			}
//			break;
//		case 3:
//			tagCode += str[*i];
//			if ('>' == str[*i]) {
//				state = 5;
//				if (!getWord && tag != tagWord)
//					state = 1;
//				else if (getWord && tag != tagWord) {
//					state = 2;
//					tagWord = "";
//				}
//			}
//			else if (isBlank(str[*i])) {
//				state = 4;
//				if (!getWord && tag != tagWord)
//					state = 1;
//				else if (getWord && tag != tagWord) {
//					state = 2;
//					tagWord = "";
//				}
//			}
//			else
//				tagWord += str[*i];
//			break;
//		case 4:
//			getWord = true;
//			tagCode += str[*i];
//			if ('>' == str[*i])
//				state = 5;
//			break;
//		case 5:
//			getWord = true;
//			tagCode += str[*i];
//			if (endTag && tag == tagWord) {
//				tagCode.erase(tagCode.length() - 1);
//				goto exit;
//			}
//			else if ('<' == str[*i]) {
//				state = 2;
//			}
//			break;
//		default:
//			break;
//		}
//		(*i)++;
//	}
//exit:
//	return tagCode;
//}
//
//string getTagContent(string tagCode)
//{
//	string tagContent = "";
//	int s_i = 0, e_i = 0;
//	for (s_i = 0; s_i < tagCode.length(); s_i++) {
//		if ('>' == tagCode[s_i])
//			break;
//	}
//	for (e_i = tagCode.length() - 1; e_i >= 0; e_i--) {
//		if ('<' == tagCode[e_i])
//			break;
//	}
//	if (s_i + 1 < tagCode.length() - 1 && e_i - s_i - 1 >= 0 && e_i - s_i - 1 < tagCode.length())
//	{
//		for (int i = s_i + 1; i < e_i; i++) {
//			tagContent += tagCode[i];
//		}
//	}
//	else
//		tagContent = tagCode;
//	return tagContent;
//}
//
//
//string getTopContent(string tagCode)
//{
//	string tagContent = "";
//	tagCode = clearBlank(tagCode);
//	string rst = "";
//	for (int i = 0; i < tagCode.length(); i++) {
//		if (i + 3 < tagCode.length()) {
//			if ('<' == tagCode[i]
//				&& ('b' == tagCode[i + 1] || 'B' == tagCode[i + 1])
//				&& ('r' == tagCode[i + 2] || 'R' == tagCode[i + 2])
//				&& '>' == tagCode[i + 3])
//			{
//				i += 3;
//				continue;
//			}
//		}
//		rst += tagCode[i];
//	}
//	tagCode = rst;
//	while (1) {
//		tagContent = getTagContent(tagCode);
//		/*for (int i = 0; i < tagContent.length() && i < tagCode.length(); i++) {
//		if (tagContent[i] != tagCode[i])
//		break;
//		}
//		if(i == )
//		goto exit;*/
//		if (tagCode == tagContent)
//			goto exit;
//		tagCode = tagContent;
//
//	}
//exit:
//	return tagContent;
//}
//
//
//
//string getTagValue(const string html, const string tags)
//{
//	string ret_str = "";
//	int idx = html.find(" " + tags);
//	if (idx == -1)
//		return "";
//
//	string html2 = html.substr(idx, html.length() - idx);
//	int idx2 = html2.find_first_of('=');
//	if (idx2 == -1)
//		return "";
//
//	for (int i = idx2 + 1; i < html2.length(); i++){
//		if ('"' == html2[i]){
//			for (int j = i + 1; j < html2.length(); j++){
//				if ('"' == html2[j])
//					return ret_str;
//				ret_str += html2[j];
//			}
//		}
//		else if (!isBlank(html2[i])){
//			for (int j = i; j < html2.length(); j++){
//				if (isBlank(html2[j]))
//					return ret_str;
//				ret_str += html2[j];
//			}
//		}
//	}
//	return ret_str;
//}
//
//
//string getHtmlTag(const string html, const string tags, const string restricts, const string out_tags)
//{
//	string ret_str = "";
//	for (int i = 0; i < html.length();) {
//
//out_cicle:
//		string tagCode = getTag(tags, html, &i);
//		if ("" == restricts && out_tags == _HTML)
//			return tagCode;
//		else{
//			int idx = tagCode.find_first_of('>');
//			if (idx != -1 && idx + 1 < tagCode.length()){
//				string str = tagCode.substr(0, idx + 1);
//				string restricts2 = restricts;
//				if ("" == restricts2)
//					return getTagValue(str, out_tags);
//
//				while (true){
//					string obj, obj_val, sstr;
//					bool isBreak = false;
//					int idx = restricts2.find_first_of('&');
//					if (idx == -1) {
//						sstr = restricts2;
//						isBreak = true;
//					}
//					else{
//						sstr = restricts2.substr(0, idx);
//					}
//
//					int idx2 = sstr.find_first_of('=');
//					if (idx2 == -1){
//						obj = sstr;
//					}
//					else{
//						obj = sstr.substr(0, idx2);
//						idx2 = idx2 + 1 > sstr.length() - 1 ? idx2 : idx2 + 1;
//						obj_val = sstr.substr(idx2, sstr.length() - idx2);
//					}
//					string value = getTagValue(str, obj);
//					if (value != obj_val)
//						goto out_cicle;
//					if (isBreak){
//						return "" != restricts && out_tags == _HTML ? tagCode : getTagValue(str, out_tags);
//					}
//					else{
//						int idx3 = idx + 1 > restricts2.length() - 1 ? idx : idx + 1;
//						restricts2 = restricts2.substr(idx3, restricts2.length() - 1);
//					}
//
//				}
//			}
//		}
//	}
//	return ret_str;
//}
//
//
//bool writeFile(string filename, string record)
//{
//	FILE * fp = fopen(filename.c_str(), "a+");
//	if (!fp) {
//		cout << "Fail to open file!" << endl;
//		return false;
//	}
//	fprintf(fp, "%s\n", record.c_str());
//	fclose(fp);
//	return true;
//}
//
////changeTxtEncoding修改字符串的编码  
//char* changeTxtEncoding(char* szU8)
//{
//	int wcsLen = ::MultiByteToWideChar(CP_UTF8, NULL, szU8, strlen(szU8), NULL, 0);
//	wchar_t* wszString = new wchar_t[wcsLen + 1];
//	::MultiByteToWideChar(CP_UTF8, NULL, szU8, strlen(szU8), wszString, wcsLen);
//	wszString[wcsLen] = '\0';
//	//cout << wszString << endl;
//
//	int ansiLen = ::WideCharToMultiByte(CP_ACP, NULL, wszString, wcslen(wszString), NULL, 0, NULL, NULL);
//	char* szAnsi = new char[ansiLen + 1];
//	::WideCharToMultiByte(CP_ACP, NULL, wszString, wcslen(wszString), szAnsi, ansiLen, NULL, NULL);
//	szAnsi[ansiLen] = '\0';
//	return szAnsi;
//}
//
//bool readFile(string filename, string & html)
//{
//	ifstream infile;
//	string strLine = "";
//	string strResult = "";
//	infile.open(filename);
//	if (!infile.is_open())
//		return false;
//	if (infile)
//	{
//		while (!infile.eof()) {
//			getline(infile, strLine);
//			strResult += strLine + "\n";
//		}
//	}
//	infile.close();
//	char* changeTemp = new char[strResult.length() + 6];
//	memset(changeTemp, 0, strResult.length() + 6);
//	strncpy(changeTemp, strResult.c_str(), strResult.length());
//	char* changeResult = changeTxtEncoding(changeTemp);
//	strResult = changeResult;
//	html = strResult;
//	return true;
//}
////changeTextFromUtf8ToAnsi读取UTF-8格式的文件并将之保存为ANSI格式的文件  
//void changeTextFromUtf8ToAnsi(const char* filename)
//{
//	ifstream infile; string strLine = ""; string strResult = "";
//	infile.open(filename);
//	if (infile)
//	{
//		while (!infile.eof()) {
//			getline(infile, strLine);
//			strResult += strLine + "\n";
//		}
//	}
//	infile.close();
//	char* changeTemp = new char[strResult.length()];
//	strcpy(changeTemp, strResult.c_str());
//	char* changeResult = changeTxtEncoding(changeTemp);
//	strResult = changeResult;
//	ofstream outfile;
//	outfile.open("ANSI.txt");
//	outfile.write(strResult.c_str(), strResult.length());
//	outfile.flush();
//	outfile.close();
//}
//wstring s2ws(const string& s)
//{
//	_bstr_t t = s.c_str();
//	wchar_t* pwchar = (wchar_t*)t;
//	wstring result = pwchar;
//	return result;
//}
//string ws2s(const wstring& ws)
//{
//	string curLocale = setlocale(LC_ALL, NULL); // curLocale = "C";  
//
//	setlocale(LC_ALL, "chs");
//
//	const wchar_t* _Source = ws.c_str();
//	size_t _Dsize = 2 * ws.size() + 1;
//	char *_Dest = new char[_Dsize];
//	memset(_Dest, 0, _Dsize);
//	wcstombs(_Dest, _Source, _Dsize);
//	string result = _Dest;
//	delete[]_Dest;
//
//	setlocale(LC_ALL, curLocale.c_str());
//
//	return result;
//}
//
//string int2String(int a, bool flag)
//{
//	if (flag && a < 10 && a >= 0)
//		return "0" + to_string(a);
//	else
//		return to_string(a);
//}
//
//
////只拷贝低字节至string中
//std::string WStringToString(const std::wstring &wstr)
//{
//	std::string str(wstr.length(), ' ');
//	std::copy(wstr.begin(), wstr.end(), str.begin());
//	return str;
//}
//
