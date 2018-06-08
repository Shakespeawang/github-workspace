//
//
//#include <iostream>
//#include "get_html_tags2.h"
//using namespace std;
//
//int main()
//{
//
//	/*提取html 标签目标：
//		1.能提取出某一标签，如果未指定唯一标签，则提取出所有指定的标签，find tags : output the all tags meeting the needs.
//		2.能提取出某一标签的某一属性的值，包括标签的内容
//		3.按需提取出某一标签的所有子标签的属性、值和内容*/
//	string html;
//	string filename = "D:\\curl-workspace\\zuidaima\\login_index.html";
//	readFile(filename, html);
//	preprocess(html);
//	
//	while (true)
//	{
//		string sql;
//		char csql[3000];
//		gets(csql);
//		cin.ignore();
//		sql = csql;
//
//		vector<string> select_val;
//		vector<vector<string>> v2 = executeSql(sql, html, select_val);
//		for (size_t i = 0; i < select_val.size(); i++)
//		{
//			cout << select_val[i] << "\t";
//		}
//		cout << endl;
//		for (size_t i = 0; i < v2.size(); i++)
//		{
//			for (size_t j = 0; j < v2[i].size(); j++)
//			{
//				if (j>0)cout << ", ";
//				cout << v2[i][j];
//			}
//			cout << endl;
//		}
//	}
//
//	return 0;
//}