//
//
//#include <iostream>
//#include "get_html_tags2.h"
//using namespace std;
//
//int main()
//{
//
//	/*��ȡhtml ��ǩĿ�꣺
//		1.����ȡ��ĳһ��ǩ�����δָ��Ψһ��ǩ������ȡ������ָ���ı�ǩ��find tags : output the all tags meeting the needs.
//		2.����ȡ��ĳһ��ǩ��ĳһ���Ե�ֵ��������ǩ������
//		3.������ȡ��ĳһ��ǩ�������ӱ�ǩ�����ԡ�ֵ������*/
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