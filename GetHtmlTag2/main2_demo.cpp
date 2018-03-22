
#include <iostream>
#include "get_html_tags2.h"
//v2.0
using namespace std;

void help()
{
	printf("\ncommand: getTag -f html \"select class from div\"\n\n");
	printf("\t-f file\n");
}


int main(int argc, char * argv[])
{
	/*int argc = 4;
	char * argv[] = { "get_html_tags2", "-f", "D:\\curl-workspace\\zuidaima\\login_index.html", "select inner_text from div where class='content'"};
*/
	try {
		if (argc < 2)
			help();

		string sql, html_src;

		for (int i = 1; i < argc;) {
			if (!strcmp(argv[i], "-f")) {
				html_src = argv[++i];
				++i;
			}
			else {
				sql = argv[i];
				++i;
			}
		}

		string html;
		if (!readFile(html_src, html)) {
			return 1;
			goto exit;
		}


		vector<string> select_val;
		vector<vector<string>> v2 = executeSql(sql, html, select_val);
		for (size_t i = 0; i < select_val.size(); i++)
		{
			if (i>0)cout << ",";
			cout << select_val[i];
		}
		cout << endl;
		for (size_t i = 0; i < v2.size(); i++)
		{
			for (size_t j = 0; j < v2[i].size(); j++)
			{
				if (j>0)cout << ",";
				cout << v2[i][j];
			}
			cout << endl;
		}
	}
	catch (exception e) {
		cout << "error!" << endl;
		return 1;
	}
exit:
	return 0;
}
