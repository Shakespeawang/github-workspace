///*******************************************
//1)根据文件名读取文本文件，存储到string中
//2)从表格里提取一条信息
//3)添加到到csv文件中
//********************************************/
//
//#include <iostream>
//#include "get_html_tags.h"
//
//using namespace std;
//
//void help()
//{
//	printf("\ncommand: getHtmlTag -f html -t form -r method=post&action=/Home/Login -o class\n\n");
//	printf("\t-f file\n");
//	printf("\t-t tag\n");
//	printf("\t-r restrict\n");
//	printf("\t-o output value\n");
//}
//
///*
//	从网页中根据tag标签和内容name或者id提取出href、value或者content等等
//*/
//int main1(int argcf, char * argfv[])
//{
//	int argc = 9;
//	char * argv[] = { "getHtmlTag", "-f", "D:\\curl-workspace\\zuidaima\\login_index.html", "-t", "div", "-r", "class=content",
//		"-o", "HTML"};
//
//	try {
//		if (argc < 2)
//			help();
//
//		bool isFromFile = false;
//		string tags, restricts, out_tags, html_src;
//		tags = restricts = out_tags = "";
//
//		for (int i = 1; i < argc;) {
//			if (!strcmp(argv[i], "-f")) {
//				isFromFile = true;
//				html_src = argv[++i];
//				++i;
//			}
//			else if (!strcmp(argv[i], "-t")) {
//				tags = argv[++i];
//				++i;
//			}
//			else if (!strcmp(argv[i], "-r")) {
//				restricts = argv[++i];
//				++i;
//			}
//			else if (!strcmp(argv[i], "-o")) {
//				out_tags = argv[++i];
//				++i;
//			}
//			else if (!strcmp(argv[i], "--h") || !strcmp(argv[i], "--help")) {
//				help();
//				return 0;
//			}
//			else {
//				html_src = argv[i];
//				++i;
//			}
//		}
//
//		string html;
//		if (isFromFile) {
//			if (!readFile(html_src, html)) {
//				return 1;
//				goto exit;
//			}
//		}
//		else{
//			html = html_src;
//		}
//		//cout << html;
//		cout << getHtmlTag(html, tags, restricts, out_tags);
//	}
//	catch (exception e) {
//		cout << "run error!" << endl;
//		return 1;
//	}
//exit:
//	return 0;
//}
