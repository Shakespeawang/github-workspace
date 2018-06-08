#include <iostream>
#include <string>

#include "Base64.h"
#include "Base64.cpp"

#define BUF_SIZE 512 * 3 * 4 //设为12的倍数
using namespace std;

/*
*	usage:	MYBase64 test -o ouput.txt
*		-o:		output text	
*		-f:		read from file
*		-d:		decode base64
*		-c:		code base64
*		-u:		code by url
*		-oh:	to output hex string
*		-ih:	to read hex string
*		-h:		help
*		-H:		help
*		-help:	help
*/

void help()
{
	cout << "usage:	MYBase64 test -o ouput.txt" << endl;
	cout << "\t-o:\toutput text" << endl;
	cout << "\t-f:\tread from file" << endl;
	cout << "\t-d:\tdecode base64" << endl;
	cout << "\t-u:\tcode by url" << endl;
	cout << "\t-oh:\tto output hex string" << endl;
	cout << "\t-ih:\tto read hex string" << endl;
	cout << "\t-h:\thelp" << endl;
	cout << "\t-H:\thelp" << endl;
	cout << "\t-help:\thelp" << endl;
}

int read_bytes_from_file(unsigned char * buf, const int size_t, FILE * fp)
{
	return fread(buf, 1, size_t, fp);
}

int write_bytes_2_file(unsigned char * buf, const int size_t, FILE * fp)
{
	return fwrite(buf, 1, size_t, fp);
}

static char decimal_2_hex_char(int decimal)
{
	if (decimal >= 0 && decimal <= 9)
		return decimal + '0';
	else if (decimal > 9 && decimal <= 15)
		return decimal + 'A' - 10;
	else
		return 0;
}

static int hex_char_2_decimal(char hex)
{
	if (hex >= '0' && hex <= '9')
		return hex - '0';
	else if (hex >= 'A' && hex <= 'F')
		return hex - 'A' + 10;
	else if(hex >= 'a' && hex <= 'f')
		return hex - 'a' + 10;
	else
		return -1;
}

string bytes_to_hex_string(const unsigned char bytes[],const int len)
{
	string str_hex = "";
	for (int i = 0; i < len; i++) {
		unsigned char uch = bytes[i];
		unsigned char hex = decimal_2_hex_char((uch >> 4) & 0x0f);
		if (0 == hex)
			return "";

		str_hex += hex;

		hex = decimal_2_hex_char((uch) & 0x0f);
		if (0 == hex)
			return "";

		str_hex += hex;
	}
	return str_hex;
}

int hex_string_to_bytes(const string hex_str, unsigned char bytes[], const int bytes_size)
{
	if (bytes_size * 2 < hex_str.length())
		return 0;
	const int group = hex_str.length() / 2;
	for (int i = 0; i < group; i++) {
		char byte1 = hex_char_2_decimal(hex_str[i * 2 + 0]);
		char byte2 = hex_char_2_decimal(hex_str[i * 2 + 1]);
		if (-1 == byte1 || -1 == byte2)
			return 0;

		bytes[i] = ((byte1 << 4) & 0xf0) | (byte2 & 0x0f);
	}
	if (1 == hex_str.length() % 2) {
		char byte1 = hex_char_2_decimal(hex_str[group * 2 + 0]);
		if (-1 == byte1)
			return 0;

		bytes[group] = ((byte1 << 4) & 0xf0);
		return group + 1;
	}
	else {
		return group;
	}
}

int main(int argc,char* argv[])
{
	/*int argc = 6;
	char * argv[] = { "exe", "-f","test.pdf","-c","-o","test.txt"};*/

	try {
		if (argc < 2)
			help();

		bool isUrl = false, isCode = true, isOutHex = false, isInHex = false;//是否16进制显示;
		string read_name = "", output_name = "", base64_str = "";
		unsigned char read_buf[BUF_SIZE + 1], write_buf[BUF_SIZE + 1];
		memset(read_buf, 0, BUF_SIZE + 1);
		memset(write_buf, 0, BUF_SIZE + 1);

		for (int i = 1; i < argc;) {
			if (!strcmp(argv[i],"-o")) {
				output_name = argv[++i];
				++i;
			}
			else if (!strcmp(argv[i],"-f")) {
				read_name = argv[++i];
				++i;
			}
			else if (!strcmp(argv[i], "-d")) {
				isCode = false;
				++i;
			}
			else if (!strcmp(argv[i], "-c")) {
				isCode = true;
				++i;
			}
			else if (!strcmp(argv[i], "-oh")) {
				isOutHex = true;
				++i;
			}
			else if (!strcmp(argv[i], "-ih")) {
				isInHex = true;
				++i;
			}
			else if (!strcmp(argv[i], "-u")) {
				isUrl = true;
				++i;
			}
			else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "-H") || !strcmp(argv[i], "-help")) {
				help();
				return 0;
			}
			else {
				base64_str = argv[i];
				++i;
			}
		}

		if (read_name.empty() && output_name.empty()) {
			string str = "";
			if (isCode) {
				if (isInHex) {
					int bytes_max_size = base64_str.length() + 1;
					unsigned char * bytes = new unsigned char[bytes_max_size + 1];
					memset(bytes, 0, bytes_max_size + 1);
					int input_len = hex_string_to_bytes(base64_str, bytes, bytes_max_size);
					str = Base64(bytes, input_len, isUrl);
				}
				else {
					str = Base64((unsigned char *)base64_str.data(), base64_str.length(), isUrl);
				}
			}
			else {
				int ret = 0;
				if (isInHex) {
					int bytes_max_size = base64_str.length() + 1;
					unsigned char * bytes = new unsigned char[bytes_max_size + 1];
					memset(bytes, 0, bytes_max_size + 1);
					int input_len = hex_string_to_bytes(base64_str, bytes, bytes_max_size);
					string str_t = bytes_2_string(bytes, input_len);
					ret = deBase64(str_t, write_buf, BUF_SIZE);
				}
				else {
					ret = deBase64(base64_str, write_buf, BUF_SIZE);
				}

				if (isOutHex)
					str = bytes_to_hex_string(write_buf, ret);
				else
					str = bytes_2_string(write_buf, ret);
			}
			cout << str;
		}
		else if (read_name.empty() && !output_name.empty()) {
			FILE * fp = fopen(output_name.c_str(), "wb");
			if (!fp)
				return -1;
			if (isCode) {
				string str = "";
				if (isInHex) {
					unsigned char * bytes = new unsigned char[base64_str.length() + 1];
					memset(bytes, 0, base64_str.length() + 1);
					int bytes_size = base64_str.length() + 1;
					int input_len = hex_string_to_bytes(base64_str, bytes, bytes_size);
					str = Base64(bytes, input_len, isUrl);
				}
				else {
					str = Base64((unsigned char *)base64_str.data(), base64_str.length(), isUrl);
				}
				write_bytes_2_file((unsigned char *)str.data(), str.length(), fp);
			}
			else {
				int de_ret = 0;
				if (isInHex) {
					int bytes_max_size = base64_str.length() + 1;
					unsigned char * bytes = new unsigned char[bytes_max_size + 1];
					memset(bytes, 0, bytes_max_size + 1);
					int input_len = hex_string_to_bytes(base64_str, bytes, bytes_max_size);
					string str_t = bytes_2_string(bytes, input_len);
					de_ret = deBase64(str_t, write_buf, BUF_SIZE);
				}
				else {
					de_ret = deBase64(base64_str, write_buf, BUF_SIZE);
				}
				write_bytes_2_file(write_buf, de_ret, fp);
			}
			fclose(fp);
		}
		else if (!read_name.empty() && output_name.empty()) {
			FILE * fp = fopen(read_name.c_str(), "rb");
			if (!fp)
				return -1;
			while (true) {
				int ret = read_bytes_from_file(read_buf, BUF_SIZE, fp);
				if (ret <= 0)
					break;

				string str = "";
				if (isCode) {
					str = Base64(read_buf, ret, isUrl);
				}
				else {
					string str_temp = bytes_2_string(read_buf, ret);
					int de_ret = deBase64(str_temp, write_buf, BUF_SIZE);

					if (isOutHex)
						str = bytes_to_hex_string(write_buf, de_ret);
					else
						str = bytes_2_string(write_buf, de_ret);
				}
				cout << str;
			}
			fclose(fp);
		}
		else if (!read_name.empty() && !output_name.empty()) {
			FILE * fp1 = fopen(read_name.c_str(), "rb");
			FILE * fp2 = fopen(output_name.c_str(), "wb");
			if (!fp1 || !fp2)
				return -1;
			while (true) {
				int read_ret = read_bytes_from_file(read_buf, BUF_SIZE, fp1);
				if (read_ret <= 0)
					break;

				if (isCode) {
					string str = Base64(read_buf, read_ret, isUrl);
					write_bytes_2_file((unsigned char *)str.data(), str.length(), fp2);
				}
				else {
					string str_ = bytes_2_string(read_buf, read_ret);
					int de_ret = deBase64(str_, write_buf, BUF_SIZE);
					write_bytes_2_file(write_buf, de_ret, fp2);
				}
			}
			fclose(fp1);
			fclose(fp2);
		}
	}
	catch (exception e) {
		cout << e.what();
	}
	return 0;
}