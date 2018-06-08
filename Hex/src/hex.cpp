#include <iostream>
#include <string>

#define BUF_SIZE 512 * 3 * 4 //设为12的倍数
using namespace std;

/*
*	usage:	hex ABCD -o ouput.txt
*		-o:		output to text
*		-f:		read from file
*		-b:		from hex to bits
*		-h:		from bits to hex
*		--help:	help
*/

void help()
{
	cout << "usage:	hex ABCD -o ouput.txt" << endl;
	cout << "\t-o:\toutput to text" << endl;
	cout << "\t-f:\tread from file" << endl;
	cout << "\t-2b:\tfrom hex to bits" << endl;
	cout << "\t-2h:\tfrom bits to hex" << endl;
	cout << "\t--help:\thelp" << endl;
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
	else if (hex >= 'a' && hex <= 'f')
		return hex - 'a' + 10;
	else
		return -1;
}

string bytes_to_hex_string(const unsigned char bytes[], const int len)
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
		unsigned char byte1 = hex_char_2_decimal(hex_str[i * 2 + 0]);
		unsigned char byte2 = hex_char_2_decimal(hex_str[i * 2 + 1]);
		if (-1 == byte1 || -1 == byte2)
			return 0;

		bytes[i] = ((byte1 << 4) & 0xf0) | (byte2 & 0x0f);
	}
	if (1 == hex_str.length() % 2) {
		unsigned char byte1 = hex_char_2_decimal(hex_str[group * 2 + 0]);
		if (-1 == byte1)
			return 0;

		bytes[group] = ((byte1 << 4) & 0xf0);
		return group + 1;
	}
	else {
		return group;
	}
}

static std::string bytes_2_string(const unsigned char bytes[], const int len)
{
	std::string str = "";
	for (int i = 0; i < len; i++) {
		str += bytes[i];
	}
	return str;
}

static int string_2_bytes(const std::string str, unsigned char bytes[], const int len)
{
	if (len < str.length())
		return 0;

	for (int i = 0; i < str.length(); i++) {
		bytes[i] = (unsigned char)str[i];
	}
	return str.length();
}

int main(int argc, char* argv[])
{
	/*int argc = 6;
	char * argv[] = { "exe","-2b", "-f","hex.txt","-o","test2.txt"};*/

	try {
		if (argc < 2)
			help();

		bool isFromHex2Bits = true;//是否16进制显示;
		string read_name = "", output_name = "", base64_str = "";
		unsigned char read_buf[BUF_SIZE + 1], write_buf[BUF_SIZE + 1];
		::memset(read_buf, 0, BUF_SIZE + 1);
		::memset(write_buf, 0, BUF_SIZE + 1);

		for (int i = 1; i < argc;) {
			if (!strcmp(argv[i], "-o")) {
				output_name = argv[++i];
				++i;
			}
			else if (!strcmp(argv[i], "-f")) {
				read_name = argv[++i];
				++i;
			}
			else if (!strcmp(argv[i], "-2h")) {
				isFromHex2Bits = false;
				++i;
			}
			else if (!strcmp(argv[i], "-2b")) {
				isFromHex2Bits = true;
				++i;
			}
			else if (!strcmp(argv[i], "--help")) {
				help();
				return 0;
			}
			else {
				base64_str = argv[i];
				++i;
			}
		}

		if (read_name.empty() && output_name.empty()) {
			int ret_len = 0;
			if (isFromHex2Bits) {
				int bytes_max_size = base64_str.length() + 1;
				unsigned char * bytes = new unsigned char[bytes_max_size + 1];
				::memset(bytes, 0, bytes_max_size + 1);
				ret_len = hex_string_to_bytes(base64_str, bytes, bytes_max_size);
				cout << bytes_2_string(bytes, ret_len);
			}
			else {
				int bytes_max_size = base64_str.length() * 2 + 1;
				unsigned char * bytes = new unsigned char[bytes_max_size + 1];
				::memset(bytes, 0, bytes_max_size + 1);
				ret_len = string_2_bytes(base64_str, bytes, bytes_max_size);
				cout << bytes_to_hex_string(bytes, ret_len);
			}
		}
		else if (read_name.empty() && !output_name.empty()) {
			FILE * fp = fopen(output_name.c_str(), "wb");
			if (!fp)
				return -1;

			if (isFromHex2Bits) {
				int bytes_max_size = base64_str.length() + 1;
				unsigned char * bytes = new unsigned char[bytes_max_size + 1];
				::memset(bytes, 0, bytes_max_size + 1);
				int ret_len = hex_string_to_bytes(base64_str, bytes, bytes_max_size);
				string str = bytes_2_string(bytes, ret_len);
				write_bytes_2_file((unsigned char *)str.data(), str.length(), fp);
			}
			else {
				int bytes_max_size = base64_str.length() * 2 + 1;
				unsigned char * bytes = new unsigned char[bytes_max_size + 1];
				::memset(bytes, 0, bytes_max_size + 1);
				int ret_len = string_2_bytes(base64_str, bytes, bytes_max_size);
				string str = bytes_to_hex_string(bytes, ret_len);
				write_bytes_2_file((unsigned char *)str.data(), str.length(), fp);
			}
		}
		else if (!read_name.empty() && output_name.empty()) {
			FILE * fp = fopen(read_name.c_str(), "rb");
			if (!fp)
				return -1;
			while (true) {
				int read_ret = read_bytes_from_file(read_buf, BUF_SIZE, fp);
				if (read_ret <= 0)
					break;

				if (isFromHex2Bits) {
					string s_temp = bytes_2_string(read_buf, read_ret);
					int bytes_max_size = s_temp.length() + 1;
					unsigned char * bytes = new unsigned char[bytes_max_size + 1];
					::memset(bytes, 0, bytes_max_size + 1);
					int ret_len = hex_string_to_bytes(s_temp, bytes, bytes_max_size);
					cout << bytes_2_string(bytes, ret_len);
				}
				else {
					cout << bytes_to_hex_string(read_buf, read_ret);
				}
			}
			::fclose(fp);
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

				if (isFromHex2Bits) {
					string s_temp = bytes_2_string(read_buf, read_ret);
					int bytes_max_size = s_temp.length() + 1;
					unsigned char * bytes = new unsigned char[bytes_max_size + 1];
					::memset(bytes, 0, bytes_max_size + 1);
					int ret_len = hex_string_to_bytes(s_temp, bytes, bytes_max_size);
					write_bytes_2_file(bytes, ret_len, fp2);
				}
				else {
					string str = bytes_to_hex_string(read_buf,read_ret);
					write_bytes_2_file((unsigned char *)str.data(), str.length(), fp2);
				}
			}
			::fclose(fp1);
			::fclose(fp2);
		}
	}
	catch (exception e) {
		cout << e.what();
	}
	return 0;
}