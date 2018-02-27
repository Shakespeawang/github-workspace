
#include "Base64.h"

static const char base64_table[65] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'
};

static const char url_base64_table[65] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '-', '_'
};

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
		bytes[i] = (unsigned char) str[i];
	}
	return str.length();
}

static int get_base64_table_index(char ch)
{
	if (ch >= 'A' && ch <= 'Z')
		return ch - 'A';
	else if (ch >= 'a' && ch <= 'z')
		return ch - 'a' + 26;
	else if (ch >= '0' && ch <= '9')
		return ch - '0' + 52;
	else if ('+' == ch || '-' == ch)
		return 62;
	else if ('/' == ch || '_' == ch)
		return 63;
	else
		return -1;
}

static std::string Base64(const unsigned char bytes[], const int len, const bool isUrl)
{
	std::string base64_str = "";
	const char * table = (isUrl ? url_base64_table : base64_table);
	unsigned char group[3];
	const int group_cnt = len / 3;
	for (int i = 0; i < group_cnt; i++) {
		group[0] = bytes[i * 3];
		group[1] = bytes[i * 3 + 1];
		group[2] = bytes[i * 3 + 2];

		int idx = (group[0] >> 2) & 0x3f;
		base64_str += table[idx];

		idx = ((group[0] << 4) & 0x30) | ((group[1] >> 4) & 0x0f);
		base64_str += table[idx];

		idx = ((group[1] << 2) & 0x3c) | ((group[2] >> 6) & 0x03);
		base64_str += table[idx];

		idx = group[2] & 0x3f;
		base64_str += table[idx];
	}
	if (1 == len % 3) {
		group[0] = bytes[group_cnt * 3];
		int idx = (group[0] >> 2) & 0x3f;
		base64_str += table[idx];

		idx = ((group[0] << 4) & 0x30);
		base64_str += table[idx];

		base64_str += "==";
	}
	else if (2 == len % 3) {
		group[0] = bytes[group_cnt * 3];
		group[1] = bytes[group_cnt * 3 + 1];

		int idx = (group[0] >> 2) & 0x3f;
		base64_str += table[idx];

		idx = ((group[0] << 4) & 0x30) | ((group[1] >> 4) & 0x0f);
		base64_str += table[idx];

		idx = ((group[1] << 2) & 0x3c);
		base64_str += table[idx];

		base64_str += "=";
	}
	return base64_str;
}

static int deBase64(const std::string str_base64, unsigned char bytes[], const int bytes_size)
{
	if (str_base64.size() <= 0)
		return 0;

	std::string str = str_base64;

	if (1 == str.length() % 4)
		return 0;
	else if (2 == str.length() % 4)
		str += "==";
	else if (3 == str.length() % 4)
		str += "=";

	unsigned char group[4];
	const int group_cnt = str.length() / 4;

	if (bytes_size < group_cnt * 3)
		return 0;

	memset(bytes, 0, bytes_size);
	int last_group = 0;
	for (int i = 0; i < group_cnt; i++) {

		group[0] = get_base64_table_index(str[i * 4]);
		group[1] = get_base64_table_index(str[i * 4 + 1]);
		group[2] = get_base64_table_index(str[i * 4 + 2]);
		group[3] = get_base64_table_index(str[i * 4 + 3]);

		//如果到最后一组为'**=='的情况
		if ((i == group_cnt - 1) && ('=' == str[i * 4 + 2]) && ('=' == str[i * 4 + 3])) {
			//A,B,J,O
			if (-1 == group[0] || -1 == group[1])
				return 0;
			last_group = 2;
			bytes[i * 3 + 0] = ((group[0] << 2) & 0xfc) | ((group[1] >> 4) & 0x03);
		}
		else if ((i == group_cnt - 1) && ('=' == str[i * 4 + 3])) {
			//如果到最后一组为'***='的情况
			if (-1 == group[0] || -1 == group[1] || -1 == group[2])
				return 0;
			last_group = 1;
			bytes[i * 3 + 0] = ((group[0] << 2) & 0xfc) | ((group[1] >> 4) & 0x03);

			bytes[i * 3 + 1] = ((group[1] << 4) & 0xf0) | ((group[2] >> 2) & 0x0f);
		}
		else {
			if (-1 == group[0] || -1 == group[1] || -1 == group[2] || -1 == group[3])
				return 0;

			bytes[i * 3 + 0] = ((group[0] << 2) & 0xfc) | ((group[1] >> 4) & 0x03);

			bytes[i * 3 + 1] = ((group[1] << 4) & 0xf0) | ((group[2] >> 2) & 0x0f);

			bytes[i * 3 + 2] = ((group[2] << 6) & 0xc0) | ((group[3]) & 0x3f);
		}
	}
	return group_cnt * 3 - last_group;
}
