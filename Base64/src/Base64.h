#ifndef __BASE64__H
#define __BASE64__H


#include <string>

static std::string bytes_2_string(const unsigned char bytes[], const int len);

static int string_2_bytes(const std::string str, unsigned char bytes[], const int len);

static std::string Base64(const unsigned char bytes[], const int len, const bool isUrl = false);

static int deBase64(const std::string str_base64, unsigned char bytes[], const int bytes_size);


#endif // !__BASE64__H
