#include <iostream>
#include <string>
#include "curl/curl.h"
#pragma comment(lib,"libcurl.lib")
using namespace std;

class GetWeb
{
	private:
		static string url;
		static CURL *curl;
	public:
		GetWeb(void);
		~GetWeb(void);
		static size_t process_data(void *buffer, size_t size, size_t nmemb, void *user_p);
		static bool Init();
		static void Cleanup();
};

string GetWeb::url = "http://www.baidu.com";
CURL* GetWeb::curl = NULL;

GetWeb::GetWeb(void)
{
}

GetWeb::~GetWeb(void)
{
}

bool GetWeb::Init()
{
	CURLcode
}
