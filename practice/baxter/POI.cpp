#include <iostream>
#include <stdio.h>
#include <curl/curl.h>
#include <time.h>

using namespace std;
 
static size_t downloadCallback(void *buffer, size_t sz, size_t nmemb, void *writer)
{
	string* psResponse = (string*) writer;
	size_t size = sz * nmemb;
	psResponse->append((char*) buffer, size);
 
	return sz * nmemb;
}
 
int main()
{
	time_t nowtime;
	nowtime = time(NULL);
	cout << nowtime << endl;
	string strUrl = "http://api.map.baidu.com/place/v2/search?query=中学&region=昆明&page_size=20&page_num=0&output=json&ak=9s5GSYZsWbMaFU8Ps2V2VWvDlDlqGaaO ";
	string strTmpStr;
	CURL *curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, strUrl.c_str());
	curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, downloadCallback); 
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &strTmpStr);
	CURLcode res = curl_easy_perform(curl);
	curl_easy_cleanup(curl);
	string strRsp;
	if (res != CURLE_OK)
	{
		strRsp = "error";
	}
	else
	{
		strRsp = strTmpStr;
	}
	
	printf("strRsp is |%s|\n", strRsp.c_str());
	return 0;
}
