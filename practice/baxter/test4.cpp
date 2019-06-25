#include <iostream>
#include <fstream>
//#include <Windows>
//#include <wininet>
#define MAXBLOCKSIZE 1024
#pragma comment (lib, "wininet.lib")
void download(const char*);
int main(int argc, char* argv[]){
download("http://zhidao.baidu.com");
return 0 ;
if(argc > 1){
download((const char*)argv[1]);
}else{
cout<<"Usage: auto-Update url";
}
return 0;
}
/**
* 执行 文件下载 操作
* @param Url: The target action url
*
*/
void download(const char *Url)
{
HINTERNET hSession = InternetOpen("RookIE/1.0", INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
if (hSession != NULL)
{
HINTERNET handle2 = InternetOpenUrl(hSession, Url, NULL, 0, INTERNET_FLAG_DONT_CACHE, 0);
if (handle2 != NULL)
{
cout<<Url<<endl;
byte Temp[MAXBLOCKSIZE];
ULONG Number = 1;
ofstream ofs("baidu.txt");
if(ofs)
{
while (Number > 0)
{
InternetReadFile(handle2, Temp, MAXBLOCKSIZE - 1, &Number);
ofs<<Temp;
}
ofs.close();
}
InternetCloseHandle(handle2);
handle2 = NULL;
}
InternetCloseHandle(hSession);
hSession = NULL;
}
