//1   trcpy(s1, s2);
//复制字符串 s2 到字符串 s1。
//2   strcat(s1, s2);
//连接字符串 s2 到字符串 s1 的末尾。
//3   strlen(s1);
//返回字符串 s1 的长度。
//4   strcmp(s1, s2);
//如果 s1 和 s2 是相同的，则返回 0；如果 s1<s2 则返回值小于 0；如果 s1>s2 则返回值大于 0。
//5   strchr(s1, ch);
//返回一个指针，指向字符串 s1 中字符 ch 的第一次出现的位置。
//6   strstr(s1, s2);
//返回一个指针，指向字符串 s1 中字符串 s2 的第一次出现的位置。
#if 0
#include<iostream>
#include<cstring>
using namespace std;

int main()
{
    char str1[11] = "Hello";
    char str2[11] = "World";
    char str3[11];
    int len;

    strcpy(str3, str1);
    cout << "strcpy(str3, str1) : " << str3 << endl;

    strcat(str1, str2);
    cout << "strcat(str1, str2) : " << str1 << endl;

    len = strlen(str1);
    cout << "strlen(str1) : " << len << endl;
    return 0;
}
#endif

//string类提供了一系列针对字符串的操作，比如：
//1. append() -- 在字符串的末尾添加字符
//2. find() -- 在字符串中查找字符串
//4. insert() -- 插入字符
//5. length() -- 返回字符串的长度
//6. replace() -- 替换字符串
//7. substr() -- 返回某个子字符串
//8. ...

#if 0
#include<iostream>
#include<string>
using namespace std;

int main()
{
    string str1 = "Hello";
    string str2 = "World";
    string str3;
    int len;

    str3 = str1;
    cout << "str3 : " << str3 << endl;

    str3 = str1 + str2;
    cout << "str1 + str2 : " << str3 <<endl;

    len = str3.size();
    cout << "str3.size() : " << len << endl;
    return 0;
}
#endif

#if 1
#include <iostream>
#include <string>
using namespace std;

int main()
{
    string http = "www.runoob.com";
    cout << http.length() << endl;
    http.append("/C++");
    cout << http << endl;
    int pos = http.find("C++");
    cout << pos << endl;
    http.replace(pos,4,"");
    cout << http << endl;

    int first = http.find_first_of(".");
    int last = http.find_last_of(".");
    cout << http.substr(first+1, last-first-1) << endl;
    return 0;
}
#endif
