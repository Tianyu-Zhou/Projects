#if 1
#include<iostream>
using namespace std;

int max(int num1, int num2);

int main()
{
    int a,b,ret;
    cout<<"input two numbers:";
    cin>>a>>b;
    ret = max(a,b);
    cout<<"max value is:"<<ret<<endl;
    return 0;
}
int max(int num1, int num2)
{
    int result;

    if(num1 > num2)
        result = num1;
    else
        result = num2;
    return result;
}
#else

#endif
