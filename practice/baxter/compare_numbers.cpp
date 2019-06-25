#if 0 
#include<iostream>
using namespace std;

int main()
{
    int a,b;
    cout<<"input two numbers:";
    cin>>a>>b;
    a>b?cout<<a<<">"<<b<<endl:cout<<b<<">"<<a<<endl;
    return 0;
}
#else
#include<iostream>
using namespace std;

int main()
{
    int a,b,c,d,max;
    cout<<"input three numbers:";
    cin>>a>>b>>c;
    max = (d = a >= b ? a : b) >= c ? d : c;
    cout<<"the max is:"<<max<<endl;
    return 0;
}
#endif
