#if 1
#include <iostream>
using namespace std;

int main()
{
    enum Weekend{Zero,One,Two,Three,Four};
    int a,b,c,d,e;
    a=Zero;
    b=One;
    c=Two;
    d=Three;
    e=Four;
    cout<<a<<","<<b<<","<<c<<","<<d<<","<<e<<endl;
    return 0;
}
#else
#include <iostream>
using namespace std;
int main()
{
    enum Weekend{Zero,One,Two=555,Three,Four};
    int a,b,c,d,e;
    a=Zero;
    b=One;
    c=Two;
    d=Three;
    e=Four;
    std::cout<<a<<","<<b<<","<<c<<","<<d<<","<<e<<endl;
    return 0;
}
#endif
