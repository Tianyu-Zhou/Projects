#if 0
#include<iostream>
using namespace std;

void swap(int x, int y)
{
    int temp;
    temp = x;
    x = y;
    y = temp;
    return;
}

int main()
{
    int a = 100;
    int b = 200;
    cout<<"jiao huan qian a:"<<a<<endl;
    cout<<"jiao huan qian b:"<<b<<endl;
    swap(a,b);
    cout<<"jiao huan hou a:"<<a<<endl;
    cout<<"jiao huan hou b:"<<b<<endl;
    return 0;
}

#else
    #if 0
    #include<iostream>
   using namespace std;
    void swap(int *x, int *y)
    {
    int temp;
    temp = *x;
    *x = *y;
    *y = temp;
    return;
    }

    int main()
    {
        int a = 100;
        int b = 200;
        cout<<"jiao huan qian a:"<<a<<endl;
        cout<<"jiao huan qian b:"<<b<<endl;
        swap(&a,&b);
        cout<<"jiao huan hou a:"<<a<<endl;
        cout<<"jiao huan hou b:"<<b<<endl;
        return 0;
    }

    #else
    #include<iostream>
    using namespace std;
    void swap(int &x, int &y)
    {
    int temp;
    temp = x;
    x = y;
    y = temp;
    return;
    }

    int main()
    {
        int a = 100;
        int b = 200;
        cout<<"jiao huan qian a:"<<a<<endl;
        cout<<"jiao huan qian b:"<<b<<endl;
        swap(a,b);
        cout<<"jiao huan hou a:"<<a<<endl;
        cout<<"jiao huan hou b:"<<b<<endl;
        return 0;
    }

   


    #endif
#endif
