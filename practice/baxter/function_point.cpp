#if 0 
#include<iostream>
#include<cstdlib>
#include<ctime>
using namespace std;

int * getRandom()
{
    static int r[10];
    srand((unsigned)time(NULL));
    for (int i=0; i<10; i++)
    {
        r[i] = rand();
        cout << r[i] << endl;
    }
    return r;
}

int main()
{
    int *p;
    p = getRandom();
    for (int i=0; i<10; i++)
    {
        cout << "*(p + " << i << ") : ";
        cout << *(p + i) << endl;
    }
    cout << p << endl;
    return 0;
}
#else
#include<iostream>
using namespace std;

void MultMatrix(float M[4], float A[4], float B [4])
{
    M[0] = A[0]*B[0] + A[1]*B[2];
    M[1] = A[0]*B[1] + A[1]*B[3];
    M[2] = A[2]*B[0] + A[3]*B[2];
    M[3] = A[2]*B[1] + A[3]*B[3];

    cout << M[0] << " " << M[1] << endl;
    cout << M[2] << " " << M[3] << endl;    
}

int main()
{
    float A[4] = {1.75, 0.66, 0, 1.75};
    float B[4] = {1, 1, 0, 0};
    float *M = new float[4];
    MultMatrix(M, A, B);

    cout << M[0] << " " << M[1] << endl;
    cout << M[2] << " " << M[4] << endl;
    delete[] M;
    return 0;
}
#endif

//zhu shi
//静态 int array[100]; 　　定义了数组 array，并未对数组进行初始化
//静态 int array[100] = {1，2}；　　定义并初始化了数组 array
//动态 int* array = new int[100]; delete []array;　　分配了长度为 100 的数组 array
//动态 int* array = new int[100]{1，2}; 　delete []array;　为长度为100的数组array初始化前两个元素
