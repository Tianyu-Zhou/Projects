#include<iostream>
using namespace std;

double getAverage(int arr[], int size);

int main()
{
    int balance[5] = {1000, 2, 3, 17 ,50};
    int *pt1 = balance;
    size_t balance_size = sizeof(balance)/sizeof(int);
    size_t pt1_size = sizeof(pt1);
    double avg;

    cout<<"array size : "<<balance_size<<endl;
    cout<<"int pointer size : "<<pt1_size<<endl;
    avg = getAverage(balance, 5);
    cout<<"ping jun zhi:"<<avg<<endl;
    return 0;
}

double getAverage(int arr[], int size)
{
    int i,sum = 0;
    double avg;
    cout<<"Inside getAverage sizeof(arr) = "<<sizeof(arr)<<endl;
    for (i = 0; i < size; i++)
    {
        sum += arr[i];
    }
    avg = double(sum)/size;
    return avg;
}
