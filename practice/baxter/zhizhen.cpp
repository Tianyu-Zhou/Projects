#if 0
#include <iostream>
using namespace std;

int main()
{
	int ar[3] = {10, 100, 200};
	for (int i = 0; i < 3; i++)
	{
		*ar = i;
		cout << *ar << endl;
		cout << ar[i] << endl;
	}
	cout << ar << &ar[0] << endl;
	return 0;
}
#endif

#if 0
#include<iostream>
using namespace std;

const int MAX = 3;
int main()
{
	int var[MAX] = {10, 100, 200};
	cout << var << endl;
	cout << *var << "\n" << endl;
	for (int i = 0; i < MAX ; i++)
	{
		*var = i;
		cout << *var << endl;
		cout << i << endl;
		cout << &i << endl;
		cout << var << endl;
		*(var + 1) = 500;
		cout << *(var + 1) << endl;
	}
	return 0;
}
#endif

/*关于值传递，指针传递，引用传递这几个方面还会存在误区；

 * 值传递：

 * 形参是实参的拷贝，改变形参的值并不会影响外部实参的值。从被调用函数
 * 的角度来说，值传递是单向的（实参->形参），参数的值只能传入，不能传
 * 出。当函数内部需要修改参数，并且不希望这个改变影响调用者时，采用
 * 值传递。

 * 指针传递：

 * 形参为指向实参地址的指针，当对形参的指向操作时，就相当于对实参本
 * 身进行的操作

 * 引用传递：

 * 形参相当于是实参的"别名"，对形参的操作其实就是对实参的操作，在引用
 * 传递过程中，被调函数的形式参数虽然也作为局部变量在栈中开辟了内存空
 * 间，但是这时存放的是由主调函数放进来的实参变量的地址。被调函数对形
 * 参的任何操作都被处理成间接寻址，即通过栈中存放的地址访问主调函数中
 * 的实参变量。正因为如此，被调函数对形参做的任何操作都影响了主调函数
 * 中的实参变量。*/

#if 1
#include<iostream>
using namespace std;
//值传递 4  
void change1(int n){
    cout<<"值传递--函数操作地址"<<&n<<endl;         //显示的是拷贝的地址而不是源地址  6   n++;
 }
 
 //引用传递10 
 void change2(int & n){
     cout<<"引用传递--函数操作地址"<<&n<<endl; 
     n++;
 }
  //指针传递15 
  void change3(int *n){
      cout<<"指针传递--函数操作地址 "<<n<<endl; 
     *n=*n+1;
  } 
 int     main(){
     int n=10;
     cout<<"实参的地址"<<&n<<endl;
     change1(n);
     cout<<"after change1() n="<<n<<endl;
     change2(n);
     cout<<"after change2() n="<<n<<endl;
     change3(&n);
     cout<<"after change3() n="<<n<<endl;
     return 0;
}




#endif
