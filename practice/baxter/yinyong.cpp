#if 0
#include <iostream>
using namespace std;

double vals[] = {10.1, 12.6, 33.1, 24.1, 50.0};
double& setValues(int i)
{
	return vals[i];
}

int main()
{
	cout << "gai bian qian de zhi" << endl;
	for(int i = 0; i < 5; i++)
	{
		cout << "vals[" << i << "] =";
		cout << vals[i] << endl;
	}
	
	setValues(1) = 20.23;
	setValues(3) = 70.8;
	
	cout << "gai bian hou de zhi" << endl;
	for (int i = 0; i < 5; i++)
	{
		cout << "vals[" << i << "] = ";
		cout << vals[i] << endl;
	}
	return 0;
}
#endif

#if 1
#include <iostream>
using namespace std;

int main()
{
	int rats = 100;
	int &rodent = rats;
	
  cout << "rats = "<<rats<<", rosent = "<<rodent<<endl;
  cout << "rats address = "<<&rats<<endl;
  cout << "rosent address = "<<&rodent<<endl;

  cout <<"==================================="<<endl;
  int bunnies = 50;
  rodent = bunnies;

  cout << "rats = "<<rats<<", rosent = "<<rodent<<", bunnies = "<<bunnies<<endl;
  cout << "rats address = "<<&rats<<endl;
  cout << "rosent address = "<<&rodent<<endl;
  cout << "bunniess address = "<<&bunnies<<endl;

  return 0;
}

#endif
