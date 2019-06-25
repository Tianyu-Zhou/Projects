#include<iostream>
#include<stdio.h>
using namespace std;

int main()
{	
	int obs_x, obs_y, obs_i, obs_loc[100][3];
	for (obs_i = 1; obs_i < 100; obs_i++) 
	{
		cout << "input obstacles x and obstacles y : " << endl;
		cin >> obs_x >> obs_y;
//		obs_x = getchar(); obs_y = getchar();
//		if(obs_x == '\033' || obs_y == '\033')
		if(char (obs_x) == 27 || char (obs_y) == 27)
		{
			cout << char(obs_x) << endl;
			break;
		}
		else
		{
			obs_loc[obs_i][1] = obs_x;
			obs_loc[obs_i][2] = obs_y;
			cout << "obstacles X is : " << obs_loc[obs_i][1] << "  obstacles Y is " << obs_loc[obs_i
			][2] << endl;
		}
	}
	int rob_x, rob_y, dir, mov, n;
	rob_x = 0; rob_y = 0; dir = 0;
	cout << "input robot commond of move distance or direction" << endl;
	cout << "the input '-2' to turn left 90o, '-1 to turn right 90o'." << endl;
	cout << "commond should 1 <= x <= 9." <<endl;
	cin >> mov;
	for(n = 1; n < 100 ; n++)
	{
		if (mov == -2)
			dir = (dir+3)%4;
		else if (mov == -1)
			dir = (dir+1)%4;
		else if (-2 < mov || mov > 9)
			cout << "error" << endl;
		else
		{
			if(dir == 0)
				rob_x = rob_x + mov;
			if(dir == 1)
				rob_y = rob_y + mov;
			if(dir == 2)
				rob_x = rob_x - mov;
			if(dir == 3)
				rob_y = rob_y - mov;
		}		
	}
	cout << "robot location" << rob_x << " " << rob_y << endl;
	return 0;
}
