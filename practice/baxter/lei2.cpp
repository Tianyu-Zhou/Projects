
#include <iostream>
using namespace std;

#if 0
class Line
{
	public:
		double length;
		void setLength(double len);
		double getLength(void);
};

double Line::getLength(void)
{
	return length;
}

void Line::setLength(double len)
{
	length = len;
}

int main()
{
	Line line;
	
	line.setLength(6.0);
	cout << "Length of line : " << line.getLength() << endl;
	line.length = 10.0;
	cout << "Length of line : " << line.length << endl;
	return 0;
}
#endif

#if 0
class Box
{
	public:
		double length;
		void setWidth(double wid);
		double getWidth(void);
		
	private:
		double width;
};

double Box::getWidth(void)
{
	return width;
}

void Box::setWidth(double wid)
{
	width = wid;
}

int main()
{
	Box box;
	box.length = 10.0;
	cout << "Length of box : " << box.length << endl;
	
	//box.width = 5.0; // error
	box.setWidth(5.0);
	cout << "Width of box : " << box.getWidth() << endl;
	
	return 0;
}

#endif

#if 1
class Box
{
	protected:
		double width;
};

class SmallBox:Box
{
	public:
		void setSmallWidth(double wid);
		double getSmallWidth(void);
};

double SmallBox::getSmallWidth(void)
{
	return width;
}

void SmallBox::setSmallWidth(double wid)
{
	width = wid;
}

int main()
{
	SmallBox box;
	box.setSmallWidth(5.0);
	cout << "Width of box : " << box.getSmallWidth() << endl;
	return 0;
}

#endif
