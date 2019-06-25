/*
struct type_name {
member_type1 member_name1;
member_type2 member_name2;
member_type3 member_name3;
.
.
} object_names;
*/

/*
struct Books
{
   char  title[50];
   char  author[50];
   char  subject[100];
   int   book_id;
} book;
*/

#include <iostream>
#include <cstring>
using namespace std;

// void printBook( struct Books book);
// void printBook( struct Books *book);

struct Books
{
	char title[50];
	char author[50];
	char subject[100];
	int book_id;
};

int main()
{
	Books Book1;
	Books Book2;
	
	strcpy( Book1.title,"C++");
	strcpy( Book1.author, "ZTY");
	strcpy( Book1.subject, "biancheng");
	Book1.book_id = 12345;
	
	strcpy( Book2.title,"C++");
	strcpy( Book2.author, "ZTY");
	strcpy( Book2.subject, "biancheng");
	Book2.book_id = 54321;
	
	cout << "first book title : " << Book1.title << endl;
	cout << "first book author : " << Book1.author << endl;
	cout << "first book subject : " << Book1.subject << endl;
	cout << "first book ID : " << Book1.book_id << endl;
	// printBook(Book1);
	// printBook(&Book1);
	
	cout << "second book title : " << Book2.title << endl;
	cout << "second book author : " << Book2.author << endl;
	cout << "second book subject : " << Book2.subject << endl;
	cout << "second book ID : " << Book2.book_id << endl;
	// printBook(Book2);
	// printBook(&Book2);
		
	return 0;
}

/* void printBook( struct Books book)
{
	cout << "book title : " << book.title << endl;
	cout << "book author : " << book.author << endl;
	cout << "book subject : " << book.subject << endl;
	cout << "book ID : " << book.book_id << endl;
}
*/

/* void printBook( struct Books *book)
{
	cout << "book title : " << book->title << endl;
	cout << "book author : " << book->author << endl;
	cout << "book subject : " << book->subject << endl;
	cout << "book ID : " << book->book_id << endl;
}
*/

/*
typedef struct Books
{
   char  title[50];
   char  author[50];
   char  subject[100];
   int   book_id;
}Books;

Books Book1, Book2;
*/


