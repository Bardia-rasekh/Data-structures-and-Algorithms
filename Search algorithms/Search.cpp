#include "Header.h"


int main()
{
	searchAlgorithms obj;

	obj.addNumber(1);
	obj.addNumber(3);
	obj.addNumber(7);
	obj.addNumber(10);
	obj.addNumber(14);
	obj.addNumber(15);
	obj.addNumber(16);
	obj.addNumber(20);
	obj.addNumber(21);
	obj.addNumber(18);
	obj.addNumber(22);
	obj.addNumber(23);
	obj.addNumber(25);
	obj.addNumber(33);
	obj.addNumber(35);
	obj.addNumber(42);
	obj.addNumber(45);
	obj.addNumber(47);
	obj.addNumber(50);
	obj.addNumber(52);

	cout << obj.linearSearch(18) << '\n';

	cout << obj.binarySearch(18) << '\n';

	cout << obj.jumpSearch(18) << '\n';

	cout << obj.interpolationSearch(18) << '\n';

	cout << obj.exponentialSearch(18) << '\n';



	return 0;
}

