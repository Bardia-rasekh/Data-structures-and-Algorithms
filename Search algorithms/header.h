#include <bits/stdc++.h>

using namespace std;

class searchAlgorithms
{
private:
	vector<double> source;
	int sourceSize = 0;
	int inner_binarySearch(int target = -1, int low = -1, int high = -1);
public:
	void addNumber(int element = -1);
	int linearSearch(int target = -1);
	int binarySearch(int target = -1);
	int jumpSearch(int target = -1);
	int interpolationSearch(int target = -1);
	int exponentialSearch(int target = -1);
};

void searchAlgorithms::addNumber(int element)
{
	source.push_back(element);
	sourceSize++;
	sort(source.begin(), source.end());
}

int searchAlgorithms::linearSearch(int target)
{
	for (int i = 0; i < sourceSize; i++) if (source[i] == target) return i;
	return -1;
}

int searchAlgorithms::binarySearch(int target)
{
	int low = 0, high = sourceSize - 1, mid;

	while (low <= high)
	{
		mid = (high + low) / 2;
		if (source[mid] == target) return mid;
		if (source[mid] < target) low = mid + 1;
		else if (source[mid] > target) high = mid - 1;
	}
	return -1;
}

int searchAlgorithms::jumpSearch(int target)
{
	int step = sqrt(sourceSize);
	int last = step;
	
	int first = 0;
	while (source[min(last,sourceSize)-1] < target)
	{
		first = last;
		last += step;
		if (first >= sourceSize) return -1;
	}

	while (source[first] < target)
	{
		first++;
		if (first == min(last, sourceSize)) return -1;
	}

	if (source[first] == target) return first;

	return -1;
}

int searchAlgorithms::interpolationSearch(int target)
{
	int low = 0, high = (sourceSize - 1);

	while (low <= high && target >= source[low] && target <= source[high])
	{
		if (low == high)
		{
			if (source[low] == target) return low;
			return -1;
		}

		int pos = low + (((double)(static_cast<unsigned __int64>(high) - static_cast<unsigned __int64>(low)) / (source[high] - source[low])) * (target - source[low]));

		if (source[pos] == target) return pos;
		if (source[pos] < target) low = pos + 1;
		else high = pos - 1;
	}

	return -1;
}

int searchAlgorithms::inner_binarySearch(int target, int low, int high)
{
	int mid;
	while (low <= high)
	{
		mid = (high + low) / 2;
		if (source[mid] == target) return mid;
		if (source[mid] < target) low = mid + 1;
		else if (source[mid] > target) high = mid - 1;
	}
	return -1;
}

int searchAlgorithms::exponentialSearch(int target)
{
	if (source[0] == target) return 0;

	int i = 1;
	while (i < sourceSize && source[i] <= target) i *= 2;

	return inner_binarySearch(target, i / 2, min(i, sourceSize));
}
