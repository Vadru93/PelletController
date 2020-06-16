#pragma once
class DistMap
{
public:
	float dist;
	float bags;

	DistMap operator=(const DistMap other)
	{
		if (this != &other)
		{
			this->dist = other.dist;
			this->bags = other.bags;
		}
		return *this;
	}

	void init();
	void distance();
	DistMap(float _dist, float _bags)
	{
		dist = _dist;
		bags = _bags;
	}
};