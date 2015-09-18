#include "ViewPoint.h"


std::pair<int, int> ViewPoint::GetCoord(Position p)
{
	switch (p)
	{
	case W:
		return std::make_pair(-1,0);
		break;
	case NW:
		return std::make_pair(-1,1);
		break;
	case N:
		return std::make_pair(0,1);
		break;
	case NE:
		return std::make_pair(1,1);
		break;
	case E:
		return std::make_pair(1,0);
		break;
	case SE:
		return std::make_pair(1,-1);
		break;
	case S:
		return std::make_pair(0,-1);
		break;
	case SW:
		return std::make_pair(-1,-1);
		break;
	default:
		std::cout << "Error : pos not in enum range !" << std::endl;
		throw std::exception();
		break;
	}
}

inline unsigned int ViewPoint::Clamp(unsigned int x,unsigned int a,unsigned int b)
{
	return x < a ? a : (x > b ? b : x);
}

void ViewPoint::ComputeSkyline()
{
	bool found = false;

	unsigned int x = 0;
	unsigned int y = 0;

	skyline.FragCoords.clear();

	for(unsigned int i = 0; i < width; i++)
	{

		for(int j = height - 1; j >= 0; j--)
		{
			if(hits[i][j].intersect)
			{
				found = true;
				x = i;
				y = j;
				skyline.FragCoords.push_back(std::make_pair(i,j));
				break;
			}
		}
		if(found)
			break;
	}

	if(!found)
		return;

	Position pos = N;
	std::pair<int, int> c = GetCoord(pos);

	unsigned int realX = Clamp(x+c.first,0,width-1);
	unsigned int realY = Clamp(y+c.second,0,height-1);

	while(x < width - 1)
	{
		while(realY >= height || (realY < height && !hits[realX][realY].intersect))
		{
			int posInt = int(pos);
			posInt++;
			posInt = posInt % 8;
			pos = Position(posInt);
			c = GetCoord(pos);
			realX = Clamp(x+c.first,0,width);
			realY = Clamp(y+c.second,0,height);
		}

		x = realX;
		y = realY;

		if(x == skyline.FragCoords.back().first && y == skyline.FragCoords.back().second)//Si on arrive pas a avancer
		{
			x = x + 1;
			int posInt = int(pos);
			posInt-=5;
			posInt = posInt % 8;
			pos = Position(posInt);
		}
		if(x == skyline.FragCoords.front().first && y == skyline.FragCoords.front().second)//Si on a une boucle de skyline
			break;

		skyline.FragCoords.push_back(std::make_pair(x,y));

		int posInt = int(pos);
		posInt += 5;
		posInt = posInt % 8;
		pos = Position(posInt);

		c = GetCoord(pos);

		realX = Clamp(x+c.first,0,width);
		realY = Clamp(y+c.second,0,height);
	}

	for(unsigned int j = 0; j < skyline.FragCoords.size(); j++)
	{
		Hit h = hits[skyline.FragCoords[j].first][skyline.FragCoords[j].second];
		skyline.Points.push_back(h.point);
	}

	skyline.Position = position;
}

void ViewPoint::Reset()
{
	minDistance = FLT_MAX;
	maxDistance = FLT_MIN;

	skyline.FragCoords.clear();

	for(unsigned int i = 0; i < width; i++)
	{
		for(unsigned int j = 0; j < height; j++)
		{
			hits[i][j] = Hit();
		}
	}
}

void ViewPoint::ComputeMinMaxDistance()
{
	minDistance = FLT_MAX;
	maxDistance = -FLT_MAX;
	for(unsigned int i = 0; i < width; i++)
	{
		for(unsigned int j = 0; j < height; j++)
		{
			if(hits[i][j].intersect)
			{
				minDistance = std::min(minDistance,hits[i][j].distance);
				maxDistance = std::max(maxDistance,hits[i][j].distance);
			}
		}
	}
}

void Skyline::ComputeData()
{
	RadiusMin = DBL_MAX;
	RadiusMax = -DBL_MAX;
	Average = 0.0;
	Radius.clear();

	for(unsigned int i = 0; i < Points.size(); i++)
	{
		TVec3d p = Points[i] - Position;
		double r = sqrt( (p.x*p.x) + (p.y*p.y) );
		Radius.push_back(r);
		RadiusMin = std::min(r,RadiusMin);
		RadiusMax = std::max(r,RadiusMax);
		Average += r;
	}
	Average /= Points.size();

	for(unsigned int i = 0; i < Points.size(); i++)
	{
		StandardDeviation += (Radius[i] - Average) * (Radius[i] - Average);
	}

	StandardDeviation /= Points.size();
	StandardDeviation = sqrt(StandardDeviation);
}