#include "RayTracing.h"

#include "data/Hit.hpp"
#include "data/ViewPoint.h"

#include <thread>

typedef std::pair<Ray,std::pair<unsigned int,unsigned int>> RayFragCoord;

inline RayFragCoord GetRayFragCoord(Ray ray,unsigned int i, unsigned int j)
{
	return std::make_pair(ray, std::make_pair(i,j));
}

/**
*	@brief Data used by a ray tracing thread
*/
struct RayTracingData
{
	TriangleList* triangles; 
	std::vector<Ray*>* rowToDo;
};

//Loop through all triangles and check if any rays intersect with triangles
void RayLoop(RayTracingData data)
{
	for(unsigned int k = 0; k < data.rowToDo->size(); k++)
	{
		Ray* ray = data.rowToDo->at(k);
		for(unsigned int l = 0; l < data.triangles->triangles.size(); l++)
		{
			Triangle* tri = data.triangles->triangles.at(l);

			Hit hit;
			if(ray->Intersect(tri,&hit))//Check if the ray hit the triangle and
			{
				if(!ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].intersect || ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)].distance > hit.distance)//Check if it is closer than the previous one
				{
					ray->collection->viewpoint->hits[int(ray->fragCoord.x)][int(ray->fragCoord.y)] = hit;
				}
			}
		}

	}
}

void RayTracing(TriangleList* triangles, std::vector<Ray*> rays)
{
	QTime time;
	time.start();

	unsigned int tCount = std::thread::hardware_concurrency()/2;//Get how many thread we have
	unsigned int rayPerThread = rays.size() / tCount;

	//List of rays and their frag coord
	std::vector<Ray*>* toDo = new std::vector<Ray*>[tCount];//List of rays for each threads

	for(unsigned int i = 0; i < tCount; i++)
	{
		toDo[i].insert(toDo[i].begin(),rays.begin()+i*rayPerThread,rays.begin()+(i+1)*rayPerThread);
	}

	std::cout << "Thread : " << tCount << std::endl;
	std::cout << "Ray count : " << rays.size() << std::endl;

	std::vector<std::thread*> threads;//Our thread list

	for(unsigned int i = 0; i < tCount; i++)
	{
		RayTracingData data;
		data.triangles = triangles;
		data.rowToDo = &toDo[i];

		std::thread* t = new std::thread(RayLoop,data);
		threads.push_back(t);
	}

	std::cout << "Thread Launched " << std::endl;

	for(unsigned int i = 0; i < tCount; i++)//Join all our thread and update global data min and max distance
	{
		(*threads[i]).join();
		delete threads[i];
	}

	std::cout << "The joining is completed" << std::endl;

	delete[] toDo;

	std::cout << "Time : " << time.elapsed()/1000 << " sec" << std::endl;
}