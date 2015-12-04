#include "RayTracing.h"

#include "data/Hit.hpp"
#include "data/ViewPoint.h"

#include <thread>

/**
*	@brief Data used by a ray tracing thread
*/
struct RayTracingData
{
	TriangleList* triangles; ///< List of triangles of a 3D Model
	std::vector<Ray*>* rowToDo; ///< List of ray to use for ray tracing
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
			if(ray->Intersect(tri, &hit))//Check if the ray hit the triangle and
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

	unsigned int tCount = std::thread::hardware_concurrency() - 1;//Get how many thread we have
	unsigned int rayPerThread = rays.size() / tCount + tCount;

	//std::cout << rays.size() << " " << tCount << " " << rayPerThread << std::endl;
	std::cout << "Thread : " << tCount << std::endl;
	std::cout << "Ray count : " << rays.size() << std::endl;

	//List of rays and their frag coord
	std::vector<Ray*>* toDo = new std::vector<Ray*>[tCount];//List of rays for each threads

	int cpt = 0;
	int NumThread = 0;
	for(int i = 0; i < rays.size(); ++i)
	{
		toDo[NumThread].push_back(rays.at(i));
		++cpt;

		if(cpt == rayPerThread)
		{
			cpt = 0;
			++NumThread;
		}
	}

	/*for(unsigned int i = 0; i < tCount; i++)
	{
		toDo[i].insert(toDo[i].begin(),rays.begin()+i*rayPerThread,rays.begin() + std::min((i+1)*rayPerThread, (unsigned int)rays.size()-1));
	}*/

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