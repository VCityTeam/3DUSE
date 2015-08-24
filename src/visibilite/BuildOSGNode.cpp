#include "BuildOSGNode.h"

#include "osg/LineWidth"

osg::ref_ptr<osg::Node> BuildSkylineOSGNode(std::vector<std::pair<TVec2d,TVec3d>> skyline, std::string prefix)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Geometry* geom = new osg::Geometry;
	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec3Array* colors = new osg::Vec3Array;
	osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

	for(unsigned int i = 0; i < skyline.size(); i++)
	{
		vertices->push_back(osg::Vec3(skyline[i].second.x,skyline[i].second.y,skyline[i].second.z));
		colors->push_back(osg::Vec3(1.0,0.0,1.0));
	}

	for(unsigned int i = 0; i < skyline.size()-1; i++)
	{
		indices->push_back(i); indices->push_back(i+1);
	}


	geom->setVertexArray(vertices);
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(indices);
	geode->addDrawable(geom);

	osg::LineWidth* linewidth = new osg::LineWidth(); 
	linewidth->setWidth(3.0f); 
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth); 

	osg::ref_ptr<osg::Node> node = geode;
	node->setName(prefix+"ViewShed");

	return node;
}

osg::ref_ptr<osg::Node> BuildViewshedOSGNode(AnalysisResult result, std::string prefix)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::Geometry* geom = new osg::Geometry;
	osg::Vec3Array* vertices = new osg::Vec3Array;
	osg::Vec3Array* colors = new osg::Vec3Array;
	osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

	ViewPoint* viewpoint = result.viewpoint;

	unsigned int cpt = 0;

	std::cout << "Beginning" << std::endl;

	for(unsigned int i = 0; i < viewpoint->width; i++)
	{
		for(unsigned int j = 0; j < viewpoint->height; j++)
		{
			Hit hit = viewpoint->hits[i][j];

			if(hit.intersect)
			{
				TVec3d point = hit.point;
				vertices->push_back(osg::Vec3(point.x,point.y,point.z));
				vertices->push_back(osg::Vec3(point.x,point.y,point.z+1.0));
				colors->push_back(osg::Vec3(0.0,1.0,0.0));
				colors->push_back(osg::Vec3(0.0,1.0,0.0));
				indices->push_back(cpt++);
				indices->push_back(cpt++);
			}
		}
	}
	std::cout << "Ending" << std::endl;

	geom->setVertexArray(vertices);
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->addPrimitiveSet(indices);
	geode->addDrawable(geom);

	osg::LineWidth* linewidth = new osg::LineWidth(); 
	linewidth->setWidth(6.0f); 
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth); 

	osg::ref_ptr<osg::Node> node = geode;
	node->setName(prefix+"Skyline");

	return node;
}