// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "layerLas.hpp"
#include "application.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
LayerLas::LayerLas(const std::string& name)
    : abstractLayer(name)
{

}
////////////////////////////////////////////////////////////////////////////////
LayerLas::~LayerLas()
{
}
////////////////////////////////////////////////////////////////////////////////
void LayerLas::addLASpoint(const LASpoint& laspoint)
{
	LayerLas_LASpoint p;

	p.X = laspoint.get_X()/100.;
	p.Y = laspoint.get_Y()/100.;
	p.Z = laspoint.get_Z()/100.;
	p.classification = laspoint.classification;

    m_LASpoints.push_back(p);
}
////////////////////////////////////////////////////////////////////////////////
const std::string LayerLas::getType() const
{
    return "LayerLas";
}
////////////////////////////////////////////////////////////////////////////////
URI LayerLas::getURI() const
{
    URI uri;
    uri.append(getName(), getType());
    uri.setType(getType());

    return uri;
}
////////////////////////////////////////////////////////////////////////////////
void LayerLas::dump()
{
	 log() << "    " << "TODO" << "\n";
}
////////////////////////////////////////////////////////////////////////////////
void LayerLas::exportJSON()
{
	std::string basePath("./"); // dossier de sortie
	std::string fileName("exportLAS"); // fichier de sortie
	std::string fullPath(basePath + fileName + ".json");

	std::ofstream outFile;
	outFile.open(fullPath);
	outFile << std::fixed; // effet ?

	log() << " -> exportJSON: " << fullPath << " - nb points: " << (int)(m_LASpoints.size()) << "\n";

	outFile << "{\n";

	unsigned int cpt = 1;
    for(std::vector<LayerLas_LASpoint>::iterator it=m_LASpoints.begin(); it<m_LASpoints.end(); ++it)
    {
        outFile << "    "; // indent
		outFile << "\"pts_" << cpt++ << "\":[" << (*it).X << "," << (*it).Y << "," << (*it).Z  << "," << (int)((*it).classification) << "]";

		if ((it+1)<m_LASpoints.end())
			outFile << ",\n";
		else
			outFile << "\n"; // last
    }

	outFile << "}\n";

	outFile.close();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
