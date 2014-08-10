#ifndef __CITYGML_ASSIMPIMPORT_HPP__
#define __CITYGML_ASSIMPIMPORT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "importer.hpp"
#include "../citygml.hpp"

#include <assimp/ai_assert.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief CityGML import using assimp
///
class ImporterAssimp : public Importer
{
public:
    ImporterAssimp();
    virtual ~ImporterAssimp() override;

    /// Read filename with assimp and converts it to CityGML
    CityModel* import(const std::string& fileName);

private:
    CityModel* assimpSceneToCityGML(const struct aiScene* aiScene);
    void assimpNodeToCityGML(const struct aiScene* aiScene, const struct aiNode* aiNode, CityObject* parent);

    CityModel* m_model; ///< Result of import
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_ASSIMPIMPORT_HPP__
