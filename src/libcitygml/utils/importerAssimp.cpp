// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "importerAssimp.hpp"

#include <assimp/ai_assert.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
    ////////////////////////////////////////////////////////////////////////////////
    ImporterAssimp::ImporterAssimp()
        : m_model(nullptr)
    {

    }
    ////////////////////////////////////////////////////////////////////////////////
    ImporterAssimp::~ImporterAssimp()
    {
        delete m_model;
    }
    ////////////////////////////////////////////////////////////////////////////////
    /*void ImporterAssimp::assimpNodeToCityGML(const struct aiScene* aiScene, const struct aiNode* aiNode, CityObject* parent)
    {
    static int id = 0;
    CityObject* obj = new WallSurface("wall_"+std::to_string(id));

    // read geometry
    for(unsigned int i=0; i<aiNode->mNumMeshes; ++i)
    {
    const struct aiMesh* mesh = aiScene->mMeshes[ aiNode->mMeshes[i] ];

    Geometry* geom = new Geometry("geom_"+std::to_string(id)+'_'+std::to_string(i), GT_Wall, 2);

    for(unsigned int f=0; f<mesh->mNumFaces; ++f)
    {
    Polygon* poly = new Polygon("poly_"+std::to_string(id)+'_'+std::to_string(i)+'_'+std::to_string(f));
    LinearRing* ring = new LinearRing("ring_"+std::to_string(id)+'_'+std::to_string(i)+'_'+std::to_string(f), true);
    const struct aiFace& face = mesh->mFaces[f];
    for(unsigned ind=0; ind<face.mNumIndices; ++ind)
    {
    const aiVector3D& vTmp = mesh->mVertices[face.mIndices[ind]];
    TVec3d v(vTmp.x + m_offsetX, vTmp.y + m_offsetY, vTmp.z);
    ring->addVertex(v);
    }
    poly->addRing(ring);
    geom->addPolygon(poly);
    }

    // handle textures
    const aiVector3D* aiTexCoords = mesh->mTextureCoords[0];
    if(aiTexCoords)
    {
    Texture* tex = new Texture("tex_"+std::to_string(id)+'_'+std::to_string(i));
    const aiMaterial* aiMtl = aiScene->mMaterials[mesh->mMaterialIndex];
    aiString path;
    aiMtl->GetTexture(aiTextureType_DIFFUSE, mesh->mMaterialIndex, &path);
    tex->setUrl(path.C_Str());
    m_model->getAppearanceManager()->addAppearance(tex);

    for(unsigned int f=0; f<mesh->mNumFaces; ++f)
    {
    const struct aiFace& face = mesh->mFaces[f];

    m_model->getAppearanceManager()->assignNode("ring_"+std::to_string(id)+'_'+std::to_string(i)+'_'+std::to_string(f));

    TexCoords* vec = new TexCoords();

    for(unsigned ind=0; ind<face.mNumIndices; ++ind)
    {
    const aiVector3D& vTmp = aiTexCoords[face.mIndices[ind]];
    TVec2f t(vTmp.x, vTmp.y);
    vec->push_back(t);
    }
    m_model->getAppearanceManager()->assignTexCoords(vec);
    }

    m_model->getAppearanceManager()->assignNode("geom_"+std::to_string(id)+'_'+std::to_string(i));
    m_model->getAppearanceManager()->refresh();
    }

    obj->addGeometry(geom);
    }

    m_model->addCityObject(obj);
    parent->insertNode(obj);

    ++id;

    for(unsigned int i=0; i<aiNode->mNumChildren; ++i)
    {
    assimpNodeToCityGML(aiScene, aiNode->mChildren[i], parent);
    }
    }*/
    void ImporterAssimp::assimpNodeToCityGML(const struct aiScene* aiScene, const struct aiNode* aiNode, CityObject* parent)
    {
        static int id = 0;

        std::string Name = std::to_string(id) + "_Building"/* + std::string(aiNode->mName.C_Str())*/;

        citygml::CityObject* BuildingCO = new citygml::Building(Name);
        citygml::CityObject* RoofCO = new citygml::RoofSurface(Name + "_Roof");
        citygml::Geometry* RoofGeom = new citygml::Geometry(Name + "_RoofGeometry", citygml::GT_Roof, 2);
        citygml::CityObject* WallCO = new citygml::WallSurface(Name + "_Wall");
        citygml::Geometry* WallGeom = new citygml::Geometry(Name + "_WallGeometry", citygml::GT_Wall, 2);

        // read geometry
        for (unsigned int i = 0; i < aiNode->mNumMeshes; ++i)
        {
            const struct aiMesh* mesh = aiScene->mMeshes[aiNode->mMeshes[i]];

            for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
            {
                Polygon* poly = new Polygon("poly_" + std::to_string(id) + '_' + std::to_string(i) + '_' + std::to_string(f));
                LinearRing* ring = new LinearRing("ring_" + std::to_string(id) + '_' + std::to_string(i) + '_' + std::to_string(f), true);
                const struct aiFace& face = mesh->mFaces[f];
                for (unsigned ind = 0; ind < face.mNumIndices; ++ind)
                {
                    const aiVector3D& vTmp = mesh->mVertices[face.mIndices[ind]];
                    TVec3d v(vTmp.x + m_offsetX, vTmp.y + m_offsetY, vTmp.z);
                    ring->addVertex(v);
                }
                poly->addRing(ring);
                TVec3d Normale = ring->computeNormal();
                if ((std::abs(Normale.z) <= 0.2) || (!_detectRoof))
                    WallGeom->addPolygon(poly);
                else
                    RoofGeom->addPolygon(poly);
            }

            // handle textures
            /*const aiVector3D* aiTexCoords = mesh->mTextureCoords[0];
            if(aiTexCoords)
            {
                Texture* tex = new Texture("tex_"+std::to_string(id)+'_'+std::to_string(i));
                const aiMaterial* aiMtl = aiScene->mMaterials[mesh->mMaterialIndex];
                aiString path;
                aiMtl->GetTexture(aiTextureType_DIFFUSE, mesh->mMaterialIndex, &path);
                tex->setUrl(path.C_Str());
                m_model->getAppearanceManager()->addAppearance(tex);

                for(unsigned int f=0; f<mesh->mNumFaces; ++f)
                {
                    const struct aiFace& face = mesh->mFaces[f];

                    m_model->getAppearanceManager()->assignNode("ring_"+std::to_string(id)+'_'+std::to_string(i)+'_'+std::to_string(f));

                    TexCoords* vec = new TexCoords();

                    for(unsigned ind=0; ind<face.mNumIndices; ++ind)
                    {
                        const aiVector3D& vTmp = aiTexCoords[face.mIndices[ind]];
                        TVec2f t(vTmp.x, vTmp.y);
                        vec->push_back(t);
                    }
                    m_model->getAppearanceManager()->assignTexCoords(vec);
                }

                m_model->getAppearanceManager()->assignNode(Name+"_WallGeometry");
                m_model->getAppearanceManager()->refresh();
            }*/
        }

        if (RoofGeom->getPolygons().size() != 0 || WallGeom->getPolygons().size() != 0)
        {
            RoofCO->addGeometry(RoofGeom);
            m_model->addCityObject(RoofCO);
            BuildingCO->insertNode(RoofCO);
            WallCO->addGeometry(WallGeom);
            m_model->addCityObject(WallCO);
            BuildingCO->insertNode(WallCO);

            m_model->addCityObject(BuildingCO);
            m_model->addCityObjectAsRoot(BuildingCO);

            ++id;
        }

        for (unsigned int i = 0; i < aiNode->mNumChildren; ++i)
        {
            assimpNodeToCityGML(aiScene, aiNode->mChildren[i], parent);
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    CityModel* ImporterAssimp::assimpSceneToCityGML(const struct aiScene* aiScene)
    {
        delete m_model;
        m_model = new CityModel(aiScene->mRootNode->mName.C_Str()); // build root model
        CityObject* bldg = new Building(std::string("bldg_") + aiScene->mRootNode->mName.C_Str()); // build root building
        //m_model->addCityObject(bldg);
        //m_model->addCityObjectAsRoot(bldg);

        const aiNode* aiNode = aiScene->mRootNode;
        assimpNodeToCityGML(aiScene, aiNode, bldg);
        ParserParams params;
        m_model->finish(params); // call this to triangulate the polygons

        return m_model;
    }
    ////////////////////////////////////////////////////////////////////////////////
    CityModel* ImporterAssimp::import(const std::string& fileName, bool detectRoof) //detectRoof should be set at true to detect roofs and false if we only want wall polygons
    {
        Assimp::Importer importer;
        // read file with assimp
        const aiScene* aiScene = importer.ReadFile(fileName.c_str(), aiProcessPreset_TargetRealtime_Quality);

        _detectRoof = detectRoof;
        // convert it to CityGML
        CityModel* model = assimpSceneToCityGML(aiScene);
        model->computeEnvelope();

        return model;
    }
    ////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
