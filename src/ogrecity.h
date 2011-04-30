/**
 * This code is part of OgreCity.
 *
 * @file ogrecity.h
 * @date 11.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @brief Implement abstract class City from libcity and build some city.
 *
 */

#ifndef _OGRECITY_H_
#define _OGRECITY_H_

#include <libcity.h>

#include <Ogre.h>
#include <Terrain/OgreTerrain.h>

#include "renderer.h"
#include "streetgraphrenderer.h"

class TerrainRenderer;

class OgreCity : public City, public Renderer
{
  public:
    OgreCity(Ogre::SceneManager* sceneManagerObject);
    virtual ~OgreCity();

    void setTerrain(TerrainRenderer* renderer);

    virtual void render();

    static Ogre::Vector3 libcityToOgre(Point const& point);
    static Ogre::Vector3 libcityToOgre(Vector const& vector);
    static Vector ogreToLibcity(Ogre::Vector3 const& vector);

  protected:
    virtual void createPrimaryRoadNetwork();
    virtual void createZones();
    virtual void createSecondaryRoadNetwork();
    virtual void createBlocks();
    virtual void createBuildings();

    virtual void renderRoadNetwork();
    virtual void renderBuildings();

    /**
      Inline method that is called in the constructor. This
      is the only place for any parameter initialization and
      configuration.
      */
    virtual void configuration()
    {
      /* Primary roads configuration */
      primaryRoad.height = 1;
      primaryRoad.width = 30;

      primaryRoad.material = "DrivingSim/RoadMat";


      // Texture mapping on vertices: 0, 0.03125, 0.5, 1-0.03125, 1
      primaryRoad.verticesTextureMapping.push_back(0);
      primaryRoad.verticesTextureMapping.push_back(0.03125);
      primaryRoad.verticesTextureMapping.push_back(0.5);
      primaryRoad.verticesTextureMapping.push_back(1-0.03125);
      primaryRoad.verticesTextureMapping.push_back(1);

      // Space offset of vertices: 1.1, 1, 0, -1, -1.1
      primaryRoad.verticesPositionOffset.push_back(1.0);
      primaryRoad.verticesPositionOffset.push_back(0.9);
      primaryRoad.verticesPositionOffset.push_back(0);
      primaryRoad.verticesPositionOffset.push_back(-0.9);
      primaryRoad.verticesPositionOffset.push_back(-1.0);

      /* Secondary roads configuration */
      secondaryRoad.height = 1;
      secondaryRoad.width = 15;

      secondaryRoad.material = "DrivingSim/RoadMat";

      secondaryRoad.verticesTextureMapping = primaryRoad.verticesTextureMapping; // Same as primary road
      secondaryRoad.verticesPositionOffset = primaryRoad.verticesPositionOffset;
      

      /* Building configuration */
    }

  private:
    Ogre::SceneManager* sceneManager;
    Ogre::Terrain* terrain;

    StreetGraphRenderer::RoadParameters primaryRoad, secondaryRoad;
};

#endif // _OGRECITY_H_
