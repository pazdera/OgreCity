/**
 * This code is part of OgreCity.
 *
 * @file terrainrenderer.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * This renderer is responsible for displaying terrain.
 * Terrain generation in this project is based on a
 * tutorial from OgreWiki (http://www.ogre3d.org/tikiwiki/).
 *
 */

#ifndef _TERRAINRENDERER_H_
#define _TERRAINRENDERER_H_

#include "renderer.h"

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

class TerrainRenderer : public Renderer
{
  public:
    TerrainRenderer(Ogre::SceneManager* sceneManagerObject);
    ~TerrainRenderer();

    void setLightingConditions(Ogre::Light* lighting);

    bool loadingInProgress();
    Ogre::String getStatus();
    void save();

    Ogre::Terrain* getTerrainObject();

    void render();

  private:
    void defineTerrain(long x, long y);
    void initBlendMaps(Ogre::Terrain* terrain);
    void configureTerrainDefaults(Ogre::Light* light, Ogre::ColourValue ambientLight);
    void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img);

    Ogre::TerrainGlobalOptions* mTerrainGlobals;
    Ogre::TerrainGroup* mTerrainGroup;
    bool terrainsImported;

    Ogre::Terrain* terrainObject;
    Ogre::SceneManager* sceneManager;
    Ogre::Light* light;
};

#endif // _TERRAINRENDERER_H_
