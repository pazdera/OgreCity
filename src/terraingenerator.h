#ifndef TERRAINGENERATOR_H
#define TERRAINGENERATOR_H


#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>

class TerrainGenerator
{
  public:
    TerrainGenerator(Ogre::SceneManager* sceneManagerObject, Ogre::Light* lighting);
    ~TerrainGenerator();

    bool loadingInProgress();
    Ogre::String getStatus();
    void save();

    Ogre::Terrain* getTerrainObject();

    void draw();

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

#endif // TERRAINGENERATOR_H
