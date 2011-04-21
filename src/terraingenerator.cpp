#include "terraingenerator.h"

TerrainGenerator::TerrainGenerator(Ogre::SceneManager* sceneManagerObject, Ogre::Light* lighting)
  :mTerrainGlobals(0), mTerrainGroup(0), terrainsImported(false), sceneManager(sceneManagerObject), light(lighting)
{
}

TerrainGenerator::~TerrainGenerator()
{
  if (mTerrainGroup != 0) OGRE_DELETE mTerrainGroup;
  if (mTerrainGlobals != 0) OGRE_DELETE mTerrainGlobals;
}

void TerrainGenerator::getTerrainImage(bool flipX, bool flipY, Ogre::Image& img)
{
  // FIXME
  img.load("terrain.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  if (flipX)
  {
    img.flipAroundY();
  }
  if (flipY)
  {
    img.flipAroundX();
  }
}

void TerrainGenerator::defineTerrain(long x, long y)
{
  Ogre::String filename = mTerrainGroup->generateFilename(x, y);
  if (Ogre::ResourceGroupManager::getSingleton().resourceExists(mTerrainGroup->getResourceGroup(), filename))
  {
    mTerrainGroup->defineTerrain(x, y);
  }
  else
  {
    Ogre::Image img;
    getTerrainImage(x % 2 != 0, y % 2 != 0, img);
    //mTerrainGroup->defineTerrain(x, y, &img); // Terrain from imagemap
    mTerrainGroup->defineTerrain(x, y, 0.0f);
    terrainsImported = true;
  }
}

void TerrainGenerator::initBlendMaps(Ogre::Terrain* terrain)
{
//     Ogre::TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
//     Ogre::TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);
//     Ogre::Real minHeight0 = 70;
//     Ogre::Real fadeDist0 = 40;
//     Ogre::Real minHeight1 = 70;
//     Ogre::Real fadeDist1 = 15;
//     float* pBlend1 = blendMap1->getBlendPointer();
//     for (Ogre::uint16 y = 0; y < terrain->getLayerBlendMapSize(); ++y)
//     {
//         for (Ogre::uint16 x = 0; x < terrain->getLayerBlendMapSize(); ++x)
//         {
//             Ogre::Real tx, ty;
//
//             blendMap0->convertImageToTerrainSpace(x, y, &tx, &ty);
//             Ogre::Real height = terrain->getHeightAtTerrainPosition(tx, ty);
//             Ogre::Real val = (height - minHeight0) / fadeDist0;
//             val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
//
//             val = (height - minHeight1) / fadeDist1;
//             val = Ogre::Math::Clamp(val, (Ogre::Real)0, (Ogre::Real)1);
//             *pBlend1++ = val;
//         }
//     }
//     blendMap0->dirty();
//     blendMap1->dirty();
//     blendMap0->update();
//     blendMap1->update();
}

void TerrainGenerator::configureTerrainDefaults(Ogre::Light* light, Ogre::ColourValue ambientLight)
{
  // Configure global
  mTerrainGlobals->setMaxPixelError(8);
  // testing composite map
  mTerrainGlobals->setCompositeMapDistance(3000);

  // Important to set these so that the terrain knows what to use for derived (non-realtime) data
  mTerrainGlobals->setLightMapDirection(light->getDerivedDirection());
  mTerrainGlobals->setCompositeMapAmbient(ambientLight);
  mTerrainGlobals->setCompositeMapDiffuse(light->getDiffuseColour());

  // Configure default import settings for if we use imported image
  Ogre::Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
  defaultimp.terrainSize = 513;
  defaultimp.worldSize = 12000.0f;
  defaultimp.inputScale = 600;
  defaultimp.minBatchSize = 33;
  defaultimp.maxBatchSize = 65;

  // textures
  defaultimp.layerList.resize(1);
  defaultimp.layerList[0].worldSize = 200;
  defaultimp.layerList[0].textureNames.push_back("grass_1024_diffuse.png");
  defaultimp.layerList[0].textureNames.push_back("grass_1024_normal.png");
//  defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_diffusespecular.dds");
//  defaultimp.layerList[0].textureNames.push_back("dirt_grayrocky_normalheight.dds");
//  defaultimp.layerList[1].worldSize = 30;
//  defaultimp.layerList[1].textureNames.push_back("grass_green-01_diffusespecular.dds");
//  defaultimp.layerList[1].textureNames.push_back("grass_green-01_normalheight.dds");
//  defaultimp.layerList[2].worldSize = 200;
//  defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_diffusespecular.dds");
//  defaultimp.layerList[2].textureNames.push_back("growth_weirdfungus-03_normalheight.dds");
}

void TerrainGenerator::draw()
{
  mTerrainGlobals = OGRE_NEW Ogre::TerrainGlobalOptions();
  mTerrainGroup = OGRE_NEW Ogre::TerrainGroup(sceneManager, Ogre::Terrain::ALIGN_X_Z, 513, 12000.0f);

  mTerrainGroup->setFilenameConvention(Ogre::String("city3DTerrain"), Ogre::String("dat"));
  mTerrainGroup->setOrigin(Ogre::Vector3::ZERO);

  configureTerrainDefaults(light, sceneManager->getAmbientLight());

  /* Just one terrain, no need for cycles */
  /*for (long x = 0; x <= 0; ++x)
      for (long y = 0; y <= 0; ++y)*/
        int x = 0, y = 0;
        defineTerrain(x, y);

  // sync load since we want everything in place when we start
  mTerrainGroup->loadAllTerrains(true);

  if (terrainsImported)
  {
      Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
      while(ti.hasMoreElements())
      {
          Ogre::Terrain* t = ti.getNext()->instance;
          initBlendMaps(t);
      }
  }

  mTerrainGroup->freeTemporaryResources();
}

bool TerrainGenerator::loadingInProgress()
{
  return mTerrainGroup->isDerivedDataUpdateInProgress();
}

Ogre::String TerrainGenerator::getStatus()
{
  if (loadingInProgress())
  {
    if (terrainsImported)
    {
      return "Building terrain, please wait...";
    }
    else
    {
      return "Updating textures, please wait...";
    }
  }
  else
  {
    return "Ready.";
  }
}

void TerrainGenerator::save()
{
  if (terrainsImported)
  {
    mTerrainGroup->saveAllTerrains(true);
    terrainsImported = false;
  }
}

Ogre::Terrain* TerrainGenerator::getTerrainObject()
{
  Ogre::TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
  while(ti.hasMoreElements())
  {
    return ti.getNext()->instance;
  }

  return 0;
}
