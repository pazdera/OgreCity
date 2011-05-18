/**
 * This code is part of OgreCity.
 *
 * @file environmentrenderer.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * This class is responsible for rendering all the world
 * environment where the city will be build. This includes
 * sky, terrain, lighting etc.
 *
 */

#ifndef _ENVIRONMENTRENDERER_H_
#define _ENVIRONMENTRENDERER_H_

#include "renderer.h"

class TerrainRenderer;

class EnvironmentRenderer : public Renderer
{
  public:
    EnvironmentRenderer(Ogre::SceneManager* manager);
    ~EnvironmentRenderer();

    void render();

    TerrainRenderer* getTerrain();
  private:
    void renderSky();
    void renderFog();
    void renderTerrain();
    void setupLighting();

    TerrainRenderer* terrain;
    Ogre::Light* sun;
};

#endif // _ENVIRONMENTRENDERER_H_
