/**
 * This code is part of OgreCity.
 *
 * @file environmentrenderer.cpp
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see environmentrenderer.h
 *
 */

#include "environmentrenderer.h"

#include "terrainrenderer.h"

EnvironmentRenderer::EnvironmentRenderer(Ogre::SceneManager* manager)
  : Renderer(manager), terrain(0)
{}

EnvironmentRenderer::~EnvironmentRenderer()
{}

void EnvironmentRenderer::render()
{
  setupLighting();
  renderSky();
  //renderFog();
  renderTerrain();
}

void EnvironmentRenderer::setupLighting()
{
  //Ogre::Vector3 lightDirection(0.55, -0.3, 0.75);
  Ogre::Vector3 lightDirection(1, -1, 1);
  lightDirection.normalise();

  sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  //sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);
  //sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
  //sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  //sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
  sceneManager->setAmbientLight(Ogre::ColourValue(0.7, 0.7, 0.7, 0.2));

  Ogre::Light* light = sceneManager->createLight("sunLight");
  light->setType(Ogre::Light::LT_DIRECTIONAL);
  light->setDirection(lightDirection);
  light->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2, 0.2));
  //light->setSpecularColour(Ogre::ColourValue::White);

  sun = light;
}

void EnvironmentRenderer::renderSky()
{
  sceneManager->setSkyDome(true, "SkyCloudy", 5, 8, 500);
}

void EnvironmentRenderer::renderFog()
{
  Ogre::ColourValue fadeColour(0.9, 0.9, 0.9);
  sceneManager->setFog(Ogre::FOG_LINEAR, fadeColour, 0.0, 10, 1200);
}

void EnvironmentRenderer::renderTerrain()
{
  terrain = new TerrainRenderer(sceneManager);
  terrain->setLightingConditions(sun);
  terrain->render();
}


TerrainRenderer* EnvironmentRenderer::getTerrain()
{
  return terrain;
}
