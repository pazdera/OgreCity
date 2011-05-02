/**
 * This code is part of OgreCity.
 *
 * @file renderer.cpp
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see renderer.h
 *
 */

#include "renderer.h"

#include <libcity.h>

Renderer::Renderer()
  : sceneManager(0)
{}

Renderer::Renderer(Ogre::SceneManager* manager)
{
  //assert(manager != 0);
  sceneManager = manager;
}

Renderer::~Renderer()
{}


