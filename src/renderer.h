/**
 * This code is part of OgreCity.
 *
 * @file renderer.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * Base class for all renderers in this application.
 *
 */

#ifndef _RENDERER_H_
#define _RENDERER_H_


#include <OgreSceneManager.h>

class Renderer
{
  private:
    /**
      Rendering class cannot be constructed without saying,
      what scene manager will be used.
      */
    Renderer();

  public:
    Renderer(Ogre::SceneManager* manager);
    virtual ~Renderer();

    /**
      All rendering classes must implement this method, that is invoked
      when the scene is drawn
      */
    virtual void render() = 0;

  protected:
    Ogre::SceneManager* sceneManager;
};

#endif // _RENDERER_H_
