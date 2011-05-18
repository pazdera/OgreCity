/**
 * This code is part of OgreCity.
 *
 * @file olderbuilding.h
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * LSystem based class that renders an older building scraper.
 *
 */

#ifndef _OLDER_BUILDING_H_
#define _OLDER_BUILDING_H_

#include <libcity.h>

#include "renderer.h"
#include "ogrebuilding.h"

class OlderBuilding : public OgreBuilding
{
  public:
    static const Type OLDER_BUILDING;
    OlderBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode);
    ~OlderBuilding();

  protected:
    virtual void configure();
    void setupTextures();

    virtual void finishDrawing();

    virtual void interpretSymbol(char symbol);
  private:

    void initialize();
    void freeMemory();
};

#endif // _OLDER_BUILDING_H_
