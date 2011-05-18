/**
 * This code is part of OgreCity.
 *
 * @file redbuilding.h
 * @date 10.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 *
 */

#ifndef _RED_BUILDING_H_
#define _RED_BUILDING_H_

#include <libcity.h>

#include "renderer.h"
#include "ogrebuilding.h"

class RedBuilding : public OgreBuilding
{
  public:
    static const Type RED_BUILDING;
    RedBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode);
    ~RedBuilding();

  protected:
    virtual void configure();
    void setupTextures();

    virtual void finishDrawing();

    virtual void interpretSymbol(char symbol);
  private:

    void initialize();
    void freeMemory();
};

#endif // _RED_BUILDING_H_
