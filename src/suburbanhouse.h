/**
 * This code is part of OgreCity.
 *
 * @file suburbanhouse.h
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 *
 */

#ifndef _SUBURBAN_HOUSE_H_
#define _SUBURBAN_HOUSE_H_

#include <libcity.h>

#include "renderer.h"
#include "ogrebuilding.h"

class SuburbanHouse : public OgreBuilding
{
  public:
    static const Type SUBURBAN_HOUSE;
    SuburbanHouse(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode);
    ~SuburbanHouse();

  protected:
    virtual void configure();
    void setupTextures();

    virtual void finishDrawing();

    virtual void interpretSymbol(char symbol);
  private:

    void initialize();
    void freeMemory();
};

#endif // _SUBURBAN_HOUSE_H_
