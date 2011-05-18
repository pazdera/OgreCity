/**
 * This code is part of OgreCity.
 *
 * @file skyscraper.h
 * @date 2.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * LSystem based class that renders an sky scraper.
 *
 */

#ifndef _SKYSCRAPER_H_
#define _SKYSCRAPER_H_

#include <libcity.h>

#include "renderer.h"
#include "ogrebuilding.h"

class SkyScraper : public OgreBuilding
{
  public:
    static const Type SKY_SCRAPER;
    SkyScraper(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode);
    ~SkyScraper();

  protected:
    virtual void configure();
    void setupTextures();

    virtual void setback();

    virtual void interpretSymbol(char symbol);
  private:

    void initialize();
    void freeMemory();
};

#endif // _SKYSCRAPER_H_
