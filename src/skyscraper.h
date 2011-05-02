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

class SkyScraper : public Renderer, public Building
{
  public:
    static const Type SKY_SCRAPER;
    SkyScraper(Lot* parentAlottment, Ogre::SceneManager* manager);
    ~SkyScraper();

    void render();

  protected:
    Ogre::String windowTileMaterial;
    Ogre::String basementMaterial;
    Ogre::String spacerMaterial;
    Ogre::String rooftopMaterial;

    double storeyHeight;
    double basementHeight;
    double spacerHeight;

    Ogre::ManualObject* buildingObject;
  private:
    virtual void interpretSymbol(char symbol);

    Ogre::String getUniqueObjectName();

    void drawBasement();
    void drawFloor();
    void drawSpacer();
    void drawRooftop();
    void substractBoundingBox();

    void addPlane(Polygon const& plane, Vector const& surfaceNormal, Ogre::String const& material);

    void initialize();
    void freeMemory();
};

#endif // _SKYSCRAPER_H_
