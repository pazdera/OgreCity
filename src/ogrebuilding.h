/**
 * This code is part of OgreCity.
 *
 * @file ogrebuilding.h
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * LSystem based class that renders an sky scraper.
 *
 */

#ifndef _OGRE_BUILDING_H_
#define _OGRE_BUILDING_H_

#include <libcity.h>

#include "renderer.h"

class OgreBuilding : public Renderer, public Building
{
  public:
    OgreBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode);
    ~OgreBuilding();

    void render();

  protected:
    MaterialDefinition windowTileMaterial;
    MaterialDefinition basementMaterial;
    MaterialDefinition spacerMaterial;
    MaterialDefinition ledgeMaterial;
    MaterialDefinition rooftopMaterial;
    MaterialDefinition floorMaterial;

    double storeyHeight;
    double basementHeight;
    double spacerHeight;
    double ledgeHeight;
    double tileWidth;
    double rooftopHeight;

    virtual void configure();

    Ogre::ManualObject* buildingObject;
    virtual void interpretSymbol(char symbol);
  protected:

    Ogre::String getUniqueObjectName();
    Ogre::SceneNode* parentSceneNode;

    virtual void stopGrowth();

    virtual void drawBasement();
    virtual void drawFloor();
    virtual void drawStorey();
    virtual void drawSpacer();
    virtual void drawLedge();
    virtual void drawObliqueLedge();
    virtual void drawRooftop();
    virtual void drawHoweRooftop();
    virtual void substractBoundingBox(double byThisMuch);
    virtual void addBoundingBox(double byThisMuch);
    virtual void finishDrawing();

    void renderStorey(MaterialDefinition const& material, Point const& bottom, Point const& top);
    void renderObliqueShape(MaterialDefinition const& material, Polygon const& bottomBase, Polygon const& topBase,
                                                                Point const& beginingHeight, Point const& endHeight);
    void renderFloor(MaterialDefinition const& material);
    void addPlane(Polygon const& plane, Vector const& surfaceNormal, MaterialDefinition const& material);

    void initialize();
    void freeMemory();
};

#endif // _OGRE_BUILDING_H_
