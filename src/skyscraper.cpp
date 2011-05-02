/**
 * This code is part of OgreCity.
 *
 * @file skyscraper.cpp
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see streetgraphrenderer.h
 *
 */

#include "skyscraper.h"

#include "ogrecity.h"

/* Ogre3D */
#include <OgreManualObject.h>

const Building::Type SKY_SCRAPER = Building::defineNewEntityType();

SkyScraper::SkyScraper(Lot* parentAlottment, Ogre::SceneManager* manager)
  : Renderer(manager), Building(parentAlottment)
{
  initialize();
}

void SkyScraper::initialize()
{
  /*
    B: draw basement
    F: draw floor
    S: draw spacer between floors
    R: draw rooftop
    E: floor expansion
    -: substract bounding box
   */
  addToAlphabet("BFSRE-");

  setAxiom("{BFE}");

  addRule('E', "FE");   // Next normal floor
  addRule('E', "R-FE"); // Substraction

  setInitialDirection(Vector(0,0,1));

  windowTileMaterial = "";
  basementMaterial   = "";
  spacerMaterial     = "";
  rooftopMaterial    = "";

  buildingObject = new Ogre::ManualObject(getUniqueObjectName());
}

void SkyScraper::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    case 'B':
      drawBasement();
      break;
    case 'F':
      drawFloor();
      break;
    case 'S':
      drawSpacer();
      break;
    case 'R':
      drawRooftop();
      break;
    case '-':
      substractBoundingBox();
      break;
    default:
      /* Try to interpret symbols defined in parent. */
      Building::interpretSymbol(symbol);
      break;
  }
}

SkyScraper::~SkyScraper()
{
  freeMemory();
}

void SkyScraper::freeMemory()
{
  delete buildingObject;
}

void SkyScraper::drawBasement()
{
  Point lowerBound = cursor.getPosition();
  cursor.move(basementHeight);
  Point higherBound = cursor.getPosition();

  // for each bounding boxes base edge
  //   create wall polygon (4 vertices)
  //   decide what normal will the surface be on polygon edgeNormal*(-1)
  //   draw polygon with tiled texture

  Polygon base = boundingBox->base();
  Polygon wall;
  Vector normal;
  Vector ceilingOffset = higherBound - lowerBound;
  int current, next;
  for (int i = 0; i < base.numberOfVertices(); i++)
  {
    current = i;
    next = (i+1) % base.numberOfVertices();

    normal = base.edgeNormal(current)*(-1);

    wall.clear();
    wall.addVertex(base.vertex(current));
    wall.addVertex(base.vertex(next) + ceilingOffset);
    wall.addVertex(base.vertex(next) + ceilingOffset);
    wall.addVertex(base.vertex(current) + ceilingOffset);

    addPlane(wall, normal, basementMaterial);
  }
}

void SkyScraper::drawFloor()
{

}

void SkyScraper::drawSpacer()
{

}

void SkyScraper::drawRooftop()
{

}

void SkyScraper::substractBoundingBox()
{

}

void SkyScraper::render()
{

}

void SkyScraper::addPlane(Polygon const& plane, Vector const& surfaceNormal, Ogre::String const& material)
{
  assert(plane.numberOfVertices() == 4);

  buildingObject->begin(basementMaterial);

  for (int i = 0; i < plane.numberOfVertices(); i++)
  {
    buildingObject->position(OgreCity::libcityToOgre(plane.vertex(i)));
  }

  buildingObject->quad(0, 1, 2, 3);

  buildingObject->end();
}

Ogre::String SkyScraper::getUniqueObjectName()
{
  /* Generate unique alias for naming Ogre entities */
  static int roadNumber = 0;
  Ogre::StringStream convertor;
  convertor << roadNumber;
  roadNumber++;

  return "Building_" + convertor.str();
}
