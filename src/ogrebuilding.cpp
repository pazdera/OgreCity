/**
 * This code is part of OgreCity.
 *
 * @file ogrebuilding.cpp
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see ogrebuilding.h
 *
 */

#include "ogrebuilding.h"

#include "ogrecity.h"

/* Ogre3D */
#include <OgreManualObject.h>

OgreBuilding::OgreBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode)
  : Renderer(manager), Building(parentAlottment), parentSceneNode(parentNode)
{
  initialize();
}

void OgreBuilding::initialize()
{
  buildingObject = new Ogre::ManualObject(getUniqueObjectName());
  /*
    B: draw basement
    F: draw floor
    S: draw spacer between floors
    L: draw ledge between floors (with substracting bounding box)
    O: draw oblique ledge between floors (with substracting bounding box)
    R: draw flat rooftop
    H: draw howe rooftop
    E: floor expansion
    -: substract bounding box
    +: add to bounding box
   */
  addToAlphabet("BFSOLRHE-+");

  configure();
}

void OgreBuilding::configure()
{
  setAxiom("{BFER}");

  setInitialDirection(Vector(0,0,1));

  floorMaterial   = "Concrete";
  rooftopMaterial = "RoofTop";

  storeyHeight = 20;
  basementHeight = 20;
  spacerHeight = 5;
  tileWidth = 40;
}

void OgreBuilding::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    case 'B':
      drawBasement();
      break;
    case 'F':
      drawStorey();
      break;
    case 'G':
      drawFloor();
      break;
    case 'S':
      drawSpacer();
      break;
    case 'O':
      drawObliqueLedge();
      break;
    case 'L':
      drawLedge();
      break;
    case 'R':
      drawRooftop();
      break;
    case 'H':
      drawHoweRooftop();
      break;
    case '-':
      substractBoundingBox(5);
      break;
    case '+':
      addBoundingBox(5);
      break;
    default:
      /* Try to interpret symbols defined in parent. */
      Building::interpretSymbol(symbol);
      break;
  }
}

OgreBuilding::~OgreBuilding()
{
  freeMemory();
}

void OgreBuilding::freeMemory()
{
  delete buildingObject;
}

void OgreBuilding::drawBasement()
{
  //debug("OgreBuilding::drawBasement()");
  Point lowerBound = cursor.getPosition();
  cursor.move(basementHeight);
  Point higherBound = cursor.getPosition();

  renderStorey(basementMaterial, lowerBound, higherBound);
}

void OgreBuilding::drawStorey()
{
  //debug("OgreBuilding::drawStorey()");
  Point lowerBound = cursor.getPosition();
  cursor.move(storeyHeight);
  Point higherBound = cursor.getPosition();

  renderStorey(windowTileMaterial, lowerBound, higherBound);
}

void OgreBuilding::drawFloor()
{
  renderFloor(floorMaterial);
}

void OgreBuilding::drawSpacer()
{
  //debug("OgreBuilding::drawSpacer()");
  Point lowerBound = cursor.getPosition();
  cursor.move(spacerHeight);
  Point higherBound = cursor.getPosition();

  renderStorey(spacerMaterial, lowerBound, higherBound);
}

void OgreBuilding::drawObliqueLedge()
{
  Polygon bottomBase = boundingBox->base();

  Point lowerBound = cursor.getPosition();
  cursor.move(ledgeHeight);
  Point higherBound = cursor.getPosition();

  addBoundingBox(2);
  Polygon topBase = boundingBox->base();

  renderObliqueShape(ledgeMaterial, bottomBase, topBase, lowerBound, higherBound);

  drawFloor();
  substractBoundingBox(2);
}

void OgreBuilding::drawHoweRooftop()
{
  Polygon bottomBase = boundingBox->base();

  Point lowerBound = cursor.getPosition();
  cursor.move(rooftopHeight);
  Point higherBound = cursor.getPosition();

  /* Substract bounding box to minimum */
  Polygon base = boundingBox->base();
  Polygon baseCopy;
  double constraintsArea = parentLot->areaConstraints().area();
  while (base.isNonSelfIntersecting() && boundingBox->encloses(base) &&
         base.area() > 0.1*parentLot->areaConstraints().area())
  {
    baseCopy = base;
    base.substract(1);
  }
  boundingBox->setBase(baseCopy);

  Polygon topBase = boundingBox->base();

  renderObliqueShape(rooftopMaterial, bottomBase, topBase, lowerBound, higherBound);

  drawRooftop();
}

void OgreBuilding::drawLedge()
{
  addBoundingBox(1);
  drawFloor();

  Point lowerBound = cursor.getPosition();
  cursor.move(ledgeHeight);
  Point higherBound = cursor.getPosition();

  renderStorey(ledgeMaterial, lowerBound, higherBound);

  drawFloor();
  substractBoundingBox(1);
}

void OgreBuilding::drawRooftop()
{
  renderFloor(rooftopMaterial);
}

void OgreBuilding::substractBoundingBox(double byThisMuch)
{
  //debug("OgreBuilding::substractBoundingBox()");

  if (boundingBox->base().area() < 0.5*parentLot->areaConstraints().area())
  {
    return;
  }

  Polygon base = boundingBox->base();
  base.substract(byThisMuch);
  if (base.isNonSelfIntersecting() && boundingBox->encloses(base))
  {
    boundingBox->setBase(base);
  }
}

void OgreBuilding::addBoundingBox(double byThisMuch)
{
  //debug("OgreBuilding::addBoundingBox()");

  Polygon base = boundingBox->base();
  base.substract(-byThisMuch);
  if (base.isNonSelfIntersecting())
  {
    boundingBox->setBase(base);
  }
}

void OgreBuilding::renderStorey(MaterialDefinition const& material, Point const& bottom, Point const& top)
{
  Polygon base = boundingBox->base();
  Polygon wall;
  Vector normal;
  Vector floorOffset = Vector(0, 0, bottom.z());
  Vector ceilingOffset = Vector(0, 0, top.z());
  int current, next;
  for (int i = 0; i < base.numberOfVertices(); i++)
  {
    current = i;
    next = (i+1) % base.numberOfVertices();

    normal = base.edgeNormal(current)*(-1);

    wall.clear();
    wall.addVertex(base.vertex(current) + floorOffset);
    wall.addVertex(base.vertex(next) + floorOffset);
    wall.addVertex(base.vertex(next) + ceilingOffset);
    wall.addVertex(base.vertex(current) + ceilingOffset);

    addPlane(wall, normal, material);
  }
}

void OgreBuilding::renderFloor(MaterialDefinition const& material)
{
  Polygon base = boundingBox->base();
  assert(base.numberOfVertices() > 0);
  Polygon roof;
  Point roofVertex;
  Point min, max;
  double currentHeight = cursor.getPosition().z();

  min = base.vertex(0);
  max = base.vertex(0);
  for (int i = 0; i < base.numberOfVertices(); i++)
  {
    /* update minimum and maximum values */
    if (min.x() > base.vertex(i).x()) min.setX(base.vertex(i).x());
    if (min.y() > base.vertex(i).y()) min.setY(base.vertex(i).y());
    if (max.x() < base.vertex(i).x()) max.setX(base.vertex(i).x());
    if (max.y() < base.vertex(i).y()) max.setY(base.vertex(i).y());

    roofVertex = base.vertex(i);
    roofVertex.setZ(currentHeight);
    roof.addVertex(roofVertex);
  }

  double polygonWidth  = max.x() - min.x();
  double polygonHeight = max.y() - min.y();

  buildingObject->begin(material);

  for (int i = 0; i < roof.numberOfVertices(); i++)
  {
    buildingObject->position(OgreCity::libcityToOgre(roof.vertex(i)));
    buildingObject->textureCoord((roof.vertex(i).x() - min.x()) / polygonWidth, (roof.vertex(i).y() - min.y()) / polygonWidth);
  }

  Vector planeNormal = roof.normal();
  std::vector<int> triagnles = roof.getSurfaceIndexes();
  assert(triagnles.size() % 3 == 0);
  int numberOfTriangles = triagnles.size() / 3;

  for (int i = 0; i < numberOfTriangles; i++)
  {
      buildingObject->triangle(triagnles[3*i+2], triagnles[3*i+1], triagnles[3*i+0]);
      buildingObject->triangle(triagnles[3*i+0], triagnles[3*i+1], triagnles[3*i+2]);
    if (planeNormal == Vector(0,0,1))
    {
    }
    else
    {
    }
  }

  buildingObject->end();
}

void OgreBuilding::renderObliqueShape(MaterialDefinition const& material, Polygon const& bottomBase, Polygon const& topBase,
                                                                          Point const& beginingHeight, Point const& endHeight)
{
  assert(bottomBase.numberOfVertices() == topBase.numberOfVertices());
  assert(bottomBase.isClosed());

  Vector floorOffset = Vector(0, 0, beginingHeight.z());
  Vector ceilingOffset = Vector(0, 0, endHeight.z());

  Vector normal;
  unsigned int current, next;
  for (unsigned int number = 0; number < bottomBase.numberOfVertices(); number++)
  {
    current = number;
    next = (number + 1) % bottomBase.numberOfVertices();

    normal = bottomBase.edgeNormal(current)*(-1);

    Polygon plane;
    plane.addVertex(bottomBase.vertex(current) + floorOffset);
    plane.addVertex(bottomBase.vertex(next) + floorOffset);
    plane.addVertex(topBase.vertex(next) + ceilingOffset);
    plane.addVertex(topBase.vertex(current) + ceilingOffset);

    addPlane(plane, normal, material);
  }
}

void OgreBuilding::stopGrowth()
{
  SymbolString::iterator symbol = currentlyInterpretedSymbol,
                         removed;

  while ((symbol) != producedString->end())
  {
    char symbolChar = (*symbol)->getSymbol();
    removed = symbol;
    symbol++;
    currentlyInterpretedSymbol = symbol;

    delete *removed;
    producedString->erase(removed);

    if (symbolChar == 'E')
    {
      break;
    }
  }
}

void OgreBuilding::finishDrawing()
{
  drawRooftop();
}

void OgreBuilding::render()
{
  while (readNextSymbol())
  {
    if (!boundingBox->encloses(cursor.getPosition()))
    {
      finishDrawing();
      break;
    }
  }

  Ogre::String name = buildingObject->getName();
  buildingObject->convertToMesh(name + "Mesh");

  Ogre::SceneNode* sceneNode = parentSceneNode->createChildSceneNode(name+"Node");
  Ogre::StaticGeometry* staticObject = sceneManager->createStaticGeometry(name+"Static");

  Ogre::Entity* entity;
  entity = sceneManager->createEntity(name + "Entity", name + "Mesh");
  entity->setCastShadows(true);
  staticObject->addEntity(entity, Ogre::Vector3(0,0,0));
  staticObject->setCastShadows(true);
  staticObject->build();
  //sceneNode->attachObject(entity);
}

void OgreBuilding::addPlane(Polygon const& plane, Vector const& surfaceNormal, MaterialDefinition const& material)
{
  assert(plane.numberOfVertices() == 4);

  /* Plane must be defined in the following manner:
     |<---------^
     |          |
     v[0]------>|
    Otherwise expect unexpected results. */
  int horizontalTiles = plane.edge(0).length() / tileWidth;
  horizontalTiles = horizontalTiles == 0 ? 1 : horizontalTiles;

  double tileBottomWidth  = plane.edge(0).length() / horizontalTiles,
         tileTopWidth     = plane.edge(2).length() / horizontalTiles; //plane.edge(0).length() / horizontalTiles;

  Vector planeNormal = plane.normal();
  Vector bottomEdgeDirection(plane.vertex(0), plane.vertex(1));
  bottomEdgeDirection.normalize();
  Vector topEdgeDirection(plane.vertex(3), plane.vertex(2));
  topEdgeDirection.normalize();

  Polygon tile;

  //while (plane.vertex(0).x() + bottomEdgeDirection*tileWidth*i < plane.vertex(1).x())
  for (int i = 0; i < (horizontalTiles); i++)
  {
    buildingObject->begin(material);
    tile.clear();
    tile.addVertex(plane.vertex(0) + bottomEdgeDirection*tileBottomWidth*i);
    tile.addVertex(plane.vertex(0) + bottomEdgeDirection*tileBottomWidth*(i+1));
    tile.addVertex(plane.vertex(3) + topEdgeDirection*tileTopWidth*(i+1));
    tile.addVertex(plane.vertex(3) + topEdgeDirection*tileTopWidth*(i));

    buildingObject->position(OgreCity::libcityToOgre(tile.vertex(0)));
    buildingObject->textureCoord(1,1);
    buildingObject->position(OgreCity::libcityToOgre(tile.vertex(1)));
    buildingObject->textureCoord(0,1);
    buildingObject->position(OgreCity::libcityToOgre(tile.vertex(2)));
    buildingObject->textureCoord(0,0);
    buildingObject->position(OgreCity::libcityToOgre(tile.vertex(3)));
    buildingObject->textureCoord(1,0);

    if (planeNormal == surfaceNormal)
    {
      buildingObject->quad(0, 1, 2, 3);
    }
    else
    {
      buildingObject->quad(0, 3, 2, 1);
    }

    buildingObject->end();
    //i++;
  }


  /* Last tile */

}

Ogre::String OgreBuilding::getUniqueObjectName()
{
  /* Generate unique alias for naming Ogre entities */
  static int roadNumber = 0;
  Ogre::StringStream convertor;
  convertor << roadNumber;
  roadNumber++;

  return "Building_" + convertor.str();
}
