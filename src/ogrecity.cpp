/**
 * This code is part of OgreCity.
 *
 * @file ogrecity.cpp
 * @date 11.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see ogrecity.h
 *
 */

#include "ogrecity.h"
#include "terrainrenderer.h"
#include "streetgraphrenderer.h"
#include "skyscraper.h"
#include "olderbuilding.h"

OgreCity::OgreCity(Ogre::SceneManager* sceneManagerObject)
  : sceneManager(sceneManagerObject), Renderer(sceneManager)
{
  configuration();
}

OgreCity::~OgreCity()
{
}

void OgreCity::createPrimaryRoadNetwork()
{
  /* Define city area constraints */
  area->addVertex(Point(2000,2000));
  area->addVertex(Point(2000,-2000));
  area->addVertex(Point(-2000,-2000));
  area->addVertex(Point(-2000,2000));

  map->addRoad(Path(LineSegment(Point(2000,2000), Point(2000,-2000))));
  map->addRoad(Path(LineSegment(Point(2000,-2000), Point(-2000,-2000))));
  map->addRoad(Path(LineSegment(Point(-2000,-2000), Point(-2000,2000))));
  map->addRoad(Path(LineSegment(Point(-2000,2000), Point(2000,2000))));

//  area->addVertex(Point(1600,1600));
//  area->addVertex(Point(1600,-1600));
//  area->addVertex(Point(-1600,-1600));
//  area->addVertex(Point(-1600,1600));
//
//  map->addRoad(Path(LineSegment(Point(1600,1600), Point(1600,-1600))));
//  map->addRoad(Path(LineSegment(Point(1600,-1600), Point(-1600,-1600))));
//  map->addRoad(Path(LineSegment(Point(-1600,-1600), Point(-1600,1600))));
//  map->addRoad(Path(LineSegment(Point(-1600,1600), Point(1600,1600))));

//  area->addVertex(Point(1200,1200));
//  area->addVertex(Point(1200,-1200));
//  area->addVertex(Point(-1200,-1200));
//  area->addVertex(Point(-1200,1200));
//
//  map->addRoad(Path(LineSegment(Point(1200,1200), Point(1200,-1200))));
//  map->addRoad(Path(LineSegment(Point(1200,-1200), Point(-1200,-1200))));
//  map->addRoad(Path(LineSegment(Point(-1200,-1200), Point(-1200,1200))));
//  map->addRoad(Path(LineSegment(Point(-1200,1200), Point(1200,1200))));

//  area->addVertex(Point(800,800));
//  area->addVertex(Point(800,-800));
//  area->addVertex(Point(-800,-800));
//  area->addVertex(Point(-800,800));
//
//  map->addRoad(Path(LineSegment(Point(800,800), Point(800,-800))));
//  map->addRoad(Path(LineSegment(Point(800,-800), Point(-800,-800))));
//  map->addRoad(Path(LineSegment(Point(-800,-800), Point(-800,800))));
//  map->addRoad(Path(LineSegment(Point(-800,800), Point(800,800))));

//    area->addVertex(Point(600,600));
//    area->addVertex(Point(600,-600));
//    area->addVertex(Point(-600,-600));
//    area->addVertex(Point(-600,600));
//
//    map->addRoad(Path(LineSegment(Point(600,600), Point(600,-600))));
//    map->addRoad(Path(LineSegment(Point(600,-600), Point(-600,-600))));
//    map->addRoad(Path(LineSegment(Point(-600,-600), Point(-600,600))));
//    map->addRoad(Path(LineSegment(Point(-600,600), Point(600,600))));

  OrganicRoadPattern* generator = new OrganicRoadPattern();

  generator->setTarget(map);
  generator->setAreaConstraints(area);
  generator->setRoadType(Road::PRIMARY_ROAD);
  //generator->setRoadLength(600, 800);
  //generator->setSnapDistance(200);
  generator->setRoadLength(1200, 2000);
  generator->setSnapDistance(800);
  generator->setTurnAngle(60, 90);

  generator->setInitialPosition(area->centroid());

  generator->generate();

  map->removeFilamentRoads();
}

void OgreCity::createZones()
{
  *zones = map->findZones();
}

void OgreCity::createSecondaryRoadNetwork()
{
  for (std::list<Zone*>::iterator zonesIterator = zones->begin();
       zonesIterator != zones->end();
       zonesIterator++)
  {
    RoadLSystem* generator = new RasterRoadPattern;
    generator->setTarget(map);

    generator->setAreaConstraints(new Polygon((*zonesIterator)->areaConstraints()));
    generator->setRoadType(Road::SECONDARY_ROAD);
    generator->setInitialPosition((*zonesIterator)->areaConstraints().centroid());
    generator->setInitialDirection(Vector(0.2, 0.3));
    generator->setRoadLength(200, 230);
    generator->setSnapDistance(100);
    generator->setTurnAngle(90, 90);
    generator->generate();

    delete generator;
  }
  map->removeFilamentRoads();
}

void OgreCity::createBlocks()
{
  std::map<Road::Type, double> roadWidths;
  roadWidths[Road::PRIMARY_ROAD] = primaryRoad.width;
  roadWidths[Road::SECONDARY_ROAD] = secondaryRoad.width;

  for (std::list<Zone*>::iterator zone = zones->begin();
       zone != zones->end();
       zone++)
  {
    (*zone)->createBlocks(roadWidths);
     std::list<Block*> blocks = (*zone)->getBlocks();
     for (std::list<Block*>::iterator blocksIterator = blocks.begin();
          blocksIterator != blocks.end();
          blocksIterator++)
     {
       (*blocksIterator)->createLots(150,150,0);
     }
  }
}

void OgreCity::createBuildings()
{}

void OgreCity::render()
{
  generate();

  renderRoadNetwork();
  renderBuildings();
}

void OgreCity::renderRoadNetwork()
{
  StreetGraphRenderer* streetGraphRenderer = new StreetGraphRenderer(sceneManager);
  streetGraphRenderer->setStreetGraph(map);
  streetGraphRenderer->setTerrain(terrain);

  streetGraphRenderer->setRoadSampleLength(1);
  streetGraphRenderer->setNumberOfVerticesPerSample(5);

  streetGraphRenderer->setRoadParameters(Road::PRIMARY_ROAD, primaryRoad);
  streetGraphRenderer->setRoadParameters(Road::SECONDARY_ROAD, secondaryRoad);
  streetGraphRenderer->render();

//  flattenTerrainUnderRoads();
}

void OgreCity::renderBuildings()
{
  return;
  Point cityCenter = area->centroid();
  double distanceToCenter;

  Random generator;
  Ogre::SceneNode* zoneNode, * blockNode;

  int i = 0;
  for (std::list<Zone*>::iterator zonesIterator = zones->begin();
       zonesIterator != zones->end();
       zonesIterator++)
  {
   zoneNode = sceneManager->getRootSceneNode()->createChildSceneNode();

   std::list<Block*> blocks = (*zonesIterator)->getBlocks();
   for (std::list<Block*>::iterator blocksIterator = blocks.begin();
        blocksIterator != blocks.end();
        blocksIterator++)
   {
     blockNode = zoneNode->createChildSceneNode();

     std::list<Lot*> lots = (*blocksIterator)->getLots();
     for (std::list<Lot*>::iterator lotsIterator = lots.begin();
        lotsIterator != lots.end();
        lotsIterator++, i++)
     {
       OgreBuilding* building;

       /* Discard small lots. */
       if ((*lotsIterator)->areaConstraints().area() < 5000) continue;


       debug("lot: " << i);
       switch (generator.generateInteger(0,4))
       {
         case 0:
           building = new SkyScraper(*lotsIterator, sceneManager, blockNode);
           break;
         case 1:
         case 2:
         case 3:
         case 4:
           building = new OlderBuilding(*lotsIterator, sceneManager, blockNode);
           break;
         default:
           assert("Wrong building configuration");
       }

       distanceToCenter = Vector(cityCenter, (*lotsIterator)->areaConstraints().vertex(0)).length();
       if (distanceToCenter > 500)
       {
          building->setMaxHeight(generator.generateInteger(3, 7) * 2.5 * OgreCity::meter);
       }
       else
       {
          building->setMaxHeight(generator.generateInteger(20, 30) * 2.5 * OgreCity::meter);
       }
       //building->setMaxHeight(50 * 2.5 * OgreCity::meter);

       if (generator.generateBool(0.8))
       {
         building->render();
       }
       delete building;
     }
   }
  }
}

Ogre::Vector3 OgreCity::libcityToOgre(Point const& point)
{
  return Ogre::Vector3(point.x(), point.z(), point.y());
}

Ogre::Vector3 OgreCity::libcityToOgre(Vector const& vector)
{
  return Ogre::Vector3(vector.x(), vector.z(), vector.y());
}

Vector OgreCity::ogreToLibcity(Ogre::Vector3 const& vector)
{
  return Vector(vector.x, vector.y, vector.z);
}

void OgreCity::setTerrain(TerrainRenderer* renderer)
{
  terrain = renderer->getTerrainObject();
}
