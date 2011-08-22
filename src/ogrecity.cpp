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

#include <iostream>

#include "ogrecity.h"
#include "terrainrenderer.h"
#include "streetgraphrenderer.h"
#include "skyscraper.h"
#include "olderbuilding.h"
#include "redbuilding.h"
#include "suburbanhouse.h"

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
  printConsoleMessage("STAGE 1: Generating primary road network ...");

  OrganicRoadPattern* generator = new OrganicRoadPattern();

  generator->setTarget(map);
  generator->setAreaConstraints(area);
  generator->setRoadType(Road::PRIMARY_ROAD);
  //generator->setRoadLength(600, 800);
  //generator->setSnapDistance(200);
  generator->setRoadLength(900, 1200);
  generator->setSnapDistance(400);

  generator->setInitialPosition(area->centroid());
  generator->generate();

  /* This is important to avoid filament roads to go under buildings and such. */
  map->removeFilamentRoads();
}

void OgreCity::createZones()
{
  printConsoleMessage("STAGE 2: Creating city zones ...");

  *zones = map->findZones();
}

void OgreCity::createSecondaryRoadNetwork()
{
  printConsoleMessage("STAGE 3: Generating secondary road network ...");

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
    generator->setRoadLength(300, 330);
    generator->setSnapDistance(150);
    generator->generate();

    delete generator;
  }
  map->removeFilamentRoads();

  printConsoleMessage("         Total of " + convertToString(map->getRoads().size()) + " road segments.");
}

void OgreCity::createBlocks()
{
  printConsoleMessage("STAGE 4: Creating blocks and allotments ...");

  std::map<Road::Type, double> roadWidths;
  roadWidths[Road::PRIMARY_ROAD] = primaryRoad.width;
  roadWidths[Road::SECONDARY_ROAD] = secondaryRoad.width;

  int numberOfAllotments = 0, numberOfBlocks = 0;
  for (std::list<Zone*>::iterator zone = zones->begin();
       zone != zones->end();
       zone++)
  {
    (*zone)->createBlocks(roadWidths);
     std::list<Block*> blocks = (*zone)->getBlocks();
     numberOfBlocks += blocks.size();
     for (std::list<Block*>::iterator blocksIterator = blocks.begin();
          blocksIterator != blocks.end();
          blocksIterator++)
     {
       (*blocksIterator)->createLots(allotmentWidth,allotmentDepth,0);
       numberOfAllotments += (*blocksIterator)->getLots().size();
     }
  }

  printConsoleMessage("         " + convertToString(zones->size()) + " zones.");
  printConsoleMessage("         " + convertToString(numberOfBlocks) + " blocks.");
  printConsoleMessage("         " + convertToString(numberOfAllotments) + " allotments.");
}

void OgreCity::createBuildings()
{
  printConsoleMessage("STAGE 5: Generating buildings ...");

  /* Optimized into renderBuildings() so we dont need to go
     through the list again. */
}

void OgreCity::render()
{
  printConsoleMessage("Generating city ...");
  generate();

  printConsoleMessage("Rendering city ...");

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
       printConsoleMessage("Building " + convertToString(i));

       OgreBuilding* building;

       /* Discard small lots. */
       if ((*lotsIterator)->areaConstraints().area() < 5000) continue;

       distanceToCenter = Vector(cityCenter, (*lotsIterator)->areaConstraints().vertex(0)).length();

       /* City center, that means
          sky scrapers, older tall buildings,
          occasionaly some small historic building. */
       if (distanceToCenter < 1500)
       {
         switch (generator.generateInteger(0,2))
         {
           case 0:
             building = new SkyScraper(*lotsIterator, sceneManager, blockNode);

             if (distanceToCenter < 500)
             {
               if (generator.generateBool(0.7))
               {
                 building->setMaxHeight(generator.generateInteger(20, 30) * 2.5 * OgreCity::meter);
               }
               else
               {
                 building->setMaxHeight(generator.generateInteger(10, 15) * 2.5 * OgreCity::meter);
               }
             }
             else if (distanceToCenter < 1000)
             {
                building->setMaxHeight(generator.generateInteger(5, 20) * 2.5 * OgreCity::meter);
             }
             break;
           case 1:
             building = new RedBuilding(*lotsIterator, sceneManager, blockNode);

             if (distanceToCenter < 500)
             {
               if (generator.generateBool(0.7))
               {
                 building->setMaxHeight(generator.generateInteger(15, 25) * 2.5 * OgreCity::meter);
               }
               else
               {
                 building->setMaxHeight(generator.generateInteger(7, 10) * 2.5 * OgreCity::meter);
               }
             }
             else if (distanceToCenter < 1000)
             {
                building->setMaxHeight(generator.generateInteger(3, 15) * 2.5 * OgreCity::meter);
             }
             break;
           case 2:
             building = new OlderBuilding(*lotsIterator, sceneManager, blockNode);


             if (distanceToCenter < 500)
             {
               if (generator.generateBool(0.7))
               {
                 building->setMaxHeight(generator.generateInteger(10, 15) * 2.5 * OgreCity::meter);
               }
               else
               {
                 building->setMaxHeight(generator.generateInteger(7, 10) * 2.5 * OgreCity::meter);
               }
             }
             else if (distanceToCenter < 1000)
             {
                building->setMaxHeight(generator.generateInteger(3, 15) * 2.5 * OgreCity::meter);
             }
             break;
           default:
             assert("Wrong building configuration");
         }
       }
       else
       /* Peripheral and suburban areas.
          Small residential houses. Familly houses. */
       {
         switch (generator.generateInteger(0,3))
         {
           case 0:
             building = new RedBuilding(*lotsIterator, sceneManager, blockNode);
             break;
           case 1:
           case 2:
             building = new SuburbanHouse(*lotsIterator, sceneManager, blockNode);
             break;
           case 3:
             building = new OlderBuilding(*lotsIterator, sceneManager, blockNode);
             break;
           default:
             assert("Wrong building configuration");
         }

          building->setMaxHeight(generator.generateInteger(3, 7) * 2.5 * OgreCity::meter);
       }

       building->render();
       delete building;
     }
   }
  }

  printConsoleMessage("Total of " + convertToString(i) + " buildings rendered.");
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
  return Vector(vector.x, vector.z, vector.y);
}

void OgreCity::setTerrain(TerrainRenderer* renderer)
{
  terrain = renderer->getTerrainObject();
}

void OgreCity::printConsoleMessage(Ogre::String message)
{
  std::cout << message << std::endl;
}

Ogre::String OgreCity::convertToString(int number)
{
  Ogre::StringStream convertor;

  convertor << number;
  return convertor.str();
}
