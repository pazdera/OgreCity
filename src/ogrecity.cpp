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
  area->addVertex(Point(3000,3000));
  area->addVertex(Point(3000,-3000));
  area->addVertex(Point(-3000,-3000));
  area->addVertex(Point(-3000,3000));

  map->addRoad(Path(LineSegment(Point(3000,3000), Point(3000,-3000))));
  map->addRoad(Path(LineSegment(Point(3000,-3000), Point(-3000,-3000))));
  map->addRoad(Path(LineSegment(Point(-3000,-3000), Point(-3000,3000))));
  map->addRoad(Path(LineSegment(Point(-3000,3000), Point(3000,3000))));


//  map->addRoad(Path(LineSegment(Point(-160.567, -2076.97, 0), Point(-75.5349, -3000, 0))));
//  map->addRoad(Path(LineSegment(Point(2219.48, -3000, 0), Point(-75.5349, -3000, 0))));
//  map->addRoad(Path(LineSegment(Point(-75.5349, -3000, 0), Point(-2060.82, -3000, 0))));

//  map->addRoad(Path(LineSegment(Point(0,0), Point(0,100))));
//  map->addRoad(Path(LineSegment(Point(0,0), Point(-100,-100))));
//  map->addRoad(Path(LineSegment(Point(0,0), Point(100,-100))));
//  map->addRoad(Path(LineSegment(Point(-300,0), Point(300,0))));
//  map->addRoad(Path(LineSegment(Point(-300,300), Point(-150,0))));
//  map->addRoad(Path(LineSegment(Point(-300,-300), Point(150,0))));



  OrganicRoadPattern* generator = new OrganicRoadPattern();

  generator->setTarget(map);
  generator->setAreaConstraints(area);
  generator->setRoadType(Road::PRIMARY_ROAD);
  generator->setRoadLength(2000, 2500);
  generator->setSnapDistance(800);
  generator->setTurnAngle(60, 90);

  generator->setInitialPosition(Point(-2800, 0));

  generator->generate();
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
}

void OgreCity::createBlocks()
{
  std::map<Road::Types, double> roadWidths;
  roadWidths[Road::PRIMARY_ROAD] = primaryRoad.width;
  roadWidths[Road::SECONDARY_ROAD] = secondaryRoad.width;

  for (std::list<Zone*>::iterator zone = zones->begin();
       zone != zones->end();
       zone++)
  {
    (*zone)->createBlocks(roadWidths);
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
{}

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
