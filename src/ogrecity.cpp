#include "ogrecity.h"

OgreCity::OgreCity(Ogre::SceneManager* sceneManagerObject, Ogre::Terrain* terrainObject)
  : sceneManager(sceneManagerObject), terrain(terrainObject)
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
{}

void OgreCity::createBuildings()
{}

void OgreCity::drawRoadNetwork()
{
  Road* road;

  precomputeIntersectionsVertices();
  for (StreetGraph::iterator roadIterator = map->begin();
       roadIterator != map->end();
       roadIterator++)
  {
    road = *roadIterator;

    drawRoad(road, getRoadParameters(road), terrain);
  }

//  flattenTerrainUnderRoads();
}

void OgreCity::drawBuildings()
{}

Ogre::SimpleSpline OgreCity::mapPathToTerrain(Path* path)
{
  Ogre::SimpleSpline mappedPath;
  mappedPath.addPoint(libcityToOgre(path->begining()));
  mappedPath.addPoint(libcityToOgre(path->end()));

  return mappedPath;
}

void OgreCity::drawRoad(Road* road, RoadParameters parameters, Ogre::Terrain* terrain)
{
  /* Set parameters */
  Ogre::Real roadWidth = parameters.width; // m
  Ogre::Real roadHeight = parameters.height; // m

  Ogre::Real roadSampleSize = parameters.sampleSize;
  assert(roadSampleSize > 0);

  int numberOfHorizontalPoints = parameters.numberOfSampleVertices;
  assert(numberOfHorizontalPoints % 2 == 1);

  std::vector<double> textureXCoordinate = parameters.verticesTextureMapping;
  assert(textureXCoordinate.size() == numberOfHorizontalPoints);

  Ogre::Real textureYCoordinate = 0;

  std::vector<double> vertexOffset = parameters.verticesPositionOffset;
  assert(vertexOffset.size() == numberOfHorizontalPoints);

  /* Generate unique alias for naming Ogre entities */
  static int roadNumber = 0;
  Ogre::String roadName;
  Ogre::StringStream convertor;
  convertor << roadNumber;
  roadName = convertor.str();
  roadNumber++;

  /* Modify path according to the surrounding terrain environment */
  Ogre::SimpleSpline path;

  /* Modify begining and end if there's some precomputed intersection info */
  Point* beginingVertices = 0;
  if (intersectionVertices.find(road->begining()) != intersectionVertices.end())
  {
    beginingVertices = intersectionVertices[road->begining()][road];

    path.addPoint(libcityToOgre(beginingVertices[numberOfHorizontalPoints]));
  }
  else
  {
    path.addPoint(libcityToOgre(road->path()->begining()));
  }

  Point* endVertices = 0;
  if (intersectionVertices.find(road->end()) != intersectionVertices.end())
  {
    endVertices = intersectionVertices[road->end()][road];

    path.addPoint(libcityToOgre(endVertices[numberOfHorizontalPoints]));
  }
  else
  {
    path.addPoint(libcityToOgre(road->path()->end()));
  }

  /* Generate mesh */
  Ogre::ManualObject roadObject(roadName + "obj");
  roadObject.begin(parameters.material);

  // Temporary variables
  Ogre::Real roadSegmentLength;
  int samplesInThisSegment;
  int numberOfSamples;

  // for keeping track of the sides
  Ogre::Vector3 normalVector;
  Ogre::Vector3 pointFromPath;
  Ogre::Vector3 previousPointFromPath;

  Ogre::Real t; // Interpolation parameter

  int numberOfPoints; // Number of spline control points
  int sampleIndex, vertexIndex;

  // To add the mesh to the scene.
  Ogre::Entity* roadEntity;
  Ogre::SceneNode* roadSceneNode;

  // Gets the Road node if it's already there.
  try
  {
    roadSceneNode = sceneManager->getSceneNode("RoadNode");
  }
  catch(Ogre::Exception e)
  {
    roadSceneNode = sceneManager->getRootSceneNode()->createChildSceneNode("RoadNode");
  }

  // check here if we can load it from saved cache / prebuilt

  // Define the vertices
  numberOfPoints = path.getNumPoints();

  // we need to count the number of segments totally
  numberOfSamples = -2; // minus first two and last (intersections)

  for (int i = 0; i < numberOfPoints - 1; i++)
  {
    roadSegmentLength = (path.getPoint(i + 1) - path.getPoint(i)).length();

    samplesInThisSegment = roadSegmentLength / roadSampleSize;
    if (samplesInThisSegment == 0)
    {
      samplesInThisSegment++;
    }

    numberOfSamples += samplesInThisSegment;
  }

  if (numberOfSamples <= 0)
  {
    return;
  }

  Ogre::Vector3 sampleBegining;
  Ogre::Vector3 sampleEnd;
  Ogre::Real sampleLength;
  Ogre::Vector3 directionVector;
  int cnt=0;

  Ogre::Real currHeight;
  Ogre::Vector3 newVertices[5];

  // for some reason the first section may only have 1 segment... in this case we need to have lastPt defined
  previousPointFromPath = path.interpolate(0,0.0);
  sampleBegining = previousPointFromPath;

  for (int pointIndex = 0; pointIndex < numberOfPoints-1; pointIndex++)
  /* For each road segment */
  {
    roadSegmentLength = (path.getPoint(pointIndex + 1) - path.getPoint(pointIndex)).length();
    samplesInThisSegment = roadSegmentLength / roadSampleSize;
    if (samplesInThisSegment == 0)
    {
      samplesInThisSegment++;
    }

    // Skip first sample
    sampleIndex = (pointIndex == 0) ? 1 : 0;

    for (; sampleIndex < samplesInThisSegment; sampleIndex++)
    /* For each sample in roadSegment */
    {
        t = static_cast<Ogre::Real>(sampleIndex) / static_cast<Ogre::Real>(samplesInThisSegment);
        pointFromPath = path.interpolate(pointIndex,t); // interpolated position of next sample

        /* sampleBegining is set from previous iteration */
        sampleEnd = pointFromPath;

        sampleLength = (sampleEnd - sampleBegining).length();
        textureYCoordinate += sampleLength/1.654f; //mTerrainMesh->getTextureRepeat();
        sampleBegining = sampleEnd; // set begining for the next sample

        currHeight = pointFromPath.y; //mTerrainMesh->getHeight(vertex);
        previousPointFromPath.y = pointFromPath.y = 0;

        // positive is left vector
        directionVector = (pointFromPath - previousPointFromPath);
        normalVector = directionVector.crossProduct(Ogre::Vector3::UNIT_Y);
        normalVector *= -1;
        normalVector.normalise();
        // end of lrVector calculation

        for (vertexIndex = 0; vertexIndex < numberOfHorizontalPoints; vertexIndex++)
        {
          // create this side piece
          newVertices[vertexIndex] = pointFromPath;
          newVertices[vertexIndex] += normalVector * vertexOffset[vertexIndex] * roadWidth;

          newVertices[vertexIndex].y = terrain->getHeightAtWorldPosition(newVertices[vertexIndex].x, 0, newVertices[vertexIndex].z);

//          // update height
//          newVertices[vertexIndex].y = terrain->getHeightAtWorldPosition(newVertices[vertexIndex].x, 0, newVertices[vertexIndex].z) + roadHeight;
//          if (newVertices[vertexIndex].y > currHeight
//              && vertexIndex != 0
//              && vertexIndex != numberOfHorizontalPoints-1)
//          {
//              currHeight = newVertices[vertexIndex].y;
//          }
        }

        for (vertexIndex = 1; vertexIndex < numberOfHorizontalPoints-1; vertexIndex++)
        {
          // update height
          newVertices[vertexIndex].y = terrain->getHeightAtWorldPosition(newVertices[vertexIndex].x, 0, newVertices[vertexIndex].z) + roadHeight;
        }

        /* Here we inject begining and end vertices */
        if (pointIndex == 0 && sampleIndex == 1
            && beginingVertices != 0) /* first run */
        {
          for (int i = 0; i < numberOfHorizontalPoints; i++)
          {
            beginingVertices[i].setZ(newVertices[i].y);
            newVertices[i] = libcityToOgre(beginingVertices[i]);
          }
        }

        if ((pointIndex+1) >= (numberOfPoints-1) && (sampleIndex+1) >= samplesInThisSegment
            && endVertices != 0) /* last run */
        {
          for (int i = 0; i < numberOfHorizontalPoints; i++)
          {
            endVertices[i].setZ(newVertices[i].y);
            newVertices[i] = libcityToOgre(endVertices[i]);
          }
        }

        previousPointFromPath = newVertices[numberOfHorizontalPoints/2];

        // finally add vertices
        for (vertexIndex = 0; vertexIndex < numberOfHorizontalPoints; vertexIndex++)
        {
          // get vertex from created values
          roadObject.position(newVertices[vertexIndex]);
          roadObject.textureCoord(textureXCoordinate[vertexIndex], textureYCoordinate);
        }
    }
  }

  /* Define object face */
  /* Face will be defined by a set of 8 triangles. */
  for (sampleIndex = 0; sampleIndex < numberOfSamples; sampleIndex++)
  {
    // create four: 0,1,4 0,4,3 1,2,5 1,5,4 (0->6 + offset)
    for (vertexIndex = sampleIndex*numberOfHorizontalPoints; vertexIndex < (sampleIndex+1)*numberOfHorizontalPoints-1; vertexIndex++) // vertexIndex
    /* Each iteration creates two triangles (one quad) */
    {
        unsigned short face[3*2] = { /* First trianlge */
                                     vertexIndex + 0,
                                     vertexIndex + numberOfHorizontalPoints + 1,
                                     vertexIndex + 1,

                                     /* Second trianlge */
                                     vertexIndex + 0,
                                     vertexIndex + numberOfHorizontalPoints,
                                     vertexIndex + numberOfHorizontalPoints + 1
                                   };

        /* Define triangles by vertex indexes */
        roadObject.triangle(face[2], face[1], face[0]);
        roadObject.triangle(face[5], face[4], face[3]);
    }
  }

  roadObject.end();
  roadObject.convertToMesh(roadName + "Mesh");

  // add mesh to scene
  roadEntity = sceneManager->createEntity(roadName + "Ent", roadName + "Mesh");
  roadEntity->setRenderQueueGroupAndPriority(10, 1000);
  roadSceneNode = roadSceneNode->createChildSceneNode();
  roadSceneNode->attachObject(roadEntity);

  return;
}

void OgreCity::flattenTerrainUnderRoads()
{
  Ogre::SimpleSpline path;
  int numberOfSamples = 0;
  Road* road;

  for (StreetGraph::iterator roadIterator = map->begin();
       roadIterator != map->end();
       roadIterator++)
  {
    road = *roadIterator;

    path = mapPathToTerrain(road->path());

    for (int pointIndex = 0; pointIndex < path.getNumPoints()-1; pointIndex++)
    {
      numberOfSamples = (path.getPoint(pointIndex+1) - path.getPoint(pointIndex)).length();
      if (numberOfSamples <= 0)
      {
        numberOfSamples = 1;
      }

      for (int sampleIndex = 0; sampleIndex < numberOfSamples; sampleIndex++)
      {
        Ogre::Real t = static_cast<Ogre::Real>(sampleIndex) / static_cast<Ogre::Real>(numberOfSamples);
        Ogre::Vector3 pointFromPath = path.interpolate(pointIndex,t); // interpolated position of next sample


        // modify terrain under the road to avoid intersections
        Ogre::Vector3 terrainPosition;
        terrain->getTerrainPosition(pointFromPath.x, 0, pointFromPath.z, &terrainPosition);
        terrain->setHeightAtPoint(terrainPosition.x * terrain->getSize(), terrainPosition.y * terrain->getSize(), terrainPosition.z - 1);
      }

    }

  }

  terrain->update();
}


void OgreCity::precomputeIntersectionsVertices()
{
  StreetGraph::Intersections intersections = map->getIntersections();
  std::vector<Road*> roadsOfCurrentIntersection;
  Intersection* intersection;

  for (StreetGraph::Intersections::iterator intersectionsIterator = intersections.begin();
       intersectionsIterator != intersections.end();
       intersectionsIterator++)
  {
    intersection = *intersectionsIterator;

    /* Copy roads of current intersection into a vector */
    roadsOfCurrentIntersection.clear();
    StreetGraph::Roads roadList = intersection->getRoads();
    roadsOfCurrentIntersection.assign(roadList.begin(), roadList.end());

    switch(roadsOfCurrentIntersection.size())
    {
      case 1: // Do nothing
        break;
      case 2:
      {
        sortIntersectingRoadsCounterclockwise(intersection, &roadsOfCurrentIntersection);

        intersectionVertices[intersection][roadsOfCurrentIntersection[0]] = new Point[primaryRoad.numberOfSampleVertices+1];
        intersectionVertices[intersection][roadsOfCurrentIntersection[1]] = new Point[primaryRoad.numberOfSampleVertices+1];

        Vector firstDirection, firstNormal;
        Vector secondDirection, secondNormal;
        Point firstOrigin, secondOrigin;
        bool firstIndexInverted = false, secondIndexInverted = false;

        Ray first  = getExtensionRay(intersection, roadsOfCurrentIntersection[0]);
        firstOrigin = first.origin();
        firstDirection = first.direction();
        firstNormal = firstDirection.crossProduct(Vector(0,0,1));
        firstNormal.normalize();

        Ray second = getExtensionRay(intersection, roadsOfCurrentIntersection[1]);
        secondOrigin = second.origin();
        secondDirection = second.direction();
        secondNormal = secondDirection.crossProduct(Vector(0,0,1));
        secondNormal.normalize();

        bool firstIsBegining  = intersection == roadsOfCurrentIntersection[0]->begining(),
             secondIsBegining = intersection == roadsOfCurrentIntersection[1]->begining();

        int firstMatchIndex, secondMatchIndex, firstStoreIndex, secondStoreIndex;
        for (int vertexIndex = 0; vertexIndex < primaryRoad.numberOfSampleVertices; vertexIndex++)
        {
          if (firstIsBegining == secondIsBegining)
          {
            if (firstIsBegining)
            {
              firstMatchIndex  = vertexIndex;
              secondMatchIndex = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
            }
            else
            {
              firstMatchIndex  = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
              secondMatchIndex = vertexIndex;
            }

            firstStoreIndex = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
            secondStoreIndex = vertexIndex;
          }
          else
          {
            if (firstIsBegining)
            {
              firstMatchIndex  = vertexIndex;
              secondMatchIndex = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
            }
            else
            {
              firstMatchIndex  = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
              secondMatchIndex = vertexIndex;
            }

            firstStoreIndex = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
            secondStoreIndex = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
          }
        firstNormal = firstDirection.crossProduct(Vector(0,0,1));
        firstNormal.normalize();
        secondNormal = secondDirection.crossProduct(Vector(0,0,1));
        secondNormal.normalize();

          // Create Rays for both roads
          first.setOrigin(firstOrigin + firstNormal*primaryRoad.verticesPositionOffset[firstMatchIndex]*primaryRoad.width);
          second.setOrigin(secondOrigin + secondNormal*primaryRoad.verticesPositionOffset[secondMatchIndex]*primaryRoad.width);
          Point raysIntersection;
          Ray::Intersection intersectionResult;

          intersectionResult = first.intersection2D(second, &raysIntersection);
          debug("Ray1: " << first.toString());
          debug("Ray2: " << second.toString());
          debug("Intersection: " << raysIntersection.toString());
          if (intersectionResult == Ray::PARALLEL)
          {
            raysIntersection = intersection->position() + firstNormal*primaryRoad.verticesPositionOffset[firstMatchIndex]*primaryRoad.width;
          }
          assert(intersectionResult == Ray::INTERSECTING || intersectionResult == Ray::PARALLEL);



          intersectionVertices[intersection][roadsOfCurrentIntersection[0]][firstStoreIndex] = raysIntersection;
          intersectionVertices[intersection][roadsOfCurrentIntersection[1]][secondStoreIndex] = raysIntersection;
        }
        double hypotenuse = Vector(intersectionVertices[intersection][roadsOfCurrentIntersection[0]][0],
                                   intersectionVertices[intersection][roadsOfCurrentIntersection[0]][primaryRoad.numberOfSampleVertices - 1]).length();
        double opposedLeg = primaryRoad.width;
        double adjacentLeg = opposedLeg/std::tan(std::asin(opposedLeg/hypotenuse));
        double overlap = adjacentLeg/2;

        first  = getExtensionRay(intersection, roadsOfCurrentIntersection[0]),
        second = getExtensionRay(intersection, roadsOfCurrentIntersection[1]);
        intersectionVertices[intersection][roadsOfCurrentIntersection[0]][primaryRoad.numberOfSampleVertices] = getBeginingThatAvoidsOverlap(intersection, roadsOfCurrentIntersection[0], primaryRoad);
        intersectionVertices[intersection][roadsOfCurrentIntersection[1]][primaryRoad.numberOfSampleVertices] = getBeginingThatAvoidsOverlap(intersection, roadsOfCurrentIntersection[1], primaryRoad);
      };
        break;
      case 3:
        {
          debug("3-way intersection " << intersection->position().toString());
          sortIntersectingRoadsCounterclockwise(intersection, &roadsOfCurrentIntersection);

          determineMiddleRoad(intersection, &roadsOfCurrentIntersection);

          const int NUMBER_OF_WAYS = 3;
          Vector normals[NUMBER_OF_WAYS];
          Ray  baseRoadRays[NUMBER_OF_WAYS];

          std::vector<Line> projectedVertexRays[NUMBER_OF_WAYS];
          int indexesInverted[NUMBER_OF_WAYS];

          /* Load and prepare information about all roads of the intersection. */
          for(int way = 0; way < NUMBER_OF_WAYS; way++)
          {
            Point roadBegining = roadsOfCurrentIntersection[way]->begining()->position(),
                  roadEnd = roadsOfCurrentIntersection[way]->end()->position();

            /* It's importat that the vector is same as when the road is being rendered.
               Roads are rendered from begining to end. */
            Vector roadDirectionVector = Vector(roadBegining, roadEnd);

            /* Cross product of y and z axis will yield normal vector. */
            normals[way] = roadDirectionVector.crossProduct(Vector(0,0,1));
            normals[way].normalize();

            /* Ray, that is in the centre of the road. */
            baseRoadRays[way] = getExtensionRay(intersection, roadsOfCurrentIntersection[way]);

            /* Project the ray for each horizontal vertex of the road. */
            indexesInverted[way] = false;
            int index = 0;
            for (int vertexIndex = 0; vertexIndex < primaryRoad.numberOfSampleVertices; vertexIndex++)
            {
              /* Very important step here is to invert the horizontal vertices if
               the road starts in the intersection. We need to know wheter the
               road was flipped so we can flip it back when it comes to storing
               the vertices back.*/
              if (intersection == roadsOfCurrentIntersection[way]->begining())
              {
                index = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
                indexesInverted[way] = true;
              }
              else /* if (intersection == roadsOfCurrentIntersection[way]->end()) */
              {
                index = vertexIndex;
              }

              /* Direction vector is always the same, but the origin changes. */
              Vector originOffset(normals[way]*primaryRoad.verticesPositionOffset[index]*primaryRoad.width);
              Point newOrigin = baseRoadRays[way].origin() + originOffset;

              /* Line was used instead of ray, due to some pretty nasty bugs,
               but the use is the same. */
              Line projectedRay(newOrigin, baseRoadRays[way].direction());
              projectedVertexRays[way].push_back(projectedRay);
            }
          }

          /* References for each road's vertex lines. This is
           here just to improve readability of the code below. */
          std::vector<Line>& first = projectedVertexRays[0];
          std::vector<Line>& second = projectedVertexRays[1];
          std::vector<Line>& third = projectedVertexRays[2];

          /* The four point that form the shape of the intersection. */
          Point topLeft, topRight,
                bottomLeft, bottomRight;

          /* For storing results of various intersection tests for
           assertions. This isn't really neccessary, but it helps debugging. */
          Line::Intersection intersectionResult;

          /* Find upper two points */
          debug("Finding top right point: ");
          intersectionResult = first[0].intersection2D(second[primaryRoad.numberOfSampleVertices - 1], &topRight);
          assert(intersectionResult == Line::INTERSECTING);
          debug("  First road ray: " << first[0].toString());
          debug("  Second road ray: " << second[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Top right point: " << topRight.toString());

          debug("Finding top left point: ");
          intersectionResult = second[0].intersection2D(third[primaryRoad.numberOfSampleVertices - 1], &topLeft);
          assert(intersectionResult == Line::INTERSECTING);
          debug("  Second road ray: " << second[0].toString());
          debug("  Third road ray: " << third[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Top left point: " << topLeft.toString());

          /* Find bottom line */
          Vector topLineNormal = Vector(topLeft, topRight).crossProduct(Vector(0,0,1));
          Point normalTestPoint = Point((topLeft.x() + topRight.x())/2, (topLeft.y() + topRight.y())/2); // Line center

          /* The orientations of normal vector to a Line in 2D can be two.
           Depending on what order of points we use to create direction vector.
           In this case we need the normal, that will point "inside" the intersection.
           We create two points (each at the end of proposed normal) and test the
           distance between each of them and the position of the intersection. The
           closer points wins and the coresponding normal is used.
           */
          Vector testPlus, testMinus;
          testPlus.set(intersection->position(), normalTestPoint + topLineNormal);
          testMinus.set(intersection->position(), normalTestPoint + topLineNormal*(-1));

          if (testMinus.length() < testPlus.length())
          /* The wrong normal was chosen. Invert it. */
          {
            topLineNormal = topLineNormal*(-1);
          }

          topLineNormal.normalize();

          /* Proposed bottom points. */
          bottomLeft  = topLeft + topLineNormal*primaryRoad.width*2;
          bottomRight = topRight + topLineNormal*primaryRoad.width*2;

          Vector bottomLineDirection = Vector(bottomLeft, bottomRight);
          bottomLineDirection.normalize();

          Line bottom(bottomLeft, bottomRight);
          debug("Bottom line: " << bottom.toString());

          Point raysIntersection;

          /* Adjust the bottom two points */
          debug("Find bottom right point: ");
          intersectionResult = first[primaryRoad.numberOfSampleVertices - 1].intersection2D(bottom, &raysIntersection);
          assert(intersectionResult != Line::NONINTERSECTING);
          if (intersectionResult == Line::INTERSECTING)
          /* When the lines aren't parallel adjust the bottom point. */
          {
            bottomRight = raysIntersection;
          }
          debug("  Bottom line: " << bottom.toString());
          debug("  Road ray: " << first[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Bottom right: " << bottomRight.toString());

          debug("Find bottom left point: ");
          intersectionResult = third[0].intersection2D(bottom, &raysIntersection);
          assert(intersectionResult != Line::NONINTERSECTING);
          if (intersectionResult == Line::INTERSECTING)
          /* When the lines aren't parallel adjust the bottom point. */
          {
            bottomLeft = raysIntersection;
          }
          debug("  Bottom line: " << bottom.toString());
          debug("  Road ray: " << third[0].toString());
          debug("  Bottom left: " << bottomLeft.toString());

          /* Check if the points didnt swap. This could happen
           when the angles between all the roads are the same. */
          Vector newBottomLineDirection(bottomLeft, bottomRight);
          newBottomLineDirection.normalize();
          if (bottomLineDirection != newBottomLineDirection)
          {
            bottomLeft = bottomRight = Point((bottomLeft.x() + bottomRight.x())/2, (bottomLeft.y() + bottomRight.y())/2);
          }

          /* Now we have the 3 rays we want cut our roads with. */
          Line constraints[NUMBER_OF_WAYS];
          constraints[0].set(topRight, bottomRight);
          constraints[1].set(topLeft, topRight);
          constraints[2].set(topLeft, bottomLeft);

          /* Test each road's projectedLines against coresponding constraint
           line and save the intersection vertices. */
          for(int way = 0; way < NUMBER_OF_WAYS; way++)
          {
            intersectionVertices[intersection][roadsOfCurrentIntersection[way]] = new Point[primaryRoad.numberOfSampleVertices+1];

            for (int vertexIndex = 0; vertexIndex < primaryRoad.numberOfSampleVertices; vertexIndex++)
            {
              int index = indexesInverted[way] ? primaryRoad.numberOfSampleVertices - 1 - vertexIndex : vertexIndex;

              intersectionResult = projectedVertexRays[way][vertexIndex].intersection2D(constraints[way], &raysIntersection);
              intersectionVertices[intersection][roadsOfCurrentIntersection[way]][index] = raysIntersection;
              assert(intersectionResult == Line::INTERSECTING);
            }

            /* Modify road begining so it doesn't go all the way into
             the intersection and makes space for the computed border vertices. */
            Point modifiedBegining = getBeginingThatAvoidsOverlap(intersection, roadsOfCurrentIntersection[way], primaryRoad);
            intersectionVertices[intersection][roadsOfCurrentIntersection[way]][primaryRoad.numberOfSampleVertices] = modifiedBegining;
          }
        };
        break;
      case 4:
        {
          debug("4-way intersection " << intersection->position().toString());
          sortIntersectingRoadsCounterclockwise(intersection, &roadsOfCurrentIntersection);

          const int NUMBER_OF_WAYS = 4;
          Vector normals[NUMBER_OF_WAYS];
          Ray  baseRoadRays[NUMBER_OF_WAYS];

          std::vector<Line> projectedVertexRays[NUMBER_OF_WAYS];
          int indexesInverted[NUMBER_OF_WAYS];

          /* Load and prepare information about all roads of the intersection. */
          for(int way = 0; way < NUMBER_OF_WAYS; way++)
          {
            Point roadBegining = roadsOfCurrentIntersection[way]->begining()->position(),
                  roadEnd = roadsOfCurrentIntersection[way]->end()->position();

            /* It's importat that the vector is same as when the road is being rendered.
               Roads are rendered from begining to end. */
            Vector roadDirectionVector = Vector(roadBegining, roadEnd);

            /* Cross product of y and z axis will yield normal vector. */
            normals[way] = roadDirectionVector.crossProduct(Vector(0,0,1));
            normals[way].normalize();

            /* Ray, that is in the centre of the road. */
            baseRoadRays[way] = getExtensionRay(intersection, roadsOfCurrentIntersection[way]);

            /* Project the ray for each horizontal vertex of the road. */
            indexesInverted[way] = false;
            int index = 0;
            for (int vertexIndex = 0; vertexIndex < primaryRoad.numberOfSampleVertices; vertexIndex++)
            {
              /* Very important step here is to invert the horizontal vertices if
               the road starts in the intersection. We need to know wheter the
               road was flipped so we can flip it back when it comes to storing
               the vertices back.*/
              if (intersection == roadsOfCurrentIntersection[way]->begining())
              {
                index = primaryRoad.numberOfSampleVertices - 1 - vertexIndex;
                indexesInverted[way] = true;
              }
              else /* if (intersection == roadsOfCurrentIntersection[way]->end()) */
              {
                index = vertexIndex;
              }

              /* Direction vector is always the same, but the origin changes. */
              Vector originOffset(normals[way]*primaryRoad.verticesPositionOffset[index]*primaryRoad.width);
              Point newOrigin = baseRoadRays[way].origin() + originOffset;

              /* Line was used instead of ray, due to some pretty nasty bugs,
               but the use is the same. */
              Line projectedRay(newOrigin, baseRoadRays[way].direction());
              projectedVertexRays[way].push_back(projectedRay);
            }
          }

          /* References for each road's vertex lines. This is
           here just to improve readability of the code below. */
          std::vector<Line>& first  = projectedVertexRays[0];
          std::vector<Line>& second = projectedVertexRays[1];
          std::vector<Line>& third  = projectedVertexRays[2];
          std::vector<Line>& fourth = projectedVertexRays[3];

          /* The four point that form the shape of the intersection. */
          Point topLeft, topRight,
                bottomLeft, bottomRight;

          /* For storing results of various intersection tests for
           assertions. This isn't really neccessary, but it helps debugging. */
          Line::Intersection intersectionResult;

          /* Find upper two points */
          debug("Finding top right point: ");
          intersectionResult = first[0].intersection2D(second[primaryRoad.numberOfSampleVertices - 1], &topRight);
          if (intersectionResult == Line::PARALLEL)
          {
            topRight = first[0].nearestPoint(intersection->position());
          }
          assert(intersectionResult != Line::NONINTERSECTING);
          debug("  First road ray: " << first[0].toString());
          debug("  Second road ray: " << second[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Top right point: " << topRight.toString());

          debug("Finding top left point: ");
          intersectionResult = second[0].intersection2D(third[primaryRoad.numberOfSampleVertices - 1], &topLeft);
          if (intersectionResult == Line::PARALLEL)
          {
            topLeft = second[0].nearestPoint(intersection->position());
          }
          assert(intersectionResult != Line::NONINTERSECTING);
          debug("  Second road ray: " << second[0].toString());
          debug("  Third road ray: " << third[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Top left point: " << topLeft.toString());

          debug("Finding bottom left point: ");
          intersectionResult = third[0].intersection2D(fourth[primaryRoad.numberOfSampleVertices - 1], &bottomLeft);
          if (intersectionResult == Line::PARALLEL)
          {
            bottomLeft = third[0].nearestPoint(intersection->position());
          }
          assert(intersectionResult != Line::NONINTERSECTING);
          debug("  Third road ray: " << third[0].toString());
          debug("  Fourth road ray: " << fourth[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Bottom left point: " << bottomRight.toString());

          debug("Finding bottom right point: ");
          intersectionResult = fourth[0].intersection2D(first[primaryRoad.numberOfSampleVertices - 1], &bottomRight);
          if (intersectionResult == Line::PARALLEL)
          {
            bottomRight = fourth[0].nearestPoint(intersection->position());
          }
          assert(intersectionResult != Line::NONINTERSECTING);
          debug("  Fourth road ray: " << fourth[0].toString());
          debug("  First road ray: " << first[primaryRoad.numberOfSampleVertices - 1].toString());
          debug("  Bottom right point: " << bottomRight.toString());


          /* Now we have the 3 rays we want cut our roads with. */
          Line constraints[NUMBER_OF_WAYS];
          constraints[0].set(topRight, bottomRight);
          constraints[1].set(topLeft, topRight);
          constraints[2].set(topLeft, bottomLeft);
          constraints[3].set(bottomLeft, bottomRight);

          Point raysIntersection;
          /* Test each road's projectedLines against coresponding constraint
           line and save the intersection vertices. */
          for(int way = 0; way < NUMBER_OF_WAYS; way++)
          {
            intersectionVertices[intersection][roadsOfCurrentIntersection[way]] = new Point[primaryRoad.numberOfSampleVertices+1];

            for (int vertexIndex = 0; vertexIndex < primaryRoad.numberOfSampleVertices; vertexIndex++)
            {
              int index = indexesInverted[way] ? primaryRoad.numberOfSampleVertices - 1 - vertexIndex : vertexIndex;

              intersectionResult = projectedVertexRays[way][vertexIndex].intersection2D(constraints[way], &raysIntersection);
              intersectionVertices[intersection][roadsOfCurrentIntersection[way]][index] = raysIntersection;
              assert(intersectionResult == Line::INTERSECTING);
            }

            /* Modify road begining so it doesn't go all the way into
             the intersection and makes space for the computed border vertices. */
            Point modifiedBegining = getBeginingThatAvoidsOverlap(intersection, roadsOfCurrentIntersection[way], primaryRoad);
            intersectionVertices[intersection][roadsOfCurrentIntersection[way]][primaryRoad.numberOfSampleVertices] = modifiedBegining;
          }
        };
        break;
      default: // Less than one or more than four roads is wrong
        // FIXME turn on assert
        //assert(false);
        break;
    }

  }
}

Ray OgreCity::getExtensionRay(Intersection* intersection, Road* road)
{
  /* Road must go through the intersection */
  assert(intersection->hasRoad(road));

  if (road->begining() == intersection)
  {
    return Ray(road->end()->position(), road->path()->beginingDirectionVector());
    //return Ray(road->begining()->position() + (road->path()->beginingDirectionVector()*primaryRoad.width*(-2)), road->path()->beginingDirectionVector());
  }

  return Ray(road->begining()->position(), road->path()->endDirectionVector());
  //return Ray(road->begining()->position() + (road->path()->endDirectionVector()*primaryRoad.width*(-2)), road->path()->endDirectionVector());
}


double OgreCity::getRoadOverlapIntoIntersection(Intersection* intersection, Road* road, RoadParameters parameters)
{
  assert(intersection->hasRoad(road));

  int firstBorderVertex = 0, lastBorderVertex = parameters.numberOfSampleVertices - 1;
  LineSegment border(intersectionVertices[intersection][road][firstBorderVertex],
              intersectionVertices[intersection][road][lastBorderVertex]);

  double hypotenuse = border.length();
  double opposedLeg = parameters.width;
  double adjacentLeg = opposedLeg/std::tan(std::asin(opposedLeg/hypotenuse));
  double overlap = adjacentLeg/2;

  return overlap;
}

Point OgreCity::getBeginingThatAvoidsOverlap(Intersection* intersection, Road* road, RoadParameters parameters)
{
  assert(intersection->hasRoad(road));

  double overlap = getRoadOverlapIntoIntersection(intersection, road, parameters);
  Ray extensionRay = getExtensionRay(intersection, road);

  return intersectionVertices[intersection][road][parameters.numberOfSampleVertices/2] + extensionRay.direction()*overlap*(-1);
}

void OgreCity::sortIntersectingRoadsCounterclockwise(Intersection* intersection, std::vector<Road*>* roadsOfCurrentIntersection)
{
  // Sort roads in counterclockwise manner relative to X axis (Vector(1,0))
  for (int i = 0; i < roadsOfCurrentIntersection->size(); i++)
  {
    for (int j = 0; j < roadsOfCurrentIntersection->size() - 1; j++)
    {
      Vector firstVector = getExtensionRay(intersection, roadsOfCurrentIntersection->at(j)).direction()*(-1),
             secondVector = getExtensionRay(intersection, roadsOfCurrentIntersection->at(j+1)).direction()*(-1);

      if (firstVector.angleToXAxis() > secondVector.angleToXAxis())
      {
         Road* temporary = roadsOfCurrentIntersection->at(j+1);
         roadsOfCurrentIntersection->at(j+1) = roadsOfCurrentIntersection->at(j);
         roadsOfCurrentIntersection->at(j) = temporary;
      }
    }
  }
}

void OgreCity::determineMiddleRoad(Intersection* intersection, std::vector<Road*>* roadsOfCurrentIntersection)
{
  /* Only for 3-way intersections */
  assert(roadsOfCurrentIntersection->size() == 3);

  // We have the roads sorted, but we need to decide which one will go first
  Vector firstDirection = getExtensionRay(intersection, roadsOfCurrentIntersection->at(0)).direction()*(-1),
         secondDirection = getExtensionRay(intersection, roadsOfCurrentIntersection->at(1)).direction()*(-1),
         thirdDirection = getExtensionRay(intersection, roadsOfCurrentIntersection->at(2)).direction()*(-1);

  double angle12 = firstDirection.angleTo(secondDirection),
         angle23 = secondDirection.angleTo(thirdDirection),
         angle31 = thirdDirection.angleTo(firstDirection);


  debug("Angles between roads are:");
  debug("  Angle(1,2): " << angle12 << ", angle to X " << firstDirection.angleToXAxis());
  debug("  Angle(2,3): " << angle23 << ", angle to X " << secondDirection.angleToXAxis());
  debug("  Angle(3,1): " << angle31 << ", angle to X " << thirdDirection.angleToXAxis());

  if (angle12 >= angle23 && angle12 > angle31)
  {
    debug("Biggest angle is between 1 and 2.");
    Road* temporary = roadsOfCurrentIntersection->at(0);
    roadsOfCurrentIntersection->at(0) = roadsOfCurrentIntersection->at(1);
    roadsOfCurrentIntersection->at(1) = roadsOfCurrentIntersection->at(2);
    roadsOfCurrentIntersection->at(2) = temporary;
  }
  else if (angle23 > angle12 && angle23 > angle31)
  {
    debug("Biggest angle is between 2 and 3.");
    Road* temporary = roadsOfCurrentIntersection->at(2);
    roadsOfCurrentIntersection->at(2) = roadsOfCurrentIntersection->at(1);
    roadsOfCurrentIntersection->at(1) = roadsOfCurrentIntersection->at(0);
    roadsOfCurrentIntersection->at(0) = temporary;
  }
  else
  {
    debug("Biggest angle is between 3 and 0.");
    /* Keep the default setting. */
  }
}

OgreCity::RoadParameters OgreCity::getRoadParameters(Road* road)
{
  if (road->type() == Road::SECONDARY_ROAD)
  {
    return secondaryRoad;
  }
  else
  {
    return primaryRoad;
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
