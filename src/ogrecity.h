/**
 * This code is part of libcity demonstration application.
 *
 * @file ogrecity.h
 * @date 11.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @brief Implement abstract class City from libcity and build some city.
 *
 */

#ifndef OGRECITY_H
#define OGRECITY_H

#include <libcity.h>

#include <Ogre.h>
#include <Terrain/OgreTerrain.h>

class OgreCity : public City
{
  public:
    OgreCity(Ogre::SceneManager* sceneManagerObject, Ogre::Terrain* terrainObject);
    virtual ~OgreCity();

  protected:
    virtual void createPrimaryRoadNetwork();
    virtual void createZones();
    virtual void createSecondaryRoadNetwork();
    virtual void createBlocks();
    virtual void createBuildings();

    virtual void drawRoadNetwork();
    virtual void drawBuildings();

    /**
      Inline method that is called in the constructor. This
      is the only place for any parameter initialization and
      configuration.
      */
    virtual void configuration()
    {
      /* Primary roads configuration */
      primaryRoad.height = 1;
      primaryRoad.sampleSize = 1;
      primaryRoad.width = 30;

      primaryRoad.material = "DrivingSim/RoadMat";

      primaryRoad.numberOfSampleVertices = 5;

      // Texture mapping on vertices: 0, 0.03125, 0.5, 1-0.03125, 1
      primaryRoad.verticesTextureMapping.push_back(0);
      primaryRoad.verticesTextureMapping.push_back(0.03125);
      primaryRoad.verticesTextureMapping.push_back(0.5);
      primaryRoad.verticesTextureMapping.push_back(1-0.03125);
      primaryRoad.verticesTextureMapping.push_back(1);

      // Space offset of vertices: 1.1, 1, 0, -1, -1.1
      primaryRoad.verticesPositionOffset.push_back(1.0);
      primaryRoad.verticesPositionOffset.push_back(0.9);
      primaryRoad.verticesPositionOffset.push_back(0);
      primaryRoad.verticesPositionOffset.push_back(-0.9);
      primaryRoad.verticesPositionOffset.push_back(-1.0);

      /* Secondary roads configuration */
      secondaryRoad.height = 1;
      secondaryRoad.sampleSize = 1;
      secondaryRoad.width = 15;

      secondaryRoad.material = "DrivingSim/RoadMat";

      secondaryRoad.numberOfSampleVertices = 5;
      secondaryRoad.verticesTextureMapping = primaryRoad.verticesTextureMapping; // Same as primary road
      secondaryRoad.verticesPositionOffset = primaryRoad.verticesPositionOffset;

      /* Building configuration */
    }

    /**
      Data structure to keep all parameters for different road types.
      Each City object should create couple of instances of this class
      to pass to the rendering method.
      */
    class RoadParameters
    {
      public:
        double width;
        double height;

        double sampleSize;

        Ogre::String material;

        int numberOfSampleVertices;
        std::vector<double> verticesTextureMapping;
        std::vector<double> verticesPositionOffset;
    };

  private:
    Ogre::SceneManager* sceneManager;
    Ogre::Terrain* terrain;

    RoadParameters primaryRoad, secondaryRoad;

    /* For storing precomputed information about intersections */
    typedef std::map<Road*, Point*> RoadEndVertices;
    typedef std::map<Intersection*, RoadEndVertices> IntersectionVertices;
    IntersectionVertices intersectionVertices;

    /**
      Precompute vertices of all intersections in StreetGraph.
     @remarks
      This step is very important so the intersecting roads
      lock into each other perfectly. Otherwise the result
      is just total mess.
     */
    void precomputeIntersectionsVertices();

    /**
      Project 2D path onto terrain (@see OgreCity::terrain).
     @note
       Current imlpementation does nothing more, but transformation
       of Path object to Ogre::SimpleSpline.
     @todo
       Improve the projection process. Maybe add some sort of
       heuristic to determine more realistic path by taking
       the path of least elevation.

     @param[in] path Path to be mapped.
     @return Projected path.
    */
    Ogre::SimpleSpline mapPathToTerrain(Path* path);


    /**
      Draw a road on terrain with specified path.
     @remarks
      Function generates a mesh using Ogre::ManualMesh. Then
      creates an Entity, SceneNode and inserts the road
      into the scene.
     @todo
      Make implementation more versatile and configurable. Texture,
      road width, sampling size should be configurable. Also modify
      return value (or remove it completely).

     @param[in] road           Road to be rendered.
     @param[in] roadParameters Parameters of the road (@see RoadParameters).
     @param[in] terrain        Terrain on which the road will reside on (for adjusting height of the road).
     */
    void drawRoad(Road* road, RoadParameters parameters, Ogre::Terrain* terrain);

    /**
      Experimental function that attempts to locate terrain
      vertices under the road network and flatten them to
      avoid z-fighting and object collisions.
     @note
       Doesn't provide very satisfying results.
    */
    void flattenTerrainUnderRoads();

    /**
      Get new road begining so the road doesn't overlap into the intersection.
     @warning
       Uses precomputed intersection data! The bounding vertices must be
       available when this method is called.
     @param[in] intersection Intersection we're checking.
     @param[in] road Road we're checking. The road must come to the
                specified intersection.
     @param[in] parameters Rendering parameters of the current road.
     @return New road begining.
     */
    Point getBeginingThatAvoidsOverlap(Intersection* intersection, Road* road, RoadParameters parameters);

    /**
      Compute how much the road overlaps into the current intersection.
     @warning
       Uses precomputed intersection data! The bounding vertices must be
       available when this method is called.
     @param[in] intersection Intersection we're checking.
     @param[in] road Road we're checking. The road must come to the
                specified intersection.
     @param[in] parameters Rendering parameters of the current road.
     @return Length of the road overlap.
     */
    double getRoadOverlapIntoIntersection(Intersection* intersection, Road* road, RoadParameters parameters);

    static Ogre::Vector3 libcityToOgre(Point const& point);
    static Ogre::Vector3 libcityToOgre(Vector const& vector);

    Vector ogreToLibcity(Ogre::Vector3 const& vector);

    OgreCity::RoadParameters getRoadParameters(Road* road);

    /**
      Sort roads of one intersection counter clock-wise of Vector(1,0).
     @todo
       Replace bubble sort with something more effective.
     @param[in] intersection At what intersection roads meet. This is important to
                determine road direction vectors orientation.
     @param[in,out] roadsOfCurrentIntersection Roads we want to sort. All of them
                    must start or end at the specified intersection.
      */
    void sortIntersectingRoadsCounterclockwise(Intersection* intersection, std::vector<Road*>* roadsOfCurrentIntersection);

    /**
      Sort roads of one intersection counter clock-wise of Vector(1,0).
     @todo
       Replace bubble sort with something more effective.
     @param[in] intersection At what intersection roads meet. This is important to
                determine road direction vectors orientation.
     @param[in,out] roadsOfCurrentIntersection Roads we want to shuffle sorted
                    counterclockwise. The middle road will be set into the middle of
                    the array.
      */
    void determineMiddleRoad(Intersection* intersection, std::vector<Road*>* roadsOfCurrentIntersection);

    Ray getExtensionRay(Intersection* intersection, Road* road);
};

#endif // OGRECITY_H
