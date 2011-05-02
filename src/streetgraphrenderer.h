/**
 * This code is part of OgreCity.
 *
 * @file streetgraphrenderer.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * This renderer is responsible for drawing streets and roads
 * from given StreetGraph.
 *
 */

#ifndef _STREETGRAPHRENDERER_H_
#define _STREETGRAPHRENDERER_H_

#include "renderer.h"

/* STL */
#include <map>

/* Ogre */
#include <Terrain/OgreTerrain.h>

/* libcity */
#include <libcity.h>

class StreetGraphRenderer : public Renderer
{
  private:
    StreetGraphRenderer();

  public:
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

        Ogre::String material;

        std::vector<double> verticesTextureMapping;
        std::vector<double> verticesPositionOffset;
    };

    StreetGraphRenderer(Ogre::SceneManager* manager);

    void setStreetGraph(StreetGraph* newStreetGraph);
    void setTerrain(Ogre::Terrain* newTerrain);
    void setRoadSampleLength(double length);
    void setNumberOfVerticesPerSample(unsigned short vertices);
    void setRoadParameters(Road::Type type, RoadParameters parameters);

    void render();

  protected:
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

    StreetGraphRenderer::RoadParameters getRoadParameters(Road* road);

    Ogre::String getUniqueObjectNumber();

    Ogre::Vector3 mapPointToTerrain(Ogre::Vector3 point);
    Ogre::Vector3 mapPointToTerrain(Point point);
    int getVertexIndex(int index, bool inverted);

    StreetGraph* streetGraph;
    Ogre::Terrain* terrain;

    double sampleSize;                                    /**< Length of a single road sample. */
    int numberOfSampleVertices;                           /**< Number of vertices for each sample. */
    std::map<Road::Type, RoadParameters> roadParameters; /**< Different parameters for different road types. */

    /* For storing precomputed information about intersections */
    typedef std::map<Road*, Point*> RoadEndVertices;
    typedef std::map<Intersection*, RoadEndVertices> IntersectionVertices;
    IntersectionVertices intersectionVertices;

};

#endif // _STREETGRAPHRENDERER_H_
