/**
 * This code is part of OgreCity.
 *
 * @file redbuilding.cpp
 * @date 10.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see redbuilding.h
 *
 */

#include "redbuilding.h"

#include "ogrecity.h"
#include "libcity.h"

/* Ogre3D */
#include <OgreManualObject.h>

const Building::Type RedBuilding::RED_BUILDING = Building::defineNewEntityType();

RedBuilding::RedBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode)
  : OgreBuilding(parentAlottment, manager, parentNode)
{
  initialize();
}

void RedBuilding::initialize()
{
  configure();
}

void RedBuilding::configure()
{
  setAxiom("{BSFER}");

  //addRule('E', "FE");   // Next normal floor
  addRule('E', "LFE");  // Ledge, then floor
  //addRule('E', "R-FE"); // Setbacks

  setInitialDirection(Vector(0,0,1));

  Random generator;
  setMaxHeight(generator.generateInteger(20, 30) * 2.5 * OgreCity::meter);

  setupTextures();
}

void RedBuilding::setupTextures()
{
  Random generator;
  switch (generator.generateInteger(0,3))
  {
    case 0:
      windowTileMaterial = "Modern3Window";
      basementMaterial   = "StreetLevel3";
      spacerMaterial     = "Modern3Ledge";
      ledgeMaterial      = "Modern3Ledge";
      rooftopMaterial    = "OldRooftop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 5 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      rooftopHeight = 4 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 1:
      windowTileMaterial = "ModernWindow2";
      basementMaterial   = "StreetLevel10";
      spacerMaterial     = "ModernLedge2";
      ledgeMaterial      = "ModernLedge2";
      rooftopMaterial    = "WhiteMetalRooftop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 5 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      rooftopHeight = 4 * OgreCity::meter;
      tileWidth = 8 * OgreCity::meter;
      break;
    case 2:
      windowTileMaterial = "ModernWindow";
      basementMaterial   = "StreetLevel5";
      spacerMaterial     = "ModernLedge";
      ledgeMaterial      = "ModernLedge";
      rooftopMaterial    = "WhiteMetalRooftop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 4.8 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 3:
      windowTileMaterial = "Brick2Window";
      basementMaterial   = "StreetLevel5";
      spacerMaterial     = "Brick2Ledge";
      ledgeMaterial      = "Brick2Ledge";
      rooftopMaterial    = "OldRooftop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 4.8 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    default:
      assert("Wrong configuration.");
  }
}

void RedBuilding::finishDrawing()
{
  drawStorey();
  addBoundingBox(2);
  drawFloor();
  drawSpacer();
  drawRooftop();
}

void RedBuilding::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    default:
      /* Try to interpret symbols defined in parent. */
      OgreBuilding::interpretSymbol(symbol);
      break;
  }
}

RedBuilding::~RedBuilding()
{
  freeMemory();
}

void RedBuilding::freeMemory()
{}
