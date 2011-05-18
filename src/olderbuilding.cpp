/**
 * This code is part of OgreCity.
 *
 * @file olderbuilding.cpp
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see olderbuilding.h
 *
 */

#include "olderbuilding.h"

#include "ogrecity.h"
#include "libcity.h"

/* Ogre3D */
#include <OgreManualObject.h>

const Building::Type OlderBuilding::OLDER_BUILDING = Building::defineNewEntityType();

OlderBuilding::OlderBuilding(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode)
  : OgreBuilding(parentAlottment, manager, parentNode)
{
  initialize();
}

void OlderBuilding::initialize()
{
  configure();
}

void OlderBuilding::configure()
{
  setAxiom("{BOFER}");

  addRule('E', "FE");   // Next normal floor
  addRule('E', "FOFE");  // Ledge, then floor
  //addRule('E', "R-FE"); // Setbacks

  setInitialDirection(Vector(0,0,1));


  Random generator;
  setMaxHeight(generator.generateInteger(3, 7) * 2.5 * OgreCity::meter);

  setupTextures();
}

void OlderBuilding::setupTextures()
{
  Random generator;
  switch (generator.generateInteger(0,2))
  {
    case 0:
      windowTileMaterial = "BrickWindow";
      basementMaterial   = "StreetLevel1";
      spacerMaterial     = "BrickLedge";
      ledgeMaterial      = "BrickLedge";
      rooftopMaterial    = "RedRooftop";

      storeyHeight   = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight   = 1 * OgreCity::meter;
      ledgeHeight    = 1 * OgreCity::meter;
      tileWidth      = 10 * OgreCity::meter;
      rooftopHeight  = 4 * OgreCity::meter;
      break;
    case 1:
      windowTileMaterial = "HistoricWindow";
      basementMaterial   = "StreetLevel3";
      spacerMaterial     = "HistoricLedge";
      ledgeMaterial      = "HistoricLedge";
      rooftopMaterial    = "RedRooftop";

      storeyHeight = 5 * OgreCity::meter;
      basementHeight = 5 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight    = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      rooftopHeight  = 4 * OgreCity::meter;
      break;
    case 2:
      windowTileMaterial = "Brick3Window";
      basementMaterial   = "StreetLevel6";
      spacerMaterial     = "Brick3Ledge";
      ledgeMaterial      = "Brick3Ledge";
      rooftopMaterial    = "RedRooftop";

      storeyHeight = 5 * OgreCity::meter;
      basementHeight = 4.5 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight    = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      rooftopHeight  = 4 * OgreCity::meter;
      break;
    default:
      assert("Wrong configuration.");
  }
}

void OlderBuilding::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    default:
      /* Try to interpret symbols defined in parent. */
      OgreBuilding::interpretSymbol(symbol);
      break;
  }
}

void OlderBuilding::finishDrawing()
{
  drawObliqueLedge();
  addBoundingBox(2);
  drawHoweRooftop();
}

OlderBuilding::~OlderBuilding()
{
  freeMemory();
}

void OlderBuilding::freeMemory()
{}
