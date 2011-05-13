/**
 * This code is part of OgreCity.
 *
 * @file skyscraper.cpp
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see skyscraper.h
 *
 */

#include "skyscraper.h"

#include "ogrecity.h"
#include "libcity.h"

/* Ogre3D */
#include <OgreManualObject.h>

const Building::Type SkyScraper::SKY_SCRAPER = Building::defineNewEntityType();

SkyScraper::SkyScraper(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode)
  : OgreBuilding(parentAlottment, manager, parentNode)
{
  initialize();
}

void SkyScraper::initialize()
{
  configure();
}

void SkyScraper::configure()
{
  setAxiom("{BFER}");

  addRule('E', "FE");   // Next normal floor
  addRule('E', "SFE");  // Ledge, then floor
  addRule('E', "R-FE"); // Setbacks

  setInitialDirection(Vector(0,0,1));

  Random generator;
  setMaxHeight(generator.generateInteger(20, 30) * 2.5 * OgreCity::meter);

  setupTextures();
}

void SkyScraper::setupTextures()
{
  Random generator;
  switch (generator.generateInteger(0,3))
  {
    case 0:
      windowTileMaterial = "BrickWindow";
      basementMaterial   = "HotelWindow";
      spacerMaterial     = "BrickLedge";
      rooftopMaterial    = "RoofTop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 3 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 1:
      windowTileMaterial = "HotelWindow";
      basementMaterial   = "HotelWindow";
      spacerMaterial     = "HotelLedge";
      rooftopMaterial    = "RoofTop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 3 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 2:
      windowTileMaterial = "OfficeBuildingWindow";
      basementMaterial   = "HotelWindow";
      spacerMaterial     = "HotelLedge";
      rooftopMaterial    = "RoofTop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 3 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 3:
      windowTileMaterial = "HistoricWindow";
      basementMaterial   = "HotelWindow";
      spacerMaterial     = "HotelLedge";
      rooftopMaterial    = "RoofTop";

      storeyHeight = 5 * OgreCity::meter;
      basementHeight = 3 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    default:
      assert("Wrong configuration.");
  }
}

void SkyScraper::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    default:
      /* Try to interpret symbols defined in parent. */
      OgreBuilding::interpretSymbol(symbol);
      break;
  }
}

SkyScraper::~SkyScraper()
{
  freeMemory();
}

void SkyScraper::freeMemory()
{}
