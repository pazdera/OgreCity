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
  addToAlphabet("C");

  setAxiom("{BSFER}");

  addRule('E', "FE");   // Next normal floor
  addRule('E', "SFE");  // Ledge, then floor
  addRule('E', "GCFE"); // Setbacks

  setInitialDirection(Vector(0,0,1));

  Random generator;
  setMaxHeight(generator.generateInteger(20, 30) * 2.5 * OgreCity::meter);

  setupTextures();
}

void SkyScraper::setupTextures()
{
  Random generator;
  switch (generator.generateInteger(0,2))
  {
    case 0:
      windowTileMaterial = "HotelWindow";
      basementMaterial   = "StreetLevel2";
      spacerMaterial     = "HotelLedge";
      ledgeMaterial      = "HotelLedge";
      rooftopMaterial    = "DirtyRooftop1";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 1:
      windowTileMaterial = "OfficeBuildingWindow";
      basementMaterial   = "StreetLevel7";
      spacerMaterial     = "OfficeBuildingLedge";
      ledgeMaterial      = "OfficeBuildingLedge";
      rooftopMaterial    = "DirtyRooftop1";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    case 2:
      windowTileMaterial = "GlassTowerWindow";
      basementMaterial   = "StreetLevel7";
      spacerMaterial     = "GlassTowerLedge";
      ledgeMaterial      = "GlassTowerLedge";
      rooftopMaterial    = "SkyScraperRooftop";

      storeyHeight = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight = 1 * OgreCity::meter;
      ledgeHeight = 1 * OgreCity::meter;
      tileWidth = 10 * OgreCity::meter;
      break;
    default:
      assert("Wrong configuration.");
  }
}

void SkyScraper::setback()
{
  if (cursor.getPosition().z() >= 0.6*maxHeight())
  {
    substractBoundingBox(5);
  }
}

void SkyScraper::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    case 'C': /* Setback */
      setback();
      break;
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
