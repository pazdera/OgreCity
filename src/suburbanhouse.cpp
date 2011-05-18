/**
 * This code is part of OgreCity.
 *
 * @file suburbanhouse.cpp
 * @date 02.05.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see suburbanhouse.h
 *
 */

#include "suburbanhouse.h"

#include "ogrecity.h"
#include "libcity.h"

/* Ogre3D */
#include <OgreManualObject.h>

const Building::Type SuburbanHouse::SUBURBAN_HOUSE = Building::defineNewEntityType();

SuburbanHouse::SuburbanHouse(Lot* parentAlottment, Ogre::SceneManager* manager, Ogre::SceneNode* parentNode)
  : OgreBuilding(parentAlottment, manager, parentNode)
{
  initialize();
}

void SuburbanHouse::initialize()
{
  configure();
}

void SuburbanHouse::configure()
{
  setAxiom("{--BF+GH}");

  addRule('E', "FE");   // Next normal floor
  //addRule('E', "R-FE"); // Setbacks

  setInitialDirection(Vector(0,0,1));

  setupTextures();
}

void SuburbanHouse::setupTextures()
{
  Random generator;
  switch (generator.generateInteger(0,1))
  {
    case 0:
      windowTileMaterial = "SuburbanFloor";
      basementMaterial   = "SuburbanStreetLevel";
      spacerMaterial     = "BrickLedge";
      ledgeMaterial      = "BrickLedge";
      rooftopMaterial    = "WoodenRooftop";

      storeyHeight   = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight   = 1 * OgreCity::meter;
      ledgeHeight    = 1 * OgreCity::meter;
      tileWidth      = 10 * OgreCity::meter;
      rooftopHeight  = 4 * OgreCity::meter;
      break;
    case 1:
      windowTileMaterial = "SuburbanResidenceFloor";
      basementMaterial   = "SuburbanResidenceFloor";
      spacerMaterial     = "BrickLedge";
      ledgeMaterial      = "BrickLedge";
      rooftopMaterial    = "WoodenRooftop";

      storeyHeight   = 2.5 * OgreCity::meter;
      basementHeight = 4 * OgreCity::meter;
      spacerHeight   = 1 * OgreCity::meter;
      ledgeHeight    = 1 * OgreCity::meter;
      tileWidth      = 10 * OgreCity::meter;
      rooftopHeight  = 4 * OgreCity::meter;
      break;
    default:
      assert("Wrong configuration.");
  }
}

void SuburbanHouse::interpretSymbol(char symbol)
{
  switch (symbol)
  {
    default:
      /* Try to interpret symbols defined in parent. */
      OgreBuilding::interpretSymbol(symbol);
      break;
  }
}

void SuburbanHouse::finishDrawing()
{
}

SuburbanHouse::~SuburbanHouse()
{
  freeMemory();
}

void SuburbanHouse::freeMemory()
{}
