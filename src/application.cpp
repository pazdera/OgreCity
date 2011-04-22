/**
 * This code is part of OgreCity.
 *
 * @file application.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @see application.h
 *
 */

#include "application.h"
#include "ogrecity.h"


Application::Application(void)
  :  mInfoLabel(0)
{
  terrain = 0;
}

Application::~Application(void)
{
  if (terrain != 0) delete terrain;
}

void Application::destroyScene(void)
{
}

void Application::createScene(void)
{
  setupCamera();
  setupTextureFiltering();

  /* Setup scene lighting */
  Ogre::Vector3 lightDirection(0.55, -0.3, 0.75);
  lightDirection.normalise();

  Ogre::Light* light = mSceneMgr->createLight("sunLight");
  light->setType(Ogre::Light::LT_DIRECTIONAL);
  light->setDirection(lightDirection);
  light->setDiffuseColour(Ogre::ColourValue::White);
  light->setSpecularColour(Ogre::ColourValue(0.2, 0.2, 0.2));

  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.6, 0.6, 0.6));

  /* Draw terrain */
  terrain = new TerrainGenerator(mSceneMgr, light);
  terrain->draw();

//  Ogre::Material* terrainMat = terrain->getTerrainObject()->getMaterial().get();
//  terrainMat->setDepthWriteEnabled(false);

  /* Draw sky */
  mSceneMgr->setSkyDome(true, "Sky/CloudySky", 5, 8, 500);

  drawFog();

  /* Add a custom ogre :-) */
  Ogre::Entity* ogre = mSceneMgr->createEntity("Ogre", "ogrehead.mesh");
  Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  ogreNode->attachObject(ogre);
  ogreNode->setPosition(Ogre::Vector3(100, 50, 100));
//
//  Ogre::Entity* ogre2 = mSceneMgr->createEntity("Ogre2", "ogrehead.mesh");
//  Ogre::SceneNode* ogreNode2 = ogreNode->createChildSceneNode();
//  ogreNode2->attachObject(ogre2);
//  ogreNode2->translate(Ogre::Vector3(100, 0, 0));
//  ogreNode2->yaw(Ogre::Degree(-30));

  OgreCity* city = new OgreCity(mSceneMgr, terrain->getTerrainObject());
  city->generate();
  city->draw();

}

void Application::setupCamera()
{
  mCamera->setPosition(Ogre::Vector3(100, 100, 100));
  mCamera->lookAt(Ogre::Vector3(0, 0, 0));
  mCamera->setNearClipDistance(10);
  mCamera->setFarClipDistance(5000);

  if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
  {
    mCamera->setFarClipDistance(0);   // enable infinite far clip distance if we can
  }
}

void Application::setupTextureFiltering()
{
  // Note: Pressing T on runtime will discarde those settings
  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
  Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(7);
}

void Application::drawFog()
{
  //Ogre::ColourValue fadeColour(0.9, 0.9, 0.9);
  //mSceneMgr->setFog(Ogre::FOG_LINEAR, fadeColour, 0.0, 10, 1200);
  //mWindow->getViewport(0)->setBackgroundColour(fadeColour);

  //Ogre::Plane plane;
  //plane.d = 100;
  //plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
}


void Application::createFrameListener(void)
{
  BaseApplication::createFrameListener();

  mInfoLabel = mTrayMgr->createLabel(OgreBites::TL_TOP, "TInfo", "", 350);
}


bool Application::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
  bool ret = BaseApplication::frameRenderingQueued(evt);

  if (terrain->loadingInProgress())
  {
    mTrayMgr->moveWidgetToTray(mInfoLabel, OgreBites::TL_TOP, 0);
    mInfoLabel->show();

    mInfoLabel->setCaption(terrain->getStatus());
  }
  else
  {
    mTrayMgr->removeWidgetFromTray(mInfoLabel);
    mInfoLabel->hide();

    terrain->save();
  }

  return ret;
}

bool Application::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{


  return BaseApplication::mousePressed(arg, id);
}
