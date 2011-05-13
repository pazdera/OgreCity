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

#include "environmentrenderer.h"
#include "terrainrenderer.h"


Application::Application(void)
  :  mInfoLabel(0)
{}

Application::~Application(void)
{
}

void Application::destroyScene(void)
{
}

void Application::createScene(void)
{
  setupCamera();
  setupTextureFiltering();

  /* Render environment */
  environment = new EnvironmentRenderer(mSceneMgr);
  environment->render();

  /* Render city */
  city = new OgreCity(mSceneMgr);
  city->setTerrain(environment->getTerrain());
  city->render();

  /* Add a custom ogre :-) */
//  Ogre::Entity* ogre = mSceneMgr->createEntity("Ogre", "ogrehead.mesh");
//  ogre->setCastShadows(true);
//  Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
//  ogreNode->attachObject(ogre);
//  ogreNode->setPosition(Ogre::Vector3(0, 300, 0));
//
//  Ogre::Entity* ogre2 = mSceneMgr->createEntity("Ogre2", "ogrehead.mesh");
//  Ogre::SceneNode* ogreNode2 = ogreNode->createChildSceneNode();
//  ogreNode2->attachObject(ogre2);
//  ogreNode2->translate(Ogre::Vector3(100, 0, 0));
//  ogreNode2->yaw(Ogre::Degree(-30));
}

void Application::setupCamera()
{
  mCamera->setPosition(Ogre::Vector3(100, 100, 500));
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



void Application::createFrameListener(void)
{
  BaseApplication::createFrameListener();

  mInfoLabel = mTrayMgr->createLabel(OgreBites::TL_TOP, "TInfo", "", 350);
}


bool Application::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
  bool ret = BaseApplication::frameRenderingQueued(evt);

  TerrainRenderer* terrain = environment->getTerrain();

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
