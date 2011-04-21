/*
*/

#ifndef _CITY3D_H_
#define _CITY3D_H_

#include "baseapplication.h"

#include <libcity.h>
#include "terraingenerator.h"

class City3D : public BaseApplication
{
  private:
    OgreBites::Label* mInfoLabel;

    TerrainGenerator* terrain;

    void setupCamera();
    void setupTextureFiltering();
    void drawFog();

  public:
    City3D(void);
    virtual ~City3D(void);

  protected:
    virtual void createScene(void);
    virtual void destroyScene(void);
    virtual void createFrameListener(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
};

#endif
