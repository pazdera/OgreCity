/**
 * This code is part of OgreCity.
 *
 * @file application.h
 * @date 22.04.2011
 * @author Radek Pazdera (xpazde00@stud.fit.vutbr.cz)
 *
 * @brief Application interface class.
 *
 * The class is based on the Ogre's 3D tutorial framework. The
 * original one can be found on the OgreWiki: http://www.ogre3d.org/tikiwiki/Home
 *
 * @mainpage OgreCity documentation
 *   Welcome to OgreCity documentation. Hopefully you'll find here
 *   exactly what you need!
 *
 *   OgreCity is a demonstration application that presents capabilities
 *   of libcity library for procedural generation of cities.
 *
 * @section license License
@verbatim
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
 *
 */


#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "baseapplication.h"

#include <libcity.h>

class EnvironmentRenderer;
class OgreCity;


class Application : public BaseApplication
{
  public:
    Application(void);
    virtual ~Application(void);

  protected:
    virtual void createScene(void);
    virtual void destroyScene(void);
    virtual void createFrameListener(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);

  private:
    OgreBites::Label* mInfoLabel;

    EnvironmentRenderer* environment;
    OgreCity* city;

    void setupCamera();
    void setupTextureFiltering();
};

#endif
