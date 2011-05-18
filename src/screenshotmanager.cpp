
#include "screenshotmanager.h"
#include "baseapplication.h"

using namespace Ogre;

ScreenshotManager::ScreenshotManager(Ogre::RenderWindow* pRenderWindow, int gridSize, Ogre::String fileExtension, bool overlayFlag)
{
    //set file extension for the Screenshot files
    mFileExtension = fileExtension;
    // the gridsize
    mGridSize         = gridSize;
    // flag for overlay rendering
    mDisableOverlays  = overlayFlag;
    //get current window size
    mWindowWidth     = pRenderWindow->getWidth();
    mWindowHeight     = pRenderWindow->getHeight();
    //create temporary texture
    mTempTex    = TextureManager::getSingleton().createManual("ScreenShotTex",
                                                               ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D,
                                                                 mWindowWidth, mWindowHeight,0, PF_B8G8R8, TU_RENDERTARGET);

    //get The current Render Target of the temp Texture
    mRT = mTempTex->getBuffer()->getRenderTarget();

    //HardwarePixelBufferSharedPtr to the Buffer of the temp Texture
    mBuffer = mTempTex->getBuffer();

    //create PixelBox
        uint8* data = new uint8[(mWindowWidth * mGridSize) * (mWindowHeight * mGridSize) * 3];
    mFinalPicturePB = PixelBox(mWindowWidth * mGridSize,mWindowHeight * mGridSize,1,PF_B8G8R8,data);

}

ScreenshotManager::~ScreenshotManager()
{
   //delete[] data;
}

/* Creates a screenshot with the given camera.
* @param camera Pointer to the camera "looking at" the scene of interest
* @param fileName the filename of the screenshot file.
*/
void ScreenshotManager::makeScreenshot(Ogre::Camera* camera, Ogre::String fileName) const
{

    //Remove all viewports, so the added Viewport(camera) ist the only
    mRT->removeAllViewports();
    mRT->addViewport(camera);

    //set the viewport settings
    Viewport *vp = mRT->getViewport(0);
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);

    // remind current overlay flag
    bool enableOverlayFlag = camera->getViewport()->getOverlaysEnabled();

    // we disable overlay rendering if it is set in config file and the viewport setting is enabled
    if(mDisableOverlays && enableOverlayFlag)
        camera->getViewport()->setOverlaysEnabled(false);

    if(mGridSize <= 1)
    {
        // Simple case where the contents of the screen are taken directly
        // Also used when an invalid value is passed within gridSize (zero or negative grid size)
        mRT->update();        //render

        //write the file on the Harddisk
        mRT->writeContentsToTimestampedFile(fileName, mFileExtension);
    }
    else
    {
        //define the original frustum extents variables
        Real originalFrustumLeft, originalFrustumRight, originalFrustumTop, originalFrustumBottom;
        // set the original Frustum extents
        camera->getFrustumExtents(originalFrustumLeft, originalFrustumRight, originalFrustumTop, originalFrustumBottom);

        // compute the Stepsize for the drid
        Real frustumGridStepHorizontal    = (originalFrustumRight * 2) / mGridSize;
        Real frustumGridStepVertical    = (originalFrustumTop * 2) / mGridSize;

        // process each grid
        Real frustumLeft, frustumRight, frustumTop, frustumBottom;
        for (unsigned int nbScreenshots = 0; nbScreenshots < mGridSize * mGridSize; nbScreenshots++)
        {
            int y = nbScreenshots / mGridSize;
            int x = nbScreenshots - y * mGridSize;

            // Shoggoth frustum extents setting
            // compute the new frustum extents
            frustumLeft        = originalFrustumLeft + frustumGridStepHorizontal * x;
            frustumRight    = frustumLeft + frustumGridStepHorizontal;
            frustumTop        = originalFrustumTop - frustumGridStepVertical * y;
            frustumBottom    = frustumTop - frustumGridStepVertical;

            // set the frustum extents value to the camera
            camera->setFrustumExtents(frustumLeft, frustumRight, frustumTop, frustumBottom);

            // ignore time duration between frames
            Ogre::Root::getSingleton().clearEventTimes();
            mRT->update();        //render

            //define the current
            Box subBox = Box(x* mWindowWidth,y * mWindowHeight,x * mWindowWidth + mWindowWidth, y * mWindowHeight + mWindowHeight);
            //copy the content from the temp buffer into the final picture PixelBox
            //Place the tempBuffer content at the right position
            mBuffer->blitToMemory(mFinalPicturePB.getSubVolume(subBox));

        }

        // set frustum extents to previous settings
        camera->resetFrustumExtents();

        Image finalImage; //declare the final Image Object
        //insert the PixelBox data into the Image Object
        finalImage = finalImage.loadDynamicImage(static_cast<unsigned char*>(mFinalPicturePB.data), mFinalPicturePB.getWidth(),mFinalPicturePB.getHeight(),PF_B8G8R8);
        // Save the Final image to a file
        finalImage.save(fileName + "." + mFileExtension);

    }

    // do we have to re-enable our overlays?
    if(enableOverlayFlag)
        camera->getViewport()->setOverlaysEnabled(true);

    // reset time since last frame to pause the scene
    Ogre::Root::getSingleton().clearEventTimes();
}
