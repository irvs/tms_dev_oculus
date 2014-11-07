
/// Copyright (C) 2013 Kojack
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.

#include "OVR_Kernel.h"
#include "OVR_CAPI.h"
#include "OVR.h"
#include "oculus_rviz_plugins/ogre_oculus.h"
#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"

#include <fstream>

using namespace OVR;

namespace
{
const float g_defaultNearClip = 0.01f;
const float g_defaultFarClip = 10000.0f;
const float g_defaultIPD = 0.064f;
const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
const float g_defaultProjectionCentreOffset = 0.14529906f;
const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0.0f};
const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
}

namespace oculus_rviz_plugins
{

Oculus::Oculus(void) :
    m_oculusReady(false), m_ogreReady(false), m_centreOffset(g_defaultProjectionCentreOffset), m_window(0), m_sceneManager(0), m_cameraNode(0)
{
  for (int i = 0; i < 2; ++i)
  {
    m_cameras[i] = 0;
    m_viewports[i] = 0;
    m_compositors[i] = 0;
  }
}

Oculus::~Oculus(void)
{
  shutDownOgre();
  shutDownOculus();
}

void Oculus::shutDownOculus()
{
  ovrHmd_Destroy(m_hmd);
  ovr_Shutdown();

  if ( m_oculusReady)
  {

  }

  m_oculusReady = false;
}

void Oculus::shutDownOgre()
{
  m_ogreReady = false;
  for (int i = 0; i < 2; ++i)
  {
    if (m_compositors[i])
    {
      Ogre::CompositorManager::getSingleton().removeCompositor(m_viewports[i], "Oculus");
      m_compositors[i] = 0;
    }
    if (m_viewports[i])
    {
      m_window->removeViewport(i);
      m_viewports[i] = 0;
    }
    if (m_cameras[i])
    {
      m_cameras[i]->getParentSceneNode()->detachObject(m_cameras[i]);
      m_sceneManager->destroyCamera(m_cameras[i]);
      m_cameras[i] = 0;
    }
  }
  if (m_cameraNode)
  {
    m_cameraNode->getParentSceneNode()->removeChild(m_cameraNode);
    m_sceneManager->destroySceneNode(m_cameraNode);
    m_cameraNode = 0;
  }
  m_window = 0;
  m_sceneManager = 0;
}

bool Oculus::isOculusReady() const
{
  return m_oculusReady;
}

bool Oculus::isOgreReady() const
{
  return m_ogreReady;
}

bool Oculus::setupOculus()
{
	if (m_oculusReady)
	{
		Ogre::LogManager::getSingleton().logMessage("Oculus: Already Initialised");
		return true;
	}

	// Initializes LibOVR. 
	Ogre::LogManager::getSingleton().logMessage("Oculus: Initialising system");
	ovr_Initialize();

	m_hmd = ovrHmd_Create(0);
	if (!m_hmd)
	{
       		// Oculus Rift nof found
        	m_hmd = ovrHmd_CreateDebug(ovrHmd_DK2);
		if (!m_hmd)
		{
			Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus Rift not detected.");
			return(1);
	    }
	}

	//Get more details about the m_hmd	
	if (m_hmd->DisplayDeviceName[0] == '\0')
		Ogre::LogManager::getSingleton().logMessage("Oculus: Rift detected, display not enabled.");

	ovrFovPort eyeFov[2] = { m_hmd->DefaultEyeFov[0], m_hmd->DefaultEyeFov[1] };
	EyeRenderDesc[0] = ovrHmd_GetRenderDesc(m_hmd, ovrEye_Left, eyeFov[0]);
	EyeRenderDesc[1] = ovrHmd_GetRenderDesc(m_hmd, ovrEye_Right, eyeFov[1]);

	ovrHmd_SetEnabledCaps(m_hmd, ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction);
	ovrHmd_ConfigureTracking(m_hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);

	m_oculusReady = true;
	Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");
	return true;
}

bool Oculus::setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent)
{
  m_window = win;
  m_sceneManager = sm;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Setting up Ogre");
  if (parent)
    m_cameraNode = parent->createChildSceneNode("StereoCameraNode");
  else
    m_cameraNode = sm->getRootSceneNode()->createChildSceneNode("StereoCameraNode");

  m_cameras[0] = sm->createCamera("CameraLeft");
  m_cameras[1] = sm->createCamera("CameraRight");

  Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
  Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Oculus/Right");
  Ogre::GpuProgramParametersSharedPtr pParamsLeft =
      matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::GpuProgramParametersSharedPtr pParamsRight =
      matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::Vector4 hmdwarp;

  hmdwarp = Ogre::Vector4(g_defaultDistortion[0], g_defaultDistortion[1], g_defaultDistortion[2],
                            g_defaultDistortion[3]);

  pParamsLeft->setNamedConstant("HmdWarpParam", hmdwarp);
  pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);

  Ogre::Vector4 hmdchrom;

  hmdchrom = Ogre::Vector4(g_defaultChromAb);

  pParamsLeft->setNamedConstant("ChromAbParam", hmdchrom);
  pParamsRight->setNamedConstant("ChromAbParam", hmdchrom);
  
  pParamsLeft->setNamedConstant("LensCenter", 0.5f + (m_centreOffset / 2.0f));
  pParamsRight->setNamedConstant("LensCenter", 0.5f - (m_centreOffset / 2.0f));

  Ogre::CompositorPtr comp = Ogre::CompositorManager::getSingleton().getByName("OculusRight");
  comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName("Ogre/Compositor/Oculus/Right");

  for (int i = 0; i < 2; ++i)
  {
    m_cameraNode->attachObject(m_cameras[i]);
    m_cameras[i]->setNearClipDistance(g_defaultNearClip);
    m_cameras[i]->setFarClipDistance(g_defaultFarClip);
    m_cameras[i]->setPosition((i * 2 - 1) * g_defaultIPD * 0.5f, 0, 0);
    m_viewports[i] = win->addViewport(m_cameras[i], i, 0.5f * i, 0, 0.5f, 1.0f);
    m_viewports[i]->setBackgroundColour(g_defaultViewportColour);
    m_compositors[i] = Ogre::CompositorManager::getSingleton().addCompositor(m_viewports[i],
                                                                             i == 0 ? "OculusLeft" : "OculusRight");
    m_compositors[i]->setEnabled(true);
  }

  updateProjectionMatrices();

  m_ogreReady = true;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");
  return true;
}

void Oculus::updateProjectionMatrices()
{
  for (int i = 0; i < 2; ++i)
  {
    m_cameras[i]->setCustomProjectionMatrix(false);
    Ogre::Matrix4 proj = Ogre::Matrix4::IDENTITY;
	proj.setTrans(Ogre::Vector3(-m_centreOffset * (2 * i - 1), 0, 0));
    m_cameras[i]->setCustomProjectionMatrix(true, proj * m_cameras[i]->getProjectionMatrix());
  }
}

void Oculus::update()
{
  if (m_ogreReady)
  {
    m_cameraNode->setOrientation(getOrientation(0));
  }
}

Ogre::SceneNode* Oculus::getCameraNode()
{
  return m_cameraNode;
}

Ogre::Quaternion Oculus::getOrientation(int eyeIndex) const
{
  if (m_oculusReady && eyeIndex<ovrEye_Count)
  {
    ovrTrackingState ts = ovrHmd_GetTrackingState(m_hmd, ovr_GetTimeInSeconds());
    if ( ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked ))
    {
       ovrPosef movePose = ts.HeadPose.ThePose;
       ovrQuatf q = movePose.Orientation;
       return Ogre::Quaternion(q.w, q.x, q.y, q.z);
    }
  }
  else
  {
    return Ogre::Quaternion::IDENTITY;
  }
}

Ogre::CompositorInstance *Oculus::getCompositor(unsigned int i)
{
  return m_compositors[i];
}

float Oculus::getCentreOffset() const
{
  return m_centreOffset;
}

}

