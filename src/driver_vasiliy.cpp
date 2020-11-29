#include "openvr_driver.h"
#include "driverlog.h"

#include <chrono>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include "ref/receiver_win.h"

#elif defined(__linux__)
#include "ref/receiver_linux.h"
#define _stricmp strcasecmp

#endif

#if defined(_WINDOWS)
#include <windows.h>
#endif



#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <variant>

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#define HMD_DLL_IMPORT extern "C" __declspec(dllimport)
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C"
#else
#error "Unsupported Platform."
#endif

namespace hobovr {
  // le version
  static const uint32_t k_nHobovrVersionMajor = 0;
  static const uint32_t k_nHobovrVersionMinor = 5;
  static const uint32_t k_nHobovrVersionBuild = 6;
  static const std::string k_sHobovrVersionGG = "phantom pain";

} // namespace hobovr

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y,
                                          double z) {
  HmdQuaternion_t quat;
  quat.w = w;
  quat.x = x;
  quat.y = y;
  quat.z = z;
  return quat;
}

// keys for use with the settings API
// driver keys
static const char *const k_pch_Hobovr_Section = "driver_hobovr";
static const char *const k_pch_Hobovr_UduDeviceManifestList_String = "DeviceManifestList";

// hmd device keys
static const char *const k_pch_Hmd_Section = "hobovr_device_hmd";
static const char *const k_pch_Hmd_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char *const k_pch_Hmd_DisplayFrequency_Float = "displayFrequency";
static const char* const k_pch_Hmd_IPD_Float = "IPD";
static const char* const k_pch_Hmd_UserHead2EyeDepthMeters_Float = "UserHeadToEyeDepthMeters";

// include has to be here, dont ask
#include "ref/hobovr_device_base.h"
#include "ref/hobovr_components.h"

#include <vector>
#include <ctime>
#include <time.h> // kruto
#include <chrono>
#include <cmath>

class OurDevice: public hobovr::HobovrDevice<false, false> {
public:
  OurDevice(std::string ser):HobovrDevice(ser, "asdfnjkl") {
    m_sRenderModelPath = "{hobovr}/rendermodels/hobovr_hmd_mh0"; 
    m_sBindPath = "{hobovr}/input/hobovr_hmd_profile.json";
  }

  EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) {
    HobovrDevice::Activate(unObjectId); // let the parent handle boilerplate stuff

    vr::VRProperties()->SetInt32Property(m_ulPropertyContainer,
                                         Prop_ControllerRoleHint_Int32,
                                         TrackedControllerRole_RightHand);

    return VRInitError_None;
  }
  
  void RunFrame(std::vector<float> &trackingPacket) {

    m_Pose.result = TrackingResult_Running_OK;
    m_Pose.vecPosition[0] = trackingPacket[0];
    m_Pose.vecPosition[1] = trackingPacket[1];
    m_Pose.vecPosition[2] = trackingPacket[2];
    
    m_Pose.qRotation = HmdQuaternion_Init(trackingPacket[3], trackingPacket[4],
                                          trackingPacket[5], trackingPacket[6]);
    
    if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
      vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
          m_unObjectId, m_Pose, sizeof(DriverPose_t));
    }

  }
};

//-----------------------------------------------------------------------------
// Purpose: serverDriver
//-----------------------------------------------------------------------------

class CServerDriver_hobovr : public IServerTrackedDeviceProvider {
public:
  CServerDriver_hobovr() {}
  virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext);
  virtual void Cleanup();
  virtual const char *const *GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
  }
  virtual bool ShouldBlockStandbyMode() { return false; }
  virtual void EnterStandby() {}
  virtual void LeaveStandby() {}
  virtual void RunFrame();
private:
  float thetaOver2 = 0.0;
  float theta = 0.0;
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float w = 0.0;
  std::shared_ptr<OurDevice> device;
};

// yes
EVRInitError CServerDriver_hobovr::Init(vr::IVRDriverContext *pDriverContext) {
  VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
  InitDriverLog(vr::VRDriverLog());

  DriverLog("driver: version: %d.%d.%d %s \n", hobovr::k_nHobovrVersionMajor,
                            hobovr::k_nHobovrVersionMinor,
                            hobovr::k_nHobovrVersionBuild,
                            hobovr::k_sHobovrVersionGG.c_str());
	
  device = std::make_shared<OurDevice>("glhf");
  vr::VRServerDriverHost()->TrackedDeviceAdded(
                    device->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker,
                    device.get());

  return VRInitError_None;
}

void CServerDriver_hobovr::Cleanup() {
  CleanupDriverLog();
  VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void CServerDriver_hobovr::RunFrame() {
  	
  	// do something with VR
    if (GetAsyncKeyState(0x25)) // Left arrow (strelochka vlevo) povorot vlevo
    {
      theta-=0.001;
    }
    
    if (GetAsyncKeyState(0x27)) // Right arrow (strelochka vpravo) povorot vpravo
    {
      theta+=0.001;
    }
    
    if (GetAsyncKeyState(0x53) != 0) // S nazad
    {       
      VR[2]+=0.10; 
    }
    
    if (GetAsyncKeyState(0x57)) // W vpered
    {
      VR[2]-=0.10; 
    }
    
    if (GetAsyncKeyState(0x41)) // A vlevo
    {
      VR[0]-=0.10;
    }
    
    if (GetAsyncKeyState(0x44)) // D vpravo
    {
      VR[0]+=0.10;
    }
    
    if (GetAsyncKeyState(0x51)) { // Q vverh
      VR[1]+=0.10;
    }
    
    if (GetAsyncKeyState(0x45)) { // E vniz
      VR[1]-=0.10;
    }
    
    thetaOver2 = theta * 0.5;
    x = 0.0;
    y = sin(thetaOver2);
    z = 0.0;
    w = np.cos(thetaOver2);

    VR[3] = w;
    VR[4] = x;
    VR[5] = y;
    VR[6] = z;

    my_device.RanFrame(VR);
  
  vr::VREvent_t vrEvent;
  while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
    device->ProcessEvent(vrEvent);
  }
}

CServerDriver_hobovr g_hobovrServerDriver;

//-----------------------------------------------------------------------------
// Purpose: driverFactory
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName,
                                      int *pReturnCode) {
  if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) {
    return &g_hobovrServerDriver;
  }

  if (pReturnCode)
    *pReturnCode = VRInitError_Init_InterfaceNotFound;

  return NULL;
}


/*
⢕⢕⢕⢕⠁⢜⠕⢁⣴⣿⡇⢓⢕⢵⢐⢕⢕⠕⢁⣾⢿⣧⠑⢕⢕⠄⢑⢕⠅⢕
⢕⢕⠵⢁⠔⢁⣤⣤⣶⣶⣶⡐⣕⢽⠐⢕⠕⣡⣾⣶⣶⣶⣤⡁⢓⢕⠄⢑⢅⢑
⠍⣧⠄⣶⣾⣿⣿⣿⣿⣿⣿⣷⣔⢕⢄⢡⣾⣿⣿⣿⣿⣿⣿⣿⣦⡑⢕⢤⠱⢐
⢠⢕⠅⣾⣿⠋⢿⣿⣿⣿⠉⣿⣿⣷⣦⣶⣽⣿⣿⠈⣿⣿⣿⣿⠏⢹⣷⣷⡅⢐
⣔⢕⢥⢻⣿⡀⠈⠛⠛⠁⢠⣿⣿⣿⣿⣿⣿⣿⣿⡀⠈⠛⠛⠁⠄⣼⣿⣿⡇⢔
⢕⢕⢽⢸⢟⢟⢖⢖⢤⣶⡟⢻⣿⡿⠻⣿⣿⡟⢀⣿⣦⢤⢤⢔⢞⢿⢿⣿⠁⢕
⢕⢕⠅⣐⢕⢕⢕⢕⢕⣿⣿⡄⠛⢀⣦⠈⠛⢁⣼⣿⢗⢕⢕⢕⢕⢕⢕⡏⣘⢕
⢕⢕⠅⢓⣕⣕⣕⣕⣵⣿⣿⣿⣾⣿⣿⣿⣿⣿⣿⣿⣷⣕⢕⢕⢕⢕⡵⢀⢕⢕
⢑⢕⠃⡈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢃⢕⢕⢕
⣆⢕⠄⢱⣄⠛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⢁⢕⢕⠕⢁
*/
