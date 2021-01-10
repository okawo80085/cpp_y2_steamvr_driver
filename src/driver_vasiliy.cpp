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

#include "ref/Quaternion.hpp"
#include "ref/Vector3.hpp"

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
static const char *const k_pch_Hobovr_Section = "driver_vasiliy";
static const char *const k_pch_Hobovr_UduDeviceManifestList_String = "DeviceManifestList";

// hmd device keys
static const char *const k_pch_Hmd_Section = "vasiliy_device_hmd";
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
#include <string>

class OurDevice: public hobovr::HobovrDevice<false, false> {
public:
  OurDevice(std::string ser):HobovrDevice(ser, "asdfnjkl") {
    m_sRenderModelPath = "{vasiliy}/rendermodels/hobovr_tracker_mt0"; 
    m_sBindPath = "{vasiliy}/input/hobovr_tracker_profile.json";

    // auto val = vr::VRSettings()->GetFloat("vasiliy_device_hmd", "UserHeadToEyeDepthMeters");
    // DriverLog("our val: %f", val);
    // auto val2 = vr::VRSettings()->GetInt32("vasiliy_comp_extendedDisplay", "windowWidth");
    // DriverLog("our val: %d", val2);
    // auto val3 = vr::VRSettings()->GetBool("vasiliy_comp_extendedDisplay", "IsDisplayOnDesktop");
    // DriverLog("our val: %d", val3);
    // char buff[1024];
    // vr::VRSettings()->GetString("driver_vasiliy", "blah", buff, sizeof(buff));
    // std::string res = buff;
    // DriverLog("our val: %s", res.c_str());
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
  float theta_x = 0.0; // x-axis rotation / povorot po osi x
  float theta_y = 0.0; // y-axis rotatinn / povorot po osi y
  float theta_z = 0.0; // z-axis rotation / povorot po osi z
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float w = 0.0;
  std::vector<float> tracker_pose; // Объявление вектора позиции
  std::shared_ptr<OurDevice> device;
  float m_fCr = 0;
};

// yes
EVRInitError CServerDriver_hobovr::Init(vr::IVRDriverContext *pDriverContext) {
  VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
  InitDriverLog(vr::VRDriverLog());

  m_fCr = vr::VRSettings()->GetFloat("driver_vasiliy", "change_rate");

  DriverLog("driver: version: %d.%d.%d %s \n", hobovr::k_nHobovrVersionMajor,
                            hobovr::k_nHobovrVersionMinor,
                            hobovr::k_nHobovrVersionBuild,
                            hobovr::k_sHobovrVersionGG.c_str());

  tracker_pose = { 0, 0, 0, 1, 0, 0, 0 };

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
  	
  	// ПОВОРОТ ПО ОСИ X
  	
    if (GetAsyncKeyState(0x26)) // Up arrow - x-axis rotation (forward) / Strelka vverh - povorot po osi x (vpered)
    {
      theta_x-=m_fCr;
    }
  
    if (GetAsyncKeyState(0x28)) // Down arrow - x-axis rotation (back) / Strelka vniz - povorot po osi x (nazad)
    {
      theta_x+=m_fCr;
    }
    
  	// ПОВОРОТ ПО ОСИ Y
  	
    if (GetAsyncKeyState(0x25)) // Left arrow - y-axis rotation (left) / Strelka vlevo - povorot po osi y (vlevo)
    {
      theta_y-=m_fCr;
    }
    
    if (GetAsyncKeyState(0x27)) // Right arrow - y-axis rotation (right) / Strelka vpravo - povorot po osi y (vpravo)
    {
      theta_y+=m_fCr;
    }
  	
  	// ПОВОРОТ ПО ОСИ Z
  	
    if (GetAsyncKeyState(0x64)) // NUMPAD 4 key - z-axis rotation (tilt to the left)/ NUMPAD 4 - povorot po osi z (naklon vlevo)
    {
      theta_z-=m_fCr;
    }
    
    if (GetAsyncKeyState(0x66)) // NUMPAD 6 key - z-axis rotation (tilt to the right) / NUMPAD 6 - povorot po osi z (naklon vpravo)
    {
      theta_z+=m_fCr;
    }
  	   POINT p; // - сокращение названия переменной 'p' - позиция
    
  if (GetCursorPos(&p)) // Текущая позиция курсора в 2-х мерном пространстве экрана по осям координат [X/Y]
    {
    	tracker_pose[0] = (double)p.x/1920 - 0.5;
    	tracker_pose[2] = (double)p.y/1080 - 0.5;  
    }
    //
    //if (GetAsyncKeyState(0x53) != 0) // S nazad
    //{       
    //  VR[2]+=0.01; 
    //}
    
    //if (GetAsyncKeyState(0x57)) // W vpered
    //{
     // VR[2]-=0.01; 
    //}
    if (GetAsyncKeyState(0x41)) // A vlevo
    {
      tracker_pose[0]-=m_fCr;
    }
    
    if (GetAsyncKeyState(0x44)) // D vpravo
    {
      tracker_pose[0]+=m_fCr;
    }
    
    if (GetAsyncKeyState(0x51)) { // Q vverh
      tracker_pose[1]+=m_fCr;
    }
    
    if (GetAsyncKeyState(0x45)) { // E vniz
      tracker_pose[1]-=m_fCr;
    }
	
    if (GetAsyncKeyState(0x52)) { // R reset
      tracker_pose[0] = 0.0;
      tracker_pose[1] = 0.0;
      tracker_pose[2] = 0.0;
    }
	
	Vector3 eulerRot = Vector3(theta_x, theta_y, theta_z); // Convert from theta to 3D vector
	Quaternion quatRot = Quaternion::FromEuler(eulerRot); // Conversion from 3D vector to quaternion (заполение полей объекта quatRot)
    tracker_pose[3] = quatRot.W; 
    tracker_pose[4] = quatRot.X;
    tracker_pose[5] = quatRot.Y;
    tracker_pose[6] = quatRot.Z;


    device->RunFrame(tracker_pose);
  
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
