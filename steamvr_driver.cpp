#include "hobovr_components.h"
#include "hobovr_device_base.h"
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

int main()
{
  float thetaOver2 = 0.0, theta = 0.0, x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  auto my_device = OurDevice("lol");
  
  std::vector<float> VR = {0, 0, 0, 1, 0, 0, 0};
  
  while (true) {
	// do something with VR
    if (GetAsyncKeyState(0x25)) // Left arrow - strelochka vlevo
    {
      theta-=0.001;
    }
    
    if (GetAsyncKeyState(0x27)) // Right arrow - strelochka vpravo
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
    
    if (GetAsyncKeyState(0x20)) { // Space probel
      VR[1]+=0.10;
      my_device.RanFrame(VR);
      std::this_thread::sleep_for(std::chrono::seconds(1));
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
  }
}


/* ?
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
???
