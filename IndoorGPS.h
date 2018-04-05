#ifndef INDOORGPS_H_
#define INDOORGPS_H_

#include <pthread.h>
#include <vector>
//#define EXTERNALGPS_IPADDRESS "192.168.1.74"
#define EXTERNALGPS_IPADDRESS "192.168.1.129"   // GVD Dec 17
#define MAX_NAMELENGTH 256
#define MULTICAST_ADDRESS "239.255.42.99"     		// IANA, local network
#define PORT_DATA 1511                			// Default multicast group
#define GUMSTIX_IPADDRESS "192.168.1.42"

struct  obj_data_t {
   double x;    // z axis in the GPS, x axis for the robot
   double y;	// x axis in the GPS, y axis for the robot
   double z;	// y axis in the GPS, z axis for the robot 
   double th;	// yaw    in the GPS, theta  for the robot
};

class ExternalGPS {
public:
    // Constructor / Destructor
    ExternalGPS(int maxnumobj,const char* ip);
    virtual ~ExternalGPS();
    //void set_values(int, char*, char*);
    double Lat_mav,Lon_mav,Alt_mav;
    float Yaw_mav;
    obj_data_t get_objdata(int obj_ID);
    std::vector<obj_data_t> obj_data_buff;
    int init(void);

  float x_n,y_n,z_n,yaw_n,x_ref,y_ref,z_ref,yaw_ref,pitch,roll,gaz,yaw;

 
private:
    char szMyIPAddress[128];
    char szServerIPAddress[128];
    const char* comp_ip;
    float q_x,q_y,q_z,q_w;
    int DataSocket;


    pthread_t recving;
    virtual void loop(void);
    static void *run(void *args) {
        reinterpret_cast<ExternalGPS*>(args)->loop();
        return NULL;
    }

    char  szData[20000];
	int addr_len;


    // Analyse the data
    void Unpack(char*,int);
    void RadiansToDegrees(float *value);
    void GetEulers(float qx, float qy, float qz, float qw, float *angle1,float *angle2, float *angle3);
};



#endif
