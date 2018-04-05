#include "IndoorGPS.h"
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <cmath>


 obj_data_t ExternalGPS::get_objdata(int obj_id)
 {
     return obj_data_buff[obj_id];
 }
 
ExternalGPS::ExternalGPS(int maxnumobj,const char* ip)
{
	obj_data_buff.resize(maxnumobj+1);
	comp_ip = ip;
}

ExternalGPS::~ExternalGPS()
{

}
/*
void ExternalGPS::set_values(int id, char *myip, char *serverip)
{
    quadID = id;
//    memcpy(szMyIPAddress,myip,sizeof(&myip));
 //   memcpy(szServerIPAddress,serverip,sizeof(&serverip));
}
*/

// Success:1
int ExternalGPS::init(void)
{
    int retval;
    strcpy(szMyIPAddress, comp_ip);
	//szMyIPAddress[128] = "192.168.1.137";		//Ardrone IP
	strcpy(szServerIPAddress, "192.168.1.74");
	//szServerIPAddress[128] = "192.168.1.145";		//Localization broadcasting IP
	in_addr MyAddress, MultiCastAddress;
	int optval = 0x100000;
	int optval_size = 4;

	MyAddress.s_addr = inet_addr(szMyIPAddress);
	MultiCastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);
	printf("ExternalGPS > Client: %s\n", szMyIPAddress);
	printf("ExternalGPS > Server: %s\n", szServerIPAddress);
	printf("ExternalGPS > Multicast Group: %s\n", MULTICAST_ADDRESS);

	// create a "Data" socket
	DataSocket = socket(AF_INET, SOCK_DGRAM, 0);

	// allow multiple clients on same machine to use address/port
	int value = 1;
	retval = setsockopt(DataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
	if (retval < 0 ){
		close(DataSocket);
		return 0;
	}

	struct sockaddr_in MySocketAddr;
	memset(&MySocketAddr, 0, sizeof(MySocketAddr));
	MySocketAddr.sin_family = AF_INET;
	MySocketAddr.sin_port = htons(PORT_DATA);
	MySocketAddr.sin_addr.s_addr = INADDR_ANY;

	if (bind(DataSocket, (struct sockaddr *)&MySocketAddr, sizeof(sockaddr)) < 0){
		printf("ExternalGPS > [PacketClient] bind failed\n");
		close(DataSocket);
		return 0;
	}

	// join multicast group
	struct ip_mreq Mreq;
	Mreq.imr_multiaddr = MultiCastAddress;
	Mreq.imr_interface = MyAddress;
	retval = setsockopt(DataSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&Mreq, sizeof(Mreq));

	if (retval <0){
		printf("ExternalGPS > [PacketClient] join failed. \n");
		close(DataSocket);
		return 0;
	}

	// create a 1MB buffer
	setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
	getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optval, (socklen_t*)&optval_size);
	if (optval != 0x100000){
		printf("ExternalGPS > [PacketClient] ReceiveBuffer size = %d. \n", optval);
	}

	addr_len = sizeof(struct sockaddr);

    pthread_create(&recving,NULL,run,this);

	return 1;
}


struct sockaddr_in TheirAddress;
void ExternalGPS::loop(void)
{

    while(1){
        recvfrom(DataSocket, szData, sizeof(szData), 0, (sockaddr *)&TheirAddress, (socklen_t *)&addr_len);
        Unpack(szData,1);				//set x,y,z,yaw in Unpack()
        //usleep(50000);
        //sleep(5);
    }
}

void ExternalGPS::Unpack(char* pData,int quadID)
{
    char *ptr = pData;

    //printf("Begin Packet\n-------\n");

    // message ID
    int MessageID = 0;
    memcpy(&MessageID, ptr, 2); ptr += 2;
    //printf("Message ID : %d\n", MessageID);

    // size
    int nBytes = 0;
    memcpy(&nBytes, ptr, 2); ptr += 2;
    //printf("Byte count : %d\n", nBytes);

    if(MessageID == 7)      // FRAME OF MOCAP DATA packet
    {
        // frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
        //printf("Frame # : %d\n", frameNumber);

	    // number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
        //printf("Marker Set Count : %d\n", nMarkerSets);

        for (int i=0; i < nMarkerSets; i++)
        {
            // Markerset name
            char szName[256];
            strcpy(szName, ptr);
            int nDataBytes = (int) strlen(szName) + 1;
            ptr += nDataBytes;
           // printf("Model Name: %s\n", szName);

        	// marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
            //printf("Marker Count : %d\n", nMarkers);

            for(int j=0; j < nMarkers; j++)
            {
                float x = 0; memcpy(&x, ptr, 4); ptr += 4;
                float y = 0; memcpy(&y, ptr, 4); ptr += 4;
                float z = 0; memcpy(&z, ptr, 4); ptr += 4;
                //printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n",j,x,y,z);
            }
        }

	    // unidentified markers
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
        //printf("Unidentified Marker Count : %d\n", nOtherMarkers);
        for(int j=0; j < nOtherMarkers; j++)
        {
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            //printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
        }

        // rigid bodies
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        //printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
        {
            // rigid body pos/ori
            int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
            float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
            float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
            float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
            float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
            float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
            float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
            float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;

	//printf("ID : %d\n", ID);
           // printf("pos: [%3.2f,%3.2f,%3.2f]\n",-z*1000,x*1000,-y*1000);
           // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);


        float yaw_l, pitch_l, roll_l;
        GetEulers(qx,qy,qz,qw,&pitch_l,&roll_l,&yaw_l);

        x_n=z*1000;  // z is z-axis in the GPS, and x_n is x-axis in the robot
        y_n=x*1000;  // x is x-axis in the GPS, and y_n is y-axis in the robot
        z_n=y*1000;  // y is y-axis in the GPS, and z_n is z-axis in the robot (don't need for UGV)
        yaw_n=yaw_l;

        obj_data_buff[ID].x=x_n;
        obj_data_buff[ID].y=y_n;
        obj_data_buff[ID].z=z_n;
        obj_data_buff[ID].th=yaw_n;




            // associated marker positions
            int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
            //printf("Marker Count: %d\n", nRigidMarkers);
            int nBytes = nRigidMarkers*3*sizeof(float);
            float* markerData = (float*)malloc(nBytes);
            memcpy(markerData, ptr, nBytes);
            ptr += nBytes;


            // associated marker IDs
            nBytes = nRigidMarkers*sizeof(int);
            int* markerIDs = (int*)malloc(nBytes);
            memcpy(markerIDs, ptr, nBytes);
            ptr += nBytes;

            // associated marker sizes
            nBytes = nRigidMarkers*sizeof(float);
            float* markerSizes = (float*)malloc(nBytes);
            memcpy(markerSizes, ptr, nBytes);
            ptr += nBytes;

            for(int k=0; k < nRigidMarkers; k++)
            {
                //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], 			markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
            }

            if(markerIDs)
                free(markerIDs);
            if(markerSizes)
                free(markerSizes);

            if(markerData)
                free(markerData);

            // Mean marker error
            float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
            //printf("Mean marker error: %3.2f\n", fError);

	} // next rigid body


        // skeletons
        int nSkeletons = 0;
        memcpy(&nSkeletons, ptr, 4); ptr += 4;
        //printf("Skeleton Count : %d\n", nSkeletons);
        for (int j=0; j < nSkeletons; j++)
        {
            // skeleton id
            int skeletonID = 0;
            memcpy(&skeletonID, ptr, 4); ptr += 4;
            // # of rigid bodies (bones) in skeleton
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4); ptr += 4;
            //printf("Rigid Body Count : %d\n", nRigidBodies);
            for (int j=0; j < nRigidBodies; j++)
            {
                    // rigid body pos/ori
                int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
                float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
                float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
                float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
                float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
                float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
                float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
                float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
                //printf("ID : %d\n", ID);
               // printf("pos: [%3.2f,%3.2f,%3.2f]\n", x,y,z);
                //printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);
                 // associated marker positions
                int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
                //printf("Marker Count: %d\n", nRigidMarkers);
                int nBytes = nRigidMarkers*3*sizeof(float);
                float* markerData = (float*)malloc(nBytes);
                memcpy(markerData, ptr, nBytes);
                ptr += nBytes;

                // associated marker IDs
                nBytes = nRigidMarkers*sizeof(int);
                int* markerIDs = (int*)malloc(nBytes);
                memcpy(markerIDs, ptr, nBytes);
                ptr += nBytes;
                // associated marker sizes
                nBytes = nRigidMarkers*sizeof(float);
                float* markerSizes = (float*)malloc(nBytes);
                memcpy(markerSizes, ptr, nBytes);
                ptr += nBytes;
                for(int k=0; k < nRigidMarkers; k++)
                {
                    //printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
                }

                // Mean marker error
                float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
                //printf("Mean marker error: %3.2f\n", fError);

                // release resources
                if(markerIDs)
                    free(markerIDs);
                if(markerSizes)
                    free(markerSizes);
                if(markerData)
                    free(markerData);

            } // next rigid body

        } // next skeleton

        // latency
        float latency = 0.0f; memcpy(&latency, ptr, 4);	ptr += 4;
        //printf("latency : %3.3f\n", latency);

	// end of data tag
        int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
        //printf("End Packet\n-------------\n");

    }
    else if(MessageID == 5) // Data Descriptions
    {
        // number of datasets
        int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
        //printf("Dataset Count : %d\n", nDatasets);

        for(int i=0; i < nDatasets; i++)
        {
            //printf("Dataset %d\n", i);

            int type = 0; memcpy(&type, ptr, 4); ptr += 4;
            //printf("Type : %d\n", i, type);

            if(type == 0)   // markerset
            {
                // name
                char szName[256];
                strcpy(szName, ptr);
                int nDataBytes = (int) strlen(szName) + 1;
                ptr += nDataBytes;
                //printf("Markerset Name: %s\n", szName);

        	    // marker data
                int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
                //printf("Marker Count : %d\n", nMarkers);

                for(int j=0; j < nMarkers; j++)
                {
                    char szName[256];
                    strcpy(szName, ptr);
                    int nDataBytes = (int) strlen(szName) + 1;
                    ptr += nDataBytes;
                    //printf("Marker Name: %s\n", szName);
                }
            }
            else if(type ==1)   // rigid body
            {
                // name
                char szName[MAX_NAMELENGTH];
                strcpy(szName, ptr);
                ptr += strlen(ptr) + 1;
                //printf("Name: %s\n", szName);


                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                //printf("ID : %d\n", ID);

                int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
                //printf("Parent ID : %d\n", parentID);

                float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
                //printf("X Offset : %3.2f\n", xoffset);

                float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
                //printf("Y Offset : %3.2f\n", yoffset);

                float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
                //printf("Z Offset : %3.2f\n", zoffset);

            }
            else if(type ==2)   // skeleton
            {
                char szName[MAX_NAMELENGTH];
                strcpy(szName, ptr);
                ptr += strlen(ptr) + 1;
                //printf("Name: %s\n", szName);

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                //printf("ID : %d\n", ID);

                int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
                //printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

                for(int i=0; i< nRigidBodies; i++)
                {
                    // RB name
                    char szName[MAX_NAMELENGTH];
                    strcpy(szName, ptr);
                    ptr += strlen(ptr) + 1;
                    //printf("Rigid Body Name: %s\n", szName);

                    int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                    //printf("RigidBody ID : %d\n", ID);

                    int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
                    //printf("Parent ID : %d\n", parentID);

                    float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
                    //printf("X Offset : %3.2f\n", xoffset);

                    float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
                    //printf("Y Offset : %3.2f\n", yoffset);

                    float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
                    //printf("Z Offset : %3.2f\n", zoffset);
                }
            }

        }   // next dataset

      // printf("End Packet\n-------------\n");

    }
    else
    {
        //printf("Unrecognized Packet Type.\n");
    }

}

void ExternalGPS::RadiansToDegrees(float *value)
{
    *value = (*value)*(180.0f/3.14159265f);
}

void ExternalGPS::GetEulers(float qx, float qy, float qz, float qw, float *angle1,float *angle2, float *angle3)
{
    float &heading = *angle1;
	float &attitude = *angle2;
	float &bank = *angle3;
	double test = qw*qx + qy*qz;
	if (test > 0.499) { 					// singularity at north pole
		heading = (float) 2.0f * atan2(qy,qw);
		attitude = 3.14159265f/2.0f;
		bank = 0;
		RadiansToDegrees(&heading);
		RadiansToDegrees(&attitude);
		RadiansToDegrees(&bank);
		return;
	}
	if (test < -0.499) {						// singularity at south pole
		heading = (float) -2.0f * atan2(qy,qw);
		attitude = - 3.14159265f/2.0f;
		bank = 0;
		RadiansToDegrees(&heading);
		RadiansToDegrees(&attitude);
		RadiansToDegrees(&bank);
		return;
	}
	double sqx = qx*qx;
	double sqy = qy*qy;
	double sqz = qz*qz;
	heading = (float) atan2((double)2.0*qw*qz-2.0*qx*qy , (double)1 - 2.0*sqz - 2.0*sqx);
	attitude = (float)asin(2.0*test);
	bank = (float) atan2((double)2.0*qw*qy-2.0*qx*qz , (double)1.0 - 2.0*sqy - 2.0*sqx);
	RadiansToDegrees(&heading);
	RadiansToDegrees(&attitude);
	RadiansToDegrees(&bank);
}

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();



 uint16_t buff_length;
 uint8_t * buff_msg;
