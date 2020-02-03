#include <iostream>
#include "VirtuoseAPI.h"

#define HAPTION_CONTROLLER_IP "192.168.0.3"

//***********************************************************************************
//------------------- Functions defined by myself -----------------------------------
//***********************************************************************************

    bool openConnectionToHaption(VirtContext & VC_)
    {
        VC_ = virtOpen(HAPTION_CONTROLLER_IP);
        if (VC_ == NULL)
        {
            fprintf(stderr, "Error occurs when connected to the Haption_Virtuose_Controller: %s\n", virtGetErrorMessage(virtGetErrorCode(NULL)));
            return false;
        }
        else
        {
            std::cout << "--- Successfully Connected to Haption Virtuose! ------------------- " << std::endl;
            std::cout << "--- Configur the Haption Device ------------------- " << std::endl;
            // set the control model.
            // INDEXING_ALL authorizes indexing on all movements, i.e. rotations and translations.
            // INDEXING_TRANS authorizes indexing only on translations.
            // INDEXING_NONE forbids indexing on all movements
            float identity[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            virtSetIndexingMode(VC_, INDEXING_ALL);
            // set force/speed factor
            virtSetForceFactor(VC_, 1.0f);
            virtSetSpeedFactor(VC_, 1.0f);
            // set simulator intergration timestep
            virtSetTimeoutValue(VC_, 0.003f);
            // set the base reference frame with respect to the environment reference frame
            virtSetBaseFrame(VC_, identity);
            // the position of the observation reference frame with respect to the environment reference frame
            virtSetObservationFrame(VC_, identity);
            // set the command type
            virtSetCommandType(VC_, COMMAND_TYPE_IMPEDANCE);
            std::cout << "----------------- Configuration is finished -------------------" << std::endl;
            virtSetPowerOn(VC_, 1);
            std::cout << "Now you can activate the force-feedback at any time, by switching the power switch to ON: ------" << std::endl;


            return true;
        }
    }

    bool closeConnectionToHaption(VirtContext & VC_)
    {
        int result = virtClose(VC_);
        if (result != 0)
        {
            fprintf(stderr, "Error occurs when close connection to the Haption_Virtuose_Controller: %s\n", virtGetErrorMessage(virtGetErrorCode(VC_)));
            return false;
        }
        else {
            std::cout << "Successfully close the connection to the Haption Device." << std::endl;
            return true;
        }
    }


    // @author
//          Gaofeng Li: gaofeng.li@iit.it
// @data
//          13-Mar-2019
// @description
//          Get the current pose, which is a vector with 7 components: 3 for position + 4 quaternion, from the Haption Virtuose 6D.
//          This function is called by "bool getPositionfromHaption(VirtContext & VC_, cVector3d& position_result)"
//                                  and "bool getRotationfromHaption(VirtContext & VC_, cVector3d& position_result)"
// @param
//          VirtContext & VC_:  the handler of the device.
//          float * pose_result:  the result is stored in this vector
// @return
//          true: if get the pose successfully
//          false: if there is error
    bool getPosefromHaption(VirtContext & VC_, float* pose_result)
    {
        int result = virtGetPosition(VC_, pose_result);

        if (result != 0)
        {
            fprintf(stderr, "Error occurs when get pose from the Haption_Virtuose_Controller: %s\n", virtGetErrorMessage(virtGetErrorCode(VC_)));
            return false;
        }
        else {
            if (pose_result == NULL)
            {
                std::cout << "getPosefromHaption: pose_result is null" << std::endl;
                return false;
            }
            else {
                return true;
            }
        }
    }


int main(int argc, char* argv[])
{
    std::cout << "Test of the connection to Haption" << std::endl;
    VirtContext VC;
    float * pose_result;
    pose_result = new float[7];


    bool result = true;
    result = openConnectionToHaption(VC);
    if(result)
    {
        std::cout << "Connection is created successfully!!" << std::endl;
        result = getPosefromHaption(VC, pose_result);
    }else
    {

    }
    // exit
    return 0;
}