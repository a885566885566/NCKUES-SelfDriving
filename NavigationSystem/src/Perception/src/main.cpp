#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>

#include <sl/Camera.hpp>

#include <socket_pi.h>

using namespace std;
using namespace sl;
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");

// Create ZED objects
Camera zed;

// Detection runtime parameters
int detection_confidence = 35;
ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence);

// Detection output
Objects objects;

bool quit = false;

void retrieve(){
    static clock_t pre_clock = 0;
    static clock_t now_clock = 0;
    sleep_ms(1000);
    while(!quit){
        auto returned_stated = zed.retrieveObjects(objects, detection_parameters_rt);

        if ((returned_stated == ERROR_CODE::SUCCESS) && objects.is_new) {
            //now_clock = clock();
            //cout << "== Retrieve-> "<< (float)(now_clock - pre_clock) / CLOCKS_PER_SEC << endl;
            //pre_clock = now_clock;
            /*cout << "Detected " << objects.object_list.size() << " Object(s)" << endl;
            for( int i=0; i<objects.object_list.size(); i++){
                auto bb = objects.object_list[i].bounding_box;
                if (!bb.empty()){
                    sl::float3 center = objects.object_list[i].position;
                    cout << "Obj: " << center.x <<", "<<center.y<<", "<<center.x<<endl;
                }
            }*/
            send_obs_msg(objects);
        }
    }
}

int main(int argc, char **argv) {
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD720;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    //init_parameters.coordinate_units = UNIT::METER;
    init_parameters.depth_maximum_distance = 50.0f * 1000.0f;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    
    // Open the camera
    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", zed_open_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // Define the Objects detection module parameters
    ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_mask_output = true;
    //detection_parameters.detection_model = DETECTION_MODEL::MULTI_CLASS_BOX;

    // If you want to have object tracking you need to enable positional tracking first
    PositionalTrackingParameters positional_tracking_parameters;
    // If the camera is static in space, enabling this settings below provides better depth quality and faster computation
    // positional_tracking_parameters.set_as_static = true;
    zed.enablePositionalTracking(positional_tracking_parameters);

    print("Object Detection: Loading Module...");
    auto returned_stated = zed.enableObjectDetection(detection_parameters);
    if (returned_stated != ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_stated, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    detection_parameters_rt.detection_confidence_threshold = detection_confidence;

    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = 50;
        
    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    thread t(retrieve);
    t.detach();
    clock_t pre_clock=0, now_clock;
    while (!quit && zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
        now_clock = clock();
        cout << "++ Grab-> " << (float)(now_clock-pre_clock) / CLOCKS_PER_SEC << endl;
        pre_clock = now_clock;
        auto tracking_state = zed.getPosition(cam_pose, REFERENCE_FRAME::WORLD);
        if (tracking_state == POSITIONAL_TRACKING_STATE::OK){
            //sl::float3 center = cam_pose.getEulerAngles();
            //cout << "Pose: "<< center.x <<", "<<center.y<<", "<<center.x<<endl;
            send_pose_msg(cam_pose);
        }
        //retrieve();
        //returned_stated = zed.retrieveObjects(objects, detection_parameters_rt);
        //cout << "Detected " << objects.object_list.size() << " Object(s)" << endl;
        if((float)clock() / CLOCKS_PER_SEC > 50)
            quit = true;
    }
    sleep_ms(1000);
    zed.disableObjectDetection();
    zed.close();
    return EXIT_SUCCESS;
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample] ";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

