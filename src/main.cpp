#include <stdexcept>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>


#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>


#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


//MocapNET includes..
//-----------------------------------------------------------------
#include "../dependencies/MocapNET/src/JointEstimator2D/cameraControl.hpp"
#include "../dependencies/MocapNET/src/JointEstimator2D/jointEstimator2D.hpp"
#include "../dependencies/MocapNET/src/JointEstimator2D/visualization.hpp"
//-----------------------------------------------------------------
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/mocapnet2.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/applicationLogic/parseCommandlineOptions.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/commonSkeleton.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/conversions.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/bvh.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/csvRead.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/csvWrite.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/IO/skeletonAbstraction.hpp"
//-----------------------------------------------------------------
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/visualization/visualization.hpp"
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/visualization/map.hpp"
//-----------------------------------------------------------------
#include "../dependencies/MocapNET/src/MocapNET2/MocapNETLib2/tools.hpp"

struct MocapNET2Options options= {0};
struct MocapNET2 mnet= {0};
struct JointEstimator2D jointEstimator;

unsigned int frameID=0;
  
struct calibration
{
    /* CAMERA INTRINSIC PARAMETERS */
    char intrinsicParametersSet;
    float intrinsic[9];
    float k1,k2,p1,p2,k3;

    float cx,cy,fx,fy;
};

#define camRGBRaw "/camera/rgb/image_rect_color"
#define camRGBInfo "/camera/rgb/camera_info"
#define DEFAULT_TF_ROOT "map"
unsigned int width=640;
unsigned int height=480;


//Configuration
char tfRoot[512]={DEFAULT_TF_ROOT}; //This is the string of the root node on the TF tree

unsigned int startTime=0, currentTime=0;


sensor_msgs::CameraInfo camInfo;

volatile bool startTrackingSwitch = false;
volatile bool stopTrackingSwitch = false;
volatile int  key=0;

//ROS
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> RgbSyncPolicy;


ros::NodeHandle * nhPtr=0;
//RGB Subscribers
message_filters::Subscriber<sensor_msgs::Image> *rgb_img_sub;
message_filters::Subscriber<sensor_msgs::CameraInfo> *rgb_cam_info_sub;



bool visualizeOn(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
    ROS_INFO("Visualize On called");
    return true;
}

bool visualizeOff(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
    ROS_INFO("Visualize Off called");
    return true;
}

bool terminate(std_srvs::Empty::Request& request,std_srvs::Empty::Response& response)
{
    ROS_INFO("Terminating MocapNET");
    exit(0);
    return true;
}

int postPoseTransform(const char * name, float x, float y, float z)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0) );  // if we just have xyz we dont have a rotation

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tfRoot, name));
    return 1;
}

void mocapNETProcessImage(cv::Mat frame)
{
    
    std::vector<float> inputValues;
    std::vector<std::vector<float> > exactMocapNET2DOutput;
    std::vector<std::vector<float> > output3DPositions;
    std::vector<float> points3DFlatOutput;
                    unsigned int skippedFramesInARow=0;

    struct skeletonSerialized resultAsSkeletonSerialized= {0};
     
    float frameRateSummary = 0.0;
    unsigned int frameSamples=0;

    std::vector<float> result;
    std::vector<float> previousResult;

    float totalTime=0.0;
    unsigned int totalSamples=0;
    
    std::vector<std::vector<float> > bvhFrames;
    struct Skeletons2DDetected skeleton2DEstimations= {0};
    struct skeletonSerialized skeleton= {0};
                    
                    
    cv::Mat viewMat = Mat(Size(jointEstimator.inputWidth2DJointDetector,jointEstimator.inputHeight2DJointDetector),CV_8UC3, Scalar(0,0,0));
    cv::Mat frameCentered;
    frame.copyTo(frameCentered);
    if ( (frameCentered.size().width>0) && (frameCentered.size().height>0) )
                                {
                                    if (
                                        !cropAndResizeCVMatToMatchSkeleton(
                                            &jointEstimator,
                                            frameCentered,
                                            &skeleton2DEstimations
                                        )
                                    )
                                        {
                                            fprintf(stderr,"Failed to crop input video\n");
                                        }
                                    //imshow("Video Input Feed", frameCentered);

                                    frameCentered.copyTo(viewMat);
                                    // viewMat.setTo(Scalar(0,0,0));

                                    //Tensorflow works with Floating point input so we need to convert our buffer..
                                    frameCentered.convertTo(frameCentered,CV_32FC3); 
                                    
                                    //At this point we are ready to execute the neural network
                                    long startTime2D = GetTickCountMicrosecondsMN();
                                    
                                    //We count the framerate of our acquisition
                                    options.fpsAcquisition = convertStartEndTimeFromMicrosecondsToFPS(options.loopStartTime,startTime2D);
                                    
                                    
                                    std::vector<std::vector<float> >  heatmaps = getHeatmaps(
                                                &jointEstimator,
                                                frameCentered.data,
                                                jointEstimator.inputWidth2DJointDetector,
                                                jointEstimator.inputHeight2DJointDetector
                                            );
                                    if (heatmaps.size()>0)
                                        {
                                            //This will spam with small heatmap windows
                                            //visualizeHeatmaps(&jointEstimator,heatmaps,frameID);

                                            estimate2DSkeletonsFromHeatmaps(&jointEstimator,&skeleton2DEstimations,heatmaps);

                                            long endTime2D = GetTickCountMicrosecondsMN();

                                            options.fps2DEstimator = convertStartEndTimeFromMicrosecondsToFPS(startTime2D,endTime2D);
                                            
                                            if (options.visualize)
                                            {
                                               dj_drawExtractedSkeletons(
                                                viewMat,
                                                &skeleton2DEstimations,
                                                jointEstimator.inputWidth2DJointDetector,
                                                jointEstimator.inputHeight2DJointDetector
                                                );
                                            }

                                            float percentageOf2DPointsMissing = percentOf2DPointsMissing(&skeleton2DEstimations);
                                            if (  percentageOf2DPointsMissing  < 50.0 ) //only work when less than 50% of information missing..
                                                {
                                                    skippedFramesInARow=0;
                                                    //We want to go from the original normalized values of skeleton2DEstimations to the original
                                                    //Resolution we grabbed our initial frame @ before cropping..
                                                    restore2DJointsToInputFrameCoordinates(&jointEstimator,&skeleton2DEstimations);

                                                    //Now that our points have their initial size let's perform a conversion to the internal
                                                    //serialized skeleton data structure that will prepare them for use in MocapNET
                                                    convertSkeletons2DDetectedToSkeletonsSerialized(
                                                        &skeleton,
                                                        &skeleton2DEstimations,
                                                        frameID,
                                                        jointEstimator.crop.frameWidth,
                                                        jointEstimator.crop.frameHeight
                                                    );

                                                    takeCareOfScalingInputAndAddingNoiseAccordingToOptions(&options,&skeleton);

                                                    unsigned int feetAreMissing=areFeetMissing(&skeleton);


                                                    long startTime = GetTickCountMicrosecondsMN();
                                                    //--------------------------------------------------------
                                                    previousResult = result;
                                                    result = runMocapNET2(
                                                                 &mnet,
                                                                 &skeleton,
                                                                 ( (options.doLowerBody) && (!feetAreMissing) ),
                                                                 options.doHands,
                                                                 options.doFace,
                                                                 options.doGestureDetection,
                                                                 options.useInverseKinematics,
                                                                 options.doOutputFiltering
                                                             );
                                                    bvhFrames.push_back(result);
                                                    //--------------------------------------------------------
                                                    long endTime = GetTickCountMicrosecondsMN();
                                                    options.fpsMocapNET = convertStartEndTimeFromMicrosecondsToFPS(startTime,endTime);
                                                    frameRateSummary += options.fpsMocapNET; 
                                                    ++frameSamples;
                                                    //--------------------------------------------------------
                                                    

                                                    options.numberOfMissingJoints = upperbodyCountMissingNSDMElements(mnet.upperBody.NSDM,0 /*Dont spam */);
                                                    //Don't spam with missing joints..
                                                    //fprintf(stderr,"Number of missing joints for UpperBody %u\n",options.numberOfMissingJoints);

                                                    //Convert BVH frame to 2D points to show on screen
                                                    exactMocapNET2DOutput = convertBVHFrameTo2DPoints(result);//,MocapNETTrainingWidth,MocapNETTrainingHeight);

                                                    if (options.saveCSV3DFile)
                                                        {
                                                            //Convert BVH frame to 3D points to output on a file
                                                            points3DFlatOutput=convertBVHFrameToFlat3DPoints(result);//,MocapNETTrainingWidth,MocapNETTrainingHeight);
                                                            output3DPositions.push_back(points3DFlatOutput); //3d Input
                                                        }

                                                    resultAsSkeletonSerialized.skeletonHeaderElements = skeleton.skeletonHeaderElements;
                                                    resultAsSkeletonSerialized.skeletonBodyElements   = skeleton.skeletonBodyElements;
                                                    if (
                                                        convertMocapNET2OutputToSkeletonSerialized(
                                                            &mnet,
                                                            &resultAsSkeletonSerialized,
                                                            exactMocapNET2DOutput,
                                                            frameID,
                                                            MocapNETTrainingWidth,
                                                            MocapNETTrainingHeight
                                                         )
                                                       )
                                                        {
                                                            //TODO : Compare resultAsSkeletonSerialized and skeleton
                                                            doReprojectionCheck(&skeleton,&resultAsSkeletonSerialized);
                                                        }
 
                                                }
                                            else
                                                {
                                                    if (skippedFramesInARow%30==0) { fprintf(stderr,"."); }
                                                    ++skippedFramesInARow;
                                                }

                                            if (options.visualize)
                                                {
                                                    visualizationCommander(
                                                        &mnet,
                                                        &options,
                                                        &skeleton,
                                                        &frame,
                                                        result,
                                                        exactMocapNET2DOutput,
                                                        frameID,
                                                        1// We will do the waitKey call ourselves
                                                    );
 
                                                    imshow("Skeletons", viewMat);
                                                }
                                        }
                                }

}











//RGB Callback is called every time we get a new frame, it is synchronized to the main thread
void rgbCallback(const sensor_msgs::Image::ConstPtr rgb_img_msg,const sensor_msgs::CameraInfo::ConstPtr camera_info_msg)
{
    struct calibration intrinsics= {0};
     
    //ROS_INFO("New frame received..");
    //Using Intrinsic camera matrix for the raw (distorted) input images.

    //Focal lengths presented here are not the same with our calculations ( using zppd zpps )
    //https://github.com/ros-drivers/openni_camera/blob/groovy-devel/src/openni_device.cpp#L197
    intrinsics.fx = camera_info_msg->K[0];
    intrinsics.fy = camera_info_msg->K[4];
    intrinsics.cx = camera_info_msg->K[2];
    intrinsics.cy = camera_info_msg->K[5];

    intrinsics.k1 = camera_info_msg->D[0];
    intrinsics.k2 = camera_info_msg->D[1];
    intrinsics.p1 = camera_info_msg->D[2];
    intrinsics.p2 = camera_info_msg->D[3];
    intrinsics.k3 = camera_info_msg->D[4];

    width = camera_info_msg->width;
    height = camera_info_msg->height;

    //printf("We received an initial frame with the following metrics :  width = %u , height = %u , fx %0.2f , fy %0.2f , cx %0.2f , cy %0.2f \n",width,height,intrinsics.fx,intrinsics.fy,intrinsics.cx,intrinsics.cy);

    //cv::Mat rgb = cv::Mat(width,height,cv::CV_8UC3);
    cv::Mat rgb = cv::Mat::zeros(width,height,CV_8UC3);
    //A new pair of frames has arrived , copy and convert them so that they are ready
    cv_bridge::CvImageConstPtr orig_rgb_img;
    cv_bridge::CvImageConstPtr orig_depth_img;
    orig_rgb_img = cv_bridge::toCvCopy(rgb_img_msg, "rgb8");
    orig_rgb_img->image.copyTo(rgb);

    //printf("Passing Frame width %u height %u fx %0.2f fy %0.2f cx %0.2f cy %0.2f\n",width,height,intrinsics.fx,intrinsics.fy,intrinsics.cx,intrinsics.cy);
    //ROS_INFO("Passing New Frames to Synergies");
    //passNewFrames((unsigned char*) rgb.data,width , height , &intrinsics , &extrinsics);
    //ROS_INFO("Synergies should now have the new frames");

    camInfo = sensor_msgs::CameraInfo(*camera_info_msg);



    cv::Mat rgbTmp = rgb.clone();
    //Take care of drawing stuff as visual output
    cv::Mat bgrMat,rgbMat(height,width,CV_8UC3,rgbTmp.data,3*width);
    cv::cvtColor(rgbMat,bgrMat, cv::COLOR_RGB2BGR);// opencv expects the image in BGR format
    //After we have our bgr Frame ready and we added the FPS text , lets show it!
    cv::imshow("MocapNET - RGB input",bgrMat);
    
    mocapNETProcessImage(bgrMat);
    
    
    std::vector<float> points3D = convertBVHFrameToFlat3DPoints(mnet.currentSolution); 
    for (unsigned int pointID=0; pointID<points3D.size()/3; pointID++)
    {
        char pubName[512];
        snprintf(pubName,512,"%s",MocapNETOutputJointNames[pointID]);
        postPoseTransform(
                          pubName,
                          points3D[pointID*3+0]/100,
                          points3D[pointID*3+1]/100,
                          points3D[pointID*3+2]/100
                         );
        
    }
    
    cv::waitKey(1);

    return;


}
  
int main(int argc, char **argv)
{
    ROS_INFO("Initializing MocapNET ROS Wrapper");
    try
    {
        ros::init(argc, argv, "MocapNET");
        ros::start();

        ros::NodeHandle nh;
        ros::NodeHandle private_node_handle("~");
        nhPtr = &nh;
 
        float rate = 30;
        std::string name;
        std::string fromRGBTopic;
        std::string fromRGBTopicInfo;
        std::string tfRootName;

        ROS_INFO("Initializing Parameters..");
        private_node_handle.param("fromRGBTopic", fromRGBTopic, std::string(camRGBRaw));
        private_node_handle.param("fromRGBTopicInfo", fromRGBTopicInfo, std::string(camRGBInfo));
        private_node_handle.param("name", name, std::string("mocapnet"));
        private_node_handle.param("rate",rate);
        private_node_handle.param("tfRoot",tfRootName, std::string(DEFAULT_TF_ROOT));
        snprintf(tfRoot,510,"%s",tfRootName.c_str());
        fprintf(stderr,"TFRoot Name = %s ",tfRoot);
        
        
        rgb_img_sub = new  message_filters::Subscriber<sensor_msgs::Image>(nh,fromRGBTopic, 1);
        rgb_cam_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,fromRGBTopicInfo,1);
        message_filters::Synchronizer<RgbSyncPolicy> *sync = new message_filters::Synchronizer<RgbSyncPolicy>(RgbSyncPolicy(5), *rgb_img_sub,*rgb_cam_info_sub);
        //rosrun rqt_graph rqt_graph to test out what is going on


        ros::ServiceServer visualizeOnService      = nh.advertiseService(name + "/visualize_on",&visualizeOn);
        ros::ServiceServer visualizeOffService     = nh.advertiseService(name + "/visualize_off", &visualizeOff);
        ros::ServiceServer terminateService        = nh.advertiseService(name + "/terminate", terminate);

        //registerResultCallback((void*) sampleResultingSynergies);
        //registerUpdateLoopCallback((void *) loopEvent);
        //registerROSSpinner( (void *) frameSpinner );
     
        ROS_INFO("Done with ROS initialization!");


    //MocapNET stuff
    
    mnet.options = & options;
    defaultMocapNET2Options(&options);
    
   /*
    *  Force effortless IK configuration 
    */
    //Be unconstrained by default 
     options.constrainPositionRotation=0;
     //Use IK  ========
     options.useInverseKinematics=1;
     options.learningRate=0.01;
     options.iterations=5;
     options.epochs=30.0;
     options.spring=1.0;
     //==============
     options.visualizationType=1;
     
    //640x480 should be a high framerate compatible resolution
    //for most webcams, you can change this using --size X Y commandline parameter
    options.width = 640;
    options.height = 480;
    
    //We might want to load a special bvh file based on our options..! 
    loadOptionsAfterBVHLoadFromCommandlineOptions(&options,argc,argv);

    //If the initialization didnt happen inside the previous call lets do it now
    if (!options.hasInit)
            { 
               if (initializeBVHConverter(0,options.visWidth,options.visHeight))
                 {
                   fprintf(stderr,"BVH code initalization successfull..\n");
                   options.hasInit=1;                   
                 }
            }
            //--------------------------------------------------------------------------
    
    ROS_INFO("Initializing 2D joint estimator");
    if (loadJointEstimator2D(
                             &jointEstimator,
                             options.jointEstimatorUsed,
                             1,
                             options.useCPUOnlyFor2DEstimator
                            ))
        {
            ROS_INFO("Initializing MocapNET");
            if ( loadMocapNET2(&mnet,"MocapNET ROS Node") )
                {
                  sync->registerCallback(rgbCallback);
                  ROS_INFO("Registered ROS services, we should be able to process incoming messages now!");
                  
                  //Lets go..
                  //////////////////////////////////////////////////////////////////////////
                  unsigned long i = 0;
                  // startTime=cvGetTickCount();
                  while ( ( key!='q' ) && (ros::ok()) )
                  {
                      ros::spinOnce();
                      if (i%1000==0)
                         {
                           fprintf(stderr,".");
                           //loopEvent(); //<- this keeps our ros node messages handled up until synergies take control of the main thread
                         }
                      ++i;

                      usleep(1000);
                  } 
                 //////////////////////////////////////////////////////////////////////////
                }else
                { ROS_ERROR("Failed to initialize MocapNET 3D pose estimation.."); }
        } else
        { ROS_ERROR("Failed to initialize MocapNET built-in 2D joint estimation.."); }

    }
    catch(std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    catch(...)               {
        ROS_ERROR("Unknown Error");
        return 1;
    }
 
    ROS_INFO("Shutdown complete");
    return 0;
}
