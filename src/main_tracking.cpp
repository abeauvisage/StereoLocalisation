#include <optimisation/BundleAdjuster.h>

// lib bundle_adjustment
#include <bundle_adjustment/StereoWindowTracker.hpp>
#include <bundle_adjustment/triangulation.hpp>

// lib MotionEstimation
#include <core/file_IO.h>
#include <gui/Graph2D.h>

//std lib
#include <chrono>

using namespace std;
using namespace me;
using namespace cv;

/*************/
/*    main   */
/*************/

int main(int argc, char *argv[])
{
    /** arguments **/

    if(argc < 2){
        cerr << "please provided a config file!" << endl;
        exit(-1);
    }
    // parsing configuration file
    if(!loadYML(argv[1])){
        std::cerr << "could not load config file" << std::endl;
        exit(-1);
    }

    /** initialisation **/
    // log file saving camera poses
    openLogFile("poses_tracking.csv");
    writeLogFile("#timestamp,p_x,p_y,p_z,r_x,r_y,r_z,gt_p_x,gt_p_y,gt_p_z,gt_r_x,gt_r_y,gt_r_z\n");
    // Helper object to read groundtruth from tf file
    GTReader gt_reader(dataset_info.gt_filename);
    assert(gt_reader.stream.is_open());
    // ignore first two lines of the file (does not contain data)
    gt_reader.readHeader();
    gt_reader.readHeader();
    // helper object to load images easily
    ImageReader img_reader(dataset_info.image_filename,ImageReader::Type::IMAGES);
    cv_sig_handler sig_handler;

    //load first images
    pair<Mat,Mat> current_imgs = img_reader.readStereo();
    if(current_imgs.first.empty() || current_imgs.second.empty()){
        cerr << "One of the images is empty! exiting..." << endl;
        exit(-1);
    }
    //setting tracking options
    Matx33d K {{param_stereo.fu1,0,param_stereo.cu1,0,param_stereo.fv1,param_stereo.cv1,0,0,1}};
    tracking::StereoWindowTracker::StereoTrackingOptions opts = {{K,K},param_stereo.baseline};
    opts.VERBOSE=true;
    opts.BORDER_SIZE = 50;
    opts.MAX_NB_FAILURES = 1;
    opts.MAX_NB_FEATURES = 200;
    opts.MAX_SHIFT = 50;
    //creating a tracker object
    tracking::StereoWindowTracker tracker(current_imgs,opts);

    /** display **/

    Graph2D graph("trajectory",2,true);
    graph.addLegend("Ground truth");
    graph.addLegend("BA",2);

    /** MotionEstimation **/
    CamPose_qd pose_base_to_camera{0,dataset_info.q_cam_to_base,dataset_info.p_cam_to_base};
    pair<int64_t,CamPose_qd> gt_pose = gt_reader.readPoseLine();
    while(gt_pose.first < img_reader.img_stamp)
		gt_pose = gt_reader.readPoseLine();
    //initialising current_camera_pose with ground truth (the pose is expressed in the world frame)
    CamPose_qd current_camera_pose = (gt_pose.second*pose_base_to_camera).inv();

    cout << "initial pose: " << current_camera_pose.inv().orientation << current_camera_pose.inv().position << endl;

    for(;;){
        cout << "##### image " << img_reader.get_img_nb()  << " ####" << endl;
        auto start_tp = chrono::steady_clock::now();
        //reading a new images
        current_imgs = img_reader.readStereo();
        if(current_imgs.first.empty() || current_imgs.second.empty())
            break;

        auto reading_tp = chrono::steady_clock::now();

        //reading current pose from ground truth reader and plotting
        do{
            gt_pose = gt_reader.readPoseLine();
        }while(gt_pose.first && gt_pose.first < img_reader.img_stamp);
        graph.addValue(Point2d{-gt_pose.second.position[1],gt_pose.second.position[0]},1);

        //tracking features
        tracking::StereoWindowTracker::TRACKING_RESULT result;
        tracker.processImages(current_imgs,result);
        //if tracking fails, saving pose and resetting
        if(!tracker.is_tracking_ok()){
            cerr << "[Tracking] tracking failed. resetting..." << endl;
            CamPose_qd current_vehicle_pose = current_camera_pose.inv() * pose_base_to_camera.inv();
            logFile << gt_pose.first << current_vehicle_pose.position << current_vehicle_pose.orientation.getEuler().getVector() << gt_pose.second.position << gt_pose.second.orientation.getEuler().getVector() << endl;
            graph.addValue(Point2d{-current_vehicle_pose.position(1),current_vehicle_pose.position(0)},2);
            tracker.resetTracker(current_imgs);
            continue;
        }
        // retrieve WBA points from tracker
        auto wba_pts = tracker.getPoints();
        optimisation::CalibrationParameters calib({K,K},1.0,param_stereo.baseline);
        std::vector<CamPose_qd> window_poses{current_camera_pose,current_camera_pose};

        //triangulating matches
        for(auto& wba_pt : wba_pts){
            // check that the pose has been observed in the window
            if(wba_pt.getFirstFrameIdx()-current_camera_pose.ID >=0 && wba_pt.getFirstFrameIdx()-current_camera_pose.ID < window_poses.size()){
                wba_pt.set3DLocation(triangulateStereoPoint(wba_pt.getFirstFeat(),{K,K},param_stereo.baseline,window_poses[wba_pt.getFirstFrameIdx()-current_camera_pose.ID]));
            }
        }
        //running bundle adjustement
        optimisation::BundleAdjuster<4> adjuster(calib,window_poses,wba_pts);
        adjuster.optimise(1);// first frame is fixed
        //retrieve optimised camera poses
        auto poses = adjuster.getCameraPoses();
        // the current camera pose correesponds to the last pose of the window poses
        current_camera_pose = {tracker.getcurrentFrameIdx(),(*(poses.end()-1)).orientation,(*(poses.end()-1)).position};

        auto processing_tp = chrono::steady_clock::now();
        cout << "reading: " << chrono::duration_cast<chrono::milliseconds>(reading_tp-start_tp).count() << "ms." << endl;
        cout << "processing: " << chrono::duration_cast<chrono::milliseconds>(processing_tp-reading_tp).count() << "ms." << endl;

        // converts from the current camera pose to the current vehicle pose
        CamPose_qd current_vehicle_pose = current_camera_pose.inv() * pose_base_to_camera.inv();
        //saves vehicle's pose and ground truth in log file and plot trajectories
        logFile << gt_pose.first << current_vehicle_pose.position << current_vehicle_pose.orientation.getEuler().getVector() << gt_pose.second.position << gt_pose.second.orientation.getEuler().getVector() << endl;
        graph.addValue(Point2d{-current_vehicle_pose.position(1),current_vehicle_pose.position(0)},2);

        auto stop_tp = chrono::steady_clock::now();
        cout << "total: " << chrono::duration_cast<chrono::milliseconds>(stop_tp-start_tp).count() << "ms." << endl;

        //wait for user's input
        sig_handler.wait();
    }
    closeLogFile();
    return waitKey();
}

