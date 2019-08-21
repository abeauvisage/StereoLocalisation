#include "asnconversion.hpp"

#include <Types/C/Frame.h>
#include <Types/C/TransformWithCovariance.h>
#include <StereoSlam/StereoSlamOrb.hpp>

#include <core/file_IO.h>
#include <gui/Graph2D.h>

//lib opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// std lib
#include <chrono>


using namespace std;
using namespace me;
using namespace cv;

using StereoSLAM = CDFF::DFN::StereoSlam::StereoSlamOrb;

/*************/
/*    main   */
/*************/

int main(int argc, char *argv[])
{
    if(argc < 3){
        cerr << "please provided a slam config file!" << endl;
        exit(-1);
    }
    if(argc < 2){
        cerr << "please provided a config file!" << endl;
        exit(-1);
    }

    if(!loadYML(argv[1])){
        std::cerr << "could not load config file" << std::endl;
        exit(-1);
    }

    /** initialisation **/
    openLogFile("poses_slam.csv");
    writeLogFile("#timestamp,p_x,p_y,p_z,r_x,r_y,r_z,gt_p_x,gt_p_y,gt_p_z,gt_r_x,gt_r_y,gt_r_z\n");
    GTReader gt_reader(dataset_info.dir+"/../../GT/tf_data.csv");
    assert(gt_reader.stream.is_open());
    gt_reader.readHeader();
    gt_reader.readHeader();

    ImageReader img_reader;
    cv_sig_handler sig_handler;

    /** display **/

    namedWindow("img_left",CV_WINDOW_FREERATIO);
    namedWindow("img_right",0);
    Graph2D graph("trajectory",2,true);

    pair<Mat,Mat> current_imgs = img_reader.readStereo();
    if(current_imgs.first.empty() || current_imgs.second.empty()){
        cerr << "One of the images is empty! exiting..." << endl;
        exit(-1);
    }

    /** MotionEstimation Classes **/
    Quatd world_to_cv{(Mat_<double>(3,3) << 0,0,1,-1,0,0,0,-1,0)}; // conversion from standard world coordinat frame to opencv
    CamPose_qd pose_base_to_camera{0,world_to_cv * Euld(-0.7,0,0).getQuat(),{0.675,0.35,0.13}};
    CamPose_qd initial_pose = gt_reader.readPoseLine().second;
    unique_ptr<StereoSLAM> estimatorSLAM {new StereoSLAM()};
    estimatorSLAM->setConfigurationFile(argv[2]);
    estimatorSLAM->configure();
    shared_ptr<asn1SccFramePair> current_pair = GenerateStereoPair(current_imgs);
    estimatorSLAM->framePairInput(*current_pair);
    estimatorSLAM->process();

    imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
    resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);

    /** main loop **/

    vector<double> ori_err,pos_err;
    cout << "[initial pose: " << initial_pose.orientation << initial_pose.position << endl;
    for(;;){
        cout << "##### image " << img_reader.get_img_nb()  << " ####" << endl;
        auto start_tp = chrono::steady_clock::now();
        current_imgs = img_reader.readStereo();
        if(current_imgs.first.empty() || current_imgs.second.empty()){
            cerr << "images were not loaded properly" << endl;
            break;
        }

        auto reading_tp = chrono::steady_clock::now();
        pair<int64_t,CamPose_qd> gt;
        do{
            gt = gt_reader.readPoseLine();
        }while(gt.first < img_reader.img_stamp);

        graph.addValue(Point2d{-gt.second.position[1],gt.second.position[0]},1);

        current_pair = GenerateStereoPair(current_imgs);
        estimatorSLAM->framePairInput(*current_pair);
        estimatorSLAM->process();
        const asn1SccTransformWithCovariance& outputSLAM_ = estimatorSLAM->poseOutput();
        Vec3d tr_(outputSLAM_.data.translation.arr);
        Vec4d qt_(outputSLAM_.data.orientation.arr);
        CamPose_qd current_pose = initial_pose * CamPose_qd(0,Quatd{qt_(3),qt_(0),qt_(1),qt_(2)},tr_);
        logFile << gt.first << current_pose.position << current_pose.orientation.getEuler().getVector() << gt.second.position << gt.second.orientation.getEuler().getVector() << endl;
        cout << "translation tr "<< tr_ << qt_ << endl;
        cout << "current position: " << current_pose.position << endl;

        auto processing_tp = chrono::steady_clock::now();
        cout << "reading: " << chrono::duration_cast<chrono::milliseconds>(reading_tp-start_tp).count() << "ms." << endl;
        cout << "processing: " << chrono::duration_cast<chrono::milliseconds>(processing_tp-reading_tp).count() << "ms." << endl;

        ori_err.push_back(norm(gt.second.orientation.getEuler().getVector()-current_pose.orientation.getEuler().getVector()));
        pos_err.push_back(norm(gt.second.position-current_pose.position));
        graph.addValue(Point2d{-current_pose.position(1),current_pose.position(0)},2);
        imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
        resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);
        sig_handler.wait();

        auto stop_tp = chrono::steady_clock::now();
        cout << "total: " << chrono::duration_cast<chrono::milliseconds>(stop_tp-start_tp).count() << "ms." << endl;
    }

    double ori_mean_square = std::accumulate(ori_err.begin(),ori_err.end(),0.0,[](double res, double e){return res+e*e;});
    double pos_mean_square = std::accumulate(pos_err.begin(),pos_err.end(),0.0,[](double res, double e){return res+e*e;});
    ori_mean_square/=(double)ori_err.size();
    pos_mean_square/=(double)pos_err.size();
    cout << "RMSE: " << sqrt(ori_mean_square) << " " << sqrt(pos_mean_square) << endl;
    closeLogFile();
    return waitKey();
}

