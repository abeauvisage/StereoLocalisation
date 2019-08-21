#include "asnconversion.hpp"

#include <core/file_IO.h>
#include <gui/Graph2D.h>

#include <Types/C/Frame.h>
#include <Types/C/TransformWithCovariance.h>
#include <VisualSlamStereo/VisualSlamStereo.hpp>


//lib opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// std lib
#include <chrono>


using namespace std;
using namespace me;
using namespace cv;

using StereoSLAM = CDFF::DFPC::VisualSlamStereo;

/*************/
/*    main   */
/*************/

int main(int argc, char *argv[])
{
    if(argc < 2){
        cerr << "please provided a config file!" << endl;
        exit(-1);
    }

    if(!loadYML(argv[1])){
        std::cerr << "could not load config file" << std::endl;
        exit(-1);
    }

    /** initialisation **/

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
    CamPose_qd initial_pose = gt_reader.readPoseLine().second;//,initial_pose{0,{initial_pose_tmp.orientation.w(),initial_pose_tmp.orientation.x(),initial_pose_tmp.orientation.y(),initial_pose_tmp.orientation.z()},{initial_pose_tmp.position[0],initial_pose_tmp.position[1],initial_pose_tmp.position[2]}};
    unique_ptr<StereoSLAM> estimatorSLAM {new StereoSLAM()};
    estimatorSLAM->setConfigurationFile("../rsrc/VisualSlamStereo.yaml");
    estimatorSLAM->setup();
    shared_ptr<asn1SccFramePair> current_pair = GenerateStereoPair(current_imgs);
    estimatorSLAM->framePairInput(*current_pair);
    estimatorSLAM->run();

    imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
    resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);

    /** main loop **/

    cout << "initial pose: " << initial_pose.orientation << initial_pose.position << endl;
    for(;;){
        cout << "##### image " << img_reader.get_img_nb()  << " ####" << endl;
        auto start_tp = chrono::steady_clock::now();
        current_imgs = img_reader.readStereo();
        if(current_imgs.first.empty() || current_imgs.second.empty())
            break;

        auto reading_tp = chrono::steady_clock::now();
        pair<int64_t,CamPose_qd> gt;
        do{
            gt = gt_reader.readPoseLine();
        }while(gt.first < img_reader.img_stamp);

        graph.addValue(Point2d{-gt.second.position[1],gt.second.position[0]},1);

        current_pair = GenerateStereoPair(current_imgs);
        estimatorSLAM->framePairInput(*current_pair);
        estimatorSLAM->run();
        const asn1SccTransformWithCovariance& outputSLAM_ = estimatorSLAM->estimatedPoseOutput();
        Vec3d tr_(outputSLAM_.data.translation.arr);
        Vec4d qt_(outputSLAM_.data.orientation.arr);
        CamPose_qd current_pose = initial_pose * CamPose_qd{0,Quatd{qt_(3),qt_(0),qt_(1),qt_(2)},tr_};
        cout << "translation tr "<< tr_ << qt_ << endl;

        auto processing_tp = chrono::steady_clock::now();
        cout << "reading: " << chrono::duration_cast<chrono::milliseconds>(reading_tp-start_tp).count() << "ms." << endl;
        cout << "processing: " << chrono::duration_cast<chrono::milliseconds>(processing_tp-reading_tp).count() << "ms." << endl;

        graph.addValue(Point2d{-current_pose.position(1),current_pose.position(0)},2);
        imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
        resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);
        waitKey(0);

        auto stop_tp = chrono::steady_clock::now();
        cout << "total: " << chrono::duration_cast<chrono::milliseconds>(stop_tp-start_tp).count() << "ms." << endl;
    }
    closeLogFile();
    return waitKey();
}

