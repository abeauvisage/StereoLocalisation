#include "asnconversion.hpp"

#include <core/file_IO.h>
#include <gui/Graph2D.h>

//lib edres
#include <StereoMotionEstimation/StereoMotionEstimationEdres.hpp>
#include <edres-wrapper/EdresStereo.h>
#include <VisualSlamStereo/VisualSlamStereo.hpp>

#include <chrono>


using namespace std;
using namespace me;
using namespace cv;

using StereoVO = CDFF::DFN::StereoMotionEstimation::StereoMotionEstimationEdres;

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


//    Ptr<StereoBM> bm_ptr = cv::StereoBM::create(96);

//    auto computeDisparityMap =[bm_ptr](const pair<Mat,Mat>& imgs){
//        Mat disp;
//        bm_ptr->compute(imgs.first,imgs.second,disp);
//        disp/=16;
//        disp.convertTo(disp,CV_32F);
//        double min_,max_;Point minLoc,maxLoc;
//        cv::minMaxLoc(disp,&min_,&max_,&minLoc,&maxLoc);
//        imshow("disparity",disp/max_);
//        return make_pair(disp,max_);
//    };

//    pair<Mat,double> cv_disp = computeDisparityMap(current_imgs);

    CamPose_qd current_pose = gt_reader.readPoseLine().second;//,initial_pose{0,{initial_pose_tmp.orientation.w(),initial_pose_tmp.orientation.x(),initial_pose_tmp.orientation.y(),initial_pose_tmp.orientation.z()},{initial_pose_tmp.position[0],initial_pose_tmp.position[1],initial_pose_tmp.position[2]}};
    unique_ptr<StereoVO> estimatorME {new StereoVO()};
    estimatorME->configure();
    shared_ptr<asn1SccFramePair> current_pair = GenerateStereoPair(current_imgs,current_pose);
//    shared_ptr<asn1SccFrame> asn_disp = GenerateDispMap(cv_disp.first,param_stereo.baseline,0,cv_disp.second);
    shared_ptr<asn1SccFrame> asn_disp {new asn1SccFrame};
    Edres::disparities(current_pair->left,current_pair->right,*asn_disp,param_stereo.baseline);
    estimatorME->framePairInput(*current_pair);
    estimatorME->disparityInput(*asn_disp);
    estimatorME->process();
    const asn1SccTransformWithCovariance& outputME = estimatorME->poseOutput();
    Vec3d tr(outputME.data.translation.arr);
    Vec4d qt(outputME.data.orientation.arr);
    cout << "translation tr "<< tr << qt << endl;

    /** main loop **/

    cout << "initial pose: " << current_pose.orientation << current_pose.position << endl;
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
//        cv_disp = computeDisparityMap(current_imgs);
//        current_pair = GenerateStereoPair(current_imgs);
//        asn_disp = GenerateDispMap(cv_disp.first,param_stereo.baseline,0,cv_disp.second);
        Edres::disparities(current_pair->left,current_pair->right,*asn_disp,param_stereo.baseline);
        estimatorME->framePairInput(*current_pair);
        estimatorME->disparityInput(*asn_disp);
        estimatorME->process();
        const asn1SccTransformWithCovariance& outputME_ = estimatorME->poseOutput();
        Vec3d tr_(outputME_.data.translation.arr);
        Vec4d qt_(outputME_.data.orientation.arr);
        current_pose.position = tr_;current_pose.orientation = Quatd{qt_(3),qt_(0),qt_(1),qt_(2)};
        cout << "translation tr "<< tr_ << qt_ << endl;

        auto processing_tp = chrono::steady_clock::now();
        cout << "reading: " << chrono::duration_cast<chrono::milliseconds>(reading_tp-start_tp).count() << "ms." << endl;
        cout << "processing: " << chrono::duration_cast<chrono::milliseconds>(processing_tp-reading_tp).count() << "ms." << endl;

        graph.addValue(Point2d{-current_pose.position(1),current_pose.position(0)},2);
        imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
        resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);
        sig_handler.wait();

        auto stop_tp = chrono::steady_clock::now();
        cout << "total: " << chrono::duration_cast<chrono::milliseconds>(stop_tp-start_tp).count() << "ms." << endl;
    }
    return waitKey();
}

