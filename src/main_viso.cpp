//libviso2
#include <viso_stereo.h>

#include <core/file_IO.h>
#include <gui/Graph2D.h>

#include <chrono>


using namespace std;
using namespace me;
using namespace cv;

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
    openLogFile("poses_viso.csv");
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

    imshow("img_left",current_imgs.first);imshow("img_right",current_imgs.second);
    resizeWindow("img_left",640,480);resizeWindow("img_right",640,480);

    /** MotionEstimation Classes **/

    VisualOdometryStereo::parameters params;
    params.base = param_stereo.baseline;
    params.calib.f = param_stereo.fu1;
    params.calib.cu = param_stereo.cu1;
    params.calib.cv = param_stereo.cv1;
    VisualOdometryStereo viso(params);
    int32_t dims[3] = {current_imgs.first.cols,current_imgs.first.rows,current_imgs.first.cols};
    viso.process(current_imgs.first.ptr<uint8_t>(),current_imgs.second.ptr<uint8_t>(),dims);

    /** main loop **/
    Quatd world_to_cv{(Mat_<double>(3,3) << 0,0,1,-1,0,0,0,-1,0)}; // conversion from standard world coordinat frame to opencv
    CamPose_qd pose_base_to_camera{0,world_to_cv * Euld(-0.7,0,0).getQuat(),{0.675,0.35,0.13}};
//    CamPose_qd pose_base_to_camera{};

    vector<double> ori_err,pos_err;
    CamPose_qd initial_pose = (gt_reader.readPoseLine().second*pose_base_to_camera).inv();
    Mat current_pose = (Mat) initial_pose.TrMat();
    cout << "initial pose: " << initial_pose.inv().orientation << endl;
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
        viso.process(current_imgs.first.ptr<uint8_t>(),current_imgs.second.ptr<uint8_t>(),dims);
        auto to_mat = [](const Matrix& mat){return cv::Mat(mat.m,mat.n,CV_64FC1,*mat.val);};
        cout << to_mat(viso.getMotion()) << endl;
        current_pose = to_mat(viso.getMotion()) * current_pose;


        Mat current_vehicle_pose = ((Mat)pose_base_to_camera.TrMat() * current_pose).inv();
        ori_err.push_back(norm(gt.second.orientation.getEuler().getVector()-Euld(current_pose(Range(0,3),Range(0,3))).getVector()));
        pos_err.push_back(norm(gt.second.position-current_pose(Range(0,3),Range(3,4))));
        graph.addValue(Point2d{-current_vehicle_pose.at<double>(1,3),current_vehicle_pose.at<double>(0,3)},2);
        logFile << gt.first << current_vehicle_pose(Range(0,3),Range(3,4)).t() << Euld(current_pose(Range(0,3),Range(0,3))).getVector() << gt.second.position << gt.second.orientation.getEuler().getVector() << endl;

        auto processing_tp = chrono::steady_clock::now();
        cout << "reading: " << chrono::duration_cast<chrono::milliseconds>(reading_tp-start_tp).count() << "ms." << endl;
        cout << "processing: " << chrono::duration_cast<chrono::milliseconds>(processing_tp-reading_tp).count() << "ms." << endl;

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
    return waitKey();
}

