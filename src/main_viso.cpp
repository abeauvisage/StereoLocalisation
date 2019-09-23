//libviso2
#include <viso_stereo.h>

#include <core/file_IO.h>
#include <gui/Graph2D.h>

#include <chrono>
#include <numeric>

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
    // log file saving camera poses
    openLogFile("poses_viso.csv");
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

    /** display **/

    namedWindow("img_left",CV_WINDOW_FREERATIO);
    namedWindow("img_right",0);
    Graph2D graph("trajectory",2,true);

    // load first images
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
    CamPose_qd pose_base_to_camera{0,dataset_info.q_cam_to_base,dataset_info.p_cam_to_base};
    pair<int64_t,CamPose_qd> gt_pose = gt_reader.readPoseLine();
    while(gt_pose.first < img_reader.img_stamp)
		gt_pose = gt_reader.readPoseLine();

    vector<double> ori_err,pos_err;
    CamPose_qd initial_pose = (gt_pose.second*pose_base_to_camera).inv();
    Mat current_pose = (Mat) initial_pose.TrMat();
    cout << "initial pose: " << initial_pose.inv().orientation << initial_pose.inv().position << endl;

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

