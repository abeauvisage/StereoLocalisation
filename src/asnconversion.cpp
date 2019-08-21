#include "asnconversion.hpp"

#include <core/file_IO.h>

using namespace me;
using namespace cv;
using namespace std;

std::shared_ptr<asn1SccFramePair> GenerateStereoPair(const std::pair<cv::Mat,cv::Mat>& cv_pair, const CamPose_qd& pose){

    assert(cv_pair.first.size() == cv_pair.second.size());

    //creating pair and initialisation
    shared_ptr<asn1SccFramePair> asn_pair{new asn1SccFramePair()};
    asn1SccFramePair_Initialize(asn_pair.get());
    //msg version and metadata
    asn_pair->msgVersion = asn_pair->left.msgVersion = asn_pair->right.msgVersion = frame_Version;
    asn_pair->left.metadata.msgVersion = asn_pair->right.metadata.msgVersion = frame_Version;
    asn_pair->left.intrinsic.msgVersion = asn_pair->right.intrinsic.msgVersion = frame_Version;
    asn_pair->left.metadata.status = asn_pair->right.metadata.status = asn1Sccstatus_VALID;
    asn_pair->left.metadata.pixelModel = asn_pair->right.metadata.pixelModel = asn1Sccpix_UNDEF;
    asn_pair->left.metadata.mode = asn_pair->right.metadata.mode = asn1Sccmode_GRAY;
    //intrinsic parameters
    asn_pair->left.intrinsic.cameraModel = asn_pair->right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
    asn1SccMatrix3d K1 {{ {{param_stereo.fu1,0.0,param_stereo.cu1}},{{0.0,param_stereo.fv1,param_stereo.cv1}},{{0.0,0.0,1.0}} }};
    asn_pair->left.intrinsic.cameraMatrix = K1;
    asn1SccMatrix3d K2 {{ {{param_stereo.fu2,0.0,param_stereo.cu2}},{{0.0,param_stereo.fv2,param_stereo.cv2}},{{0.0,0.0,1.0}} }};
    asn_pair->right.intrinsic.cameraMatrix = K2;
    //extrinsic parameters
    asn_pair->left.extrinsic.hasFixedTransform = asn_pair->right.extrinsic.hasFixedTransform = true;
    asn_pair->left.extrinsic.pose_fixedFrame_robotFrame.data.orientation = {{pose.orientation.x(),pose.orientation.y(),pose.orientation.z(),pose.orientation.w()}};
    asn_pair->left.extrinsic.pose_fixedFrame_robotFrame.data.translation = {{pose.position(0),pose.position(1),pose.position(2)}};
    asn_pair->right.extrinsic.pose_robotFrame_sensorFrame.data.orientation = {{pose.orientation.x(),pose.orientation.y(),pose.orientation.z(),pose.orientation.w()}};
    asn_pair->right.extrinsic.pose_robotFrame_sensorFrame.data.translation = {{pose.position(0),pose.position(1)-param_stereo.baseline,pose.position(2)}};

    //copying images
    asn_pair->left.data.msgVersion = asn_pair->right.data.msgVersion = array3D_Version;
    asn_pair->left.data.rows = asn_pair->right.data.rows = static_cast<asn1SccT_UInt32>(cv_pair.first.rows);
    asn_pair->left.data.cols = asn_pair->right.data.cols = static_cast<asn1SccT_UInt32>(cv_pair.first.cols);
    asn_pair->left.data.channels = asn_pair->right.data.channels = static_cast<asn1SccT_UInt32>(cv_pair.first.channels());
    asn_pair->left.data.depth = asn_pair->right.data.depth = static_cast<asn1SccArray3D_depth_t>(cv_pair.first.depth());
    asn_pair->left.data.rowSize = asn_pair->right.data.rowSize = cv_pair.first.step[0];
    asn_pair->left.data.data.nCount = asn_pair->right.data.data.nCount = static_cast<int>(asn_pair->left.data.rows * asn_pair->left.data.rowSize);
    memcpy(asn_pair->left.data.data.arr, cv_pair.first.data, static_cast<size_t>(asn_pair->left.data.data.nCount));
    memcpy(asn_pair->right.data.data.arr, cv_pair.second.data, static_cast<size_t>(asn_pair->right.data.data.nCount));

    asn_pair->baseline = -param_stereo.baseline;
    return asn_pair;
}

std::shared_ptr<asn1SccFrame> GenerateFrame(const cv::Mat& cv_img){

    assert(!cv_img.empty());

    //creating pair and initialisation
    shared_ptr<asn1SccFrame> asn_img{new asn1SccFrame()};
    asn1SccFrame_Initialize(asn_img.get());
    //msg version and metadata
    asn_img->msgVersion = frame_Version;
    asn_img->metadata.msgVersion = frame_Version;
    asn_img->intrinsic.msgVersion = frame_Version;
    asn_img->metadata.status = asn1Sccstatus_VALID;
    asn_img->metadata.pixelModel = asn1Sccpix_UNDEF;
    asn_img->metadata.mode = asn1Sccmode_GRAY;
    //intrinsic parameters
    asn_img->intrinsic.cameraModel = asn1Scccam_PINHOLE;
    asn1SccMatrix3d K {{ {{param_mono.fu,0.0,param_mono.cu}},{{0.0,param_mono.fv,param_mono.cv}},{{0.0,0.0,1.0}} }};
    asn_img->intrinsic.cameraMatrix = K;
    //extrinsic parameters
    asn_img->extrinsic.hasFixedTransform = false;
    asn_img->extrinsic.pose_fixedFrame_robotFrame.data.orientation = {{0,0,0,1}};
    asn_img->extrinsic.pose_fixedFrame_robotFrame.data.translation = {{0,0,0}};
    asn_img->extrinsic.pose_robotFrame_sensorFrame.data.orientation = {{0,0,0,1}};
    asn_img->extrinsic.pose_robotFrame_sensorFrame.data.translation = {{0,0,0}};

    //copying images
    asn_img->data.msgVersion = array3D_Version;
    asn_img->data.rows = static_cast<asn1SccT_UInt32>(cv_img.rows);
    asn_img->data.cols = static_cast<asn1SccT_UInt32>(cv_img.cols);
    asn_img->data.channels = static_cast<asn1SccT_UInt32>(cv_img.channels());
    asn_img->data.depth = static_cast<asn1SccArray3D_depth_t>(cv_img.depth());
    asn_img->data.rowSize = cv_img.step[0];
    asn_img->data.data.nCount = static_cast<int>(asn_img->data.rows * asn_img->data.rowSize);
    memcpy(asn_img->data.data.arr, cv_img.data, static_cast<size_t>(asn_img->data.data.nCount));

    return asn_img;
}

std::shared_ptr<asn1SccFrame> GenerateDispMap(const cv::Mat& cv_img, const double baseline,const double min_, const double max_){

    assert(!cv_img.empty());

    //creating pair and initialisation
    shared_ptr<asn1SccFrame> asn_img{new asn1SccFrame()};
    asn1SccFrame_Initialize(asn_img.get());
    //msg version and metadata
    asn_img->msgVersion = asn_img->msgVersion = frame_Version;
    asn_img->metadata.msgVersion = frame_Version;
    asn_img->intrinsic.msgVersion = frame_Version;
    asn_img->metadata.status = asn1Sccstatus_VALID;
    asn_img->metadata.pixelModel = asn1Sccpix_DISP;
    asn_img->metadata.mode = asn1Sccmode_GRAY;
    asn1SccVectorXd coeffs = asn1SccVectorXd{5,{1,0,baseline,min_,max_}};
    asn_img->metadata.pixelCoeffs = coeffs;
    //intrinsic parameters
    asn_img->intrinsic.cameraModel = asn1Scccam_PINHOLE;
    asn1SccMatrix3d K {{ {{param_stereo.fu1,0.0,param_stereo.cu1}},{{0.0,param_stereo.fv1,param_stereo.cv1}},{{0.0,0.0,1.0}} }};
    asn_img->intrinsic.cameraMatrix = K;
    //extrinsic parameters
    asn_img->extrinsic.hasFixedTransform = false;
    asn_img->extrinsic.pose_fixedFrame_robotFrame.data.orientation = {{0,0,0,1}};
    asn_img->extrinsic.pose_fixedFrame_robotFrame.data.translation = {{0,0,0}};
    asn_img->extrinsic.pose_robotFrame_sensorFrame.data.orientation = {{0,0,0,1}};
    asn_img->extrinsic.pose_robotFrame_sensorFrame.data.translation = {{0,0,0}};

    //copying images
    asn_img->data.msgVersion = array3D_Version;
    asn_img->data.rows = static_cast<asn1SccT_UInt32>(cv_img.rows);
    asn_img->data.cols = static_cast<asn1SccT_UInt32>(cv_img.cols);
    asn_img->data.channels = static_cast<asn1SccT_UInt32>(cv_img.channels());
    asn_img->data.depth = static_cast<asn1SccArray3D_depth_t>(cv_img.depth());
    asn_img->data.rowSize = cv_img.step[0];
    asn_img->data.data.nCount = static_cast<int>(asn_img->data.rows * asn_img->data.rowSize);
    memcpy(asn_img->data.data.arr, cv_img.data, static_cast<size_t>(asn_img->data.data.nCount));

    return asn_img;
}
