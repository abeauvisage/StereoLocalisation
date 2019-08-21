#ifndef ASNCONVERTION
#define ASNCONVERTION

#include <Types/C/Frame.h>
#include <Types/C/Eigen.h>
#include <core/feature_types.h>

#include <memory>

std::shared_ptr<asn1SccFramePair> GenerateStereoPair(const std::pair<cv::Mat,cv::Mat>& cv_pair, const me::CamPose_qd& pose=me::CamPose_qd());
std::shared_ptr<asn1SccFrame> GenerateFrame(const cv::Mat& cv_img);
std::shared_ptr<asn1SccFrame> GenerateDispMap(const cv::Mat& cv_img, const double baseline,const double min_, const double max_);

#endif // ASNCONVERTION

