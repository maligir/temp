#ifndef IMAGE_FEATURES_ORB_H
#define IMAGE_FEATURES_ORB_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#include "feature.h"
#include "third_party/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/FORB.h"
#include "third_party/ORB_SLAM3/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace features {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor,
                                   DBoW2::FORB> ORBVocabulary;

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;

const float factorPI = (float)(CV_PI/180.f);

class ORBF : public Feature,
             public XYZFeature,
             public PixelFeature {
public:
  ORBF(const cv::KeyPoint& keypoint,
       const cv::Mat workingImg,
       const Frame* frame,
       const cv::Point* pattern);

  static float IC_Angle(const cv::Mat& image,
                        cv::Point2f pt,
                        const vector<int> & u_max);

  static void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                  const cv::Mat& img,
                                  const cv::Point* pattern,
                                  uchar* desc);

  cv::Point3d getPoint() {
    return _pnt;
  }

  cv::Point2d getPixel() {
    return _kp.pt;
  }

  double getMatchDistance(const Feature* other) const;

  static int ORBdist;
  const static int TH_HIGH = 100;
  const static int TH_LOW = 50;
  const static int HISTO_LENGTH = 30;
  const static size_t dscr_size = 8;

  int32_t* _dscr;
  const cv::KeyPoint _kp;

protected:
  const cv::Point* pattern;
  cv::Point3d _pnt;
};

class ORBFE : public FeatureExtractor {
public:
  enum {HARRIS_SCORE=0, FAST_SCORE=1 };
  ORBFE(int nfeatures, float scaleFactor, int nlevels,
        int iniThFAST, int minThFAST, std::vector<int> &vLappingArea);
  ~ORBFE(){}

  void ExtractFeatures(const Frame* input,
                       std::vector<Feature*>& output);

    int inline GetLevels() {
        return nlevels;
    }

    float inline GetScaleFactor() {
        return scaleFactor;
    }

    std::vector<float> inline GetScaleFactors() {
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors() {
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares() {
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares() {
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax);
    static void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                   const std::vector<cv::Point>& pattern);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
    std::vector<int>& _vLappingArea;
};

class MatchORBbyBOW : public FeatureMatcher {
public:
  MatchORBbyBOW(const ORBVocabulary* voc,
                double nnratio,
                bool checkOri):
                  _voc(voc)
                  , mfNNratio(nnratio)
                  , mbCheckOrientation(checkOri) {}
  
  void findMatches(const Features* src,
                  const Features* pool,
                  std::map<size_t, size_t>& mats) const;

  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

protected:
  void getBoW(const Features* feats,
              DBoW2::BowVector& bvec,
              DBoW2::FeatureVector& fvec) const;

  void ComputeThreeMaxima(vector<int>* histo,
                          const int L,
                          int &ind1,
                          int &ind2,
                          int &ind3) const;
  
  const ORBVocabulary* _voc;
  double mfNNratio;
  bool mbCheckOrientation;
};

}

#endif