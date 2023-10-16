#include <vision_geometry/CameraIntrinsics.h>
#include <vision_geometry/CameraOptimizationParameters.h>
#include <vision_geometry/DistortionModel.h>

#include <vision_geometry/HomographyShortcuts.h>
#include <vision_geometry/LinearAlgebraShortcuts.h>
#include <vision_geometry/OptRTDistCostFunctor.h>
#include <vision_geometry/RigidTrans.h>
#include <vision_geometry/SingleCameraOptimizer.h>
#include <vision_geometry/TransformShortcuts.h>
#include <vision_geometry/Util.h>
#include <vision_geometry/ZhangLinearEstimate.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "ceres/ceres.h"

#include "opencv2/core/eigen.hpp"

using namespace ceres;
using namespace cv;
using namespace Eigen;
using namespace std;

int main(int argc, char **argv) {
  srand(time(NULL));

  Size s(3, 4);
  ModelChessboard chessboard(s.height, s.width, 23.0);

  //string modelName = "head";
  string modelName = "a";
  // string modelName = "head_fish";
  // string modelName = "head_rgbd";
  vector<string> files = listFiles(
      "/home/henry/Desktop/test/head/");
  //cout << files[1] << endl;
  vector<Mat> detections, images = loadImages(files);
  cout << "loadImages:  " << images.size() << endl;
  vector<MatrixXd> camPoints;
  detectCorners(s, images, detections, camPoints);

  cout << images.size() << endl;
  cout << images[0].rows << ", " << images[0].cols << endl;
  imshow("head", images[0]);
  waitKey(0);

  vector<MatrixXd> homographies = computeHomographies(chessboard.getModelCBH2D(), camPoints);

  ZhangLinearEstimate zhangLE(homographies);
  zhangLE.prettyPrint();

  vector<MatrixXd> optH =
      optimizeHomographies(chessboard.getModelCBH2D(), camPoints, homographies);

  vector<RigidTrans> rtoEs = extractRTs(zhangLE.getCameraIntrinsics(), optH);

  CameraOptimizationParameters cop(zhangLE.getCameraIntrinsics());
  SingleCameraOptimizer sco(cop, zhangLE.getCameraIntrinsics(),
                            chessboard.getModelCBH3D());
  sco.update(camPoints, rtoEs);
  sco.prettyPrint();

  double u0 = images[0].cols / 2;
  double v0 = images[0].rows / 2;
  cop.setU0V0(u0, v0);
  cop.setU0V0Pinned(true);
  sco.update(camPoints, rtoEs);
  sco.prettyPrint();

  cop.setGamma(0.0);
  cop.setGammaPinned(true);
  sco.update(camPoints, rtoEs);
  sco.prettyPrint();

  cop.setAlphaBetaEqual(true);
  sco.update(camPoints, rtoEs);
  sco.prettyPrint();

  cop.setAlpha(sco.getKOpt().getAlpha());
  cop.setBeta(sco.getKOpt().getBeta());
  cop.setAlphaBetaPinned(true);
  sco.update(camPoints, rtoEs);
  sco.prettyPrint();

  fstream fs;
  fs.open((modelName + ".yaml").c_str(), fstream::out | fstream::trunc);

  fs.precision(10);
  fs << fixed;
  fs << "image_width:   " << images[0].cols << endl;
  fs << "image_height:   " << images[0].rows << endl;
  fs << "camera_name: <fill_me_in>" << endl;
  fs << "camera_matrix:" << endl;
  fs << "   rows:   3" << endl;
  fs << "   cols:   3" << endl;
  fs << "   data:   [" << sco.getKOpt().getAlpha() << ", "
     << sco.getKOpt().getGamma() << ", " << sco.getKOpt().getU0() << ", "
     <<

      "0"
     << ", " << sco.getKOpt().getBeta() << ", " << sco.getKOpt().getV0() << ", "
     <<

      "0"
     << ", "
     << "0"
     << ", "
     << "1"
     << "]" << endl;
  fs << "distortion_model: plumb_bob" << endl;
  fs << "distortion_coefficients:" << endl;
  fs << "   rows:   1" << endl;
  fs << "   cols:   5" << endl;
  fs << "   data:   [" << sco.getDMOpt().getK1() << ", "
     << sco.getDMOpt().getK2() << ", " << sco.getDMOpt().getP1() << ", "
     << sco.getDMOpt().getP2() << ", " << sco.getDMOpt().getK3() << "]" << endl;
  fs << "rectification_matrix:" << endl;
  fs << "   rows:   3" << endl;
  fs << "   cols:   3" << endl;
  fs << "   data:   ["
     << "1"
     << ", "
     << "0"
     << ", "
     << "0"
     << ", "
     <<

      "0"
     << ", "
     << "1"
     << ", "
     << "0"
     << ", "
     <<

      "0"
     << ", "
     << "0"
     << ", "
     << "1"
     << "]" << endl;
  fs << "projection_matrix:" << endl;
  fs << "   rows:   3" << endl;
  fs << "   cols:   4" << endl;
  fs << "   data:   [" << sco.getKOpt().getAlpha() << ", "
     << sco.getKOpt().getGamma() << ", " << sco.getKOpt().getU0() << ", "
     << "0"
     << ", "
     <<

      "0"
     << ", " << sco.getKOpt().getBeta() << ", " << sco.getKOpt().getV0() << ", "
     << "0"
     << ", "
     <<

      "0"
     << ", "
     << "0"
     << ", "
     << "1"
     << ", "
     << "0"
     << "]" << endl;

  /*
  Mat map1, map2;
  initUndistortRectifyMap(
      sco.getKOpt().getCVMat(), sco.getDMOpt().getDistortionCoeffsCV()
      , Mat::eye(3, 3, CV_32F), sco.getKOpt().getCVMat()
      , Size(640, 480), CV_32FC1
      , map1, map2);

  namedWindow("Calibration Output");
  for(size_t i = 0; i < images.size(); i++) {
      Mat out;
      remap(images[i], out, map1, map2, INTER_LINEAR);
      imshow("Calibration Output", out);
      waitKey();
  }

  //Width, Height
  namedWindow("Calibration Output");
  for(int i = 0; i < camPoints.size(); i++) {
      Scalar colorA(0,255,255, 127);
      Scalar colorB(255,255,0, 127);
      Mat frameBGRA(480, 640, CV_8UC4, Scalar(0, 0, 0, 255));
      showPoints(frameBGRA, camPoints[i], colorA);
      showPoints(frameBGRA, homPts[i], colorB);
      waitKey();
  }
  */

  return 0;
}
