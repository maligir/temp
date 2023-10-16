#include "mm_point_cloud_iou.h"

#include "data_packet.h"
#include "orbslam_utils.h"
#include "occupation_map_2d.h"

MMPointCloudIOU::MMPointCloudIOU(float max_lin_vel,
                                 float max_angl_vel,
                                 size_t nsamples,
                                 float step_decay,
                                 float lin_step_min,
                                 float ang_step_min,
                                 float near_thr,
                                 float max_radius,
                                 uint16_t dmin,
                                 uint16_t dmax,
                                 size_t max_unmerged):
        _mlinv(max_lin_vel)
        , _manglv(max_angl_vel)
        , _nsamples(nsamples)
        , _step_decay(step_decay)
        , _lin_step_min(lin_step_min)
        , _ang_step_min(ang_step_min)
        , _near_thr(near_thr)
        , _max_radius(max_radius)
        , _dmin(dmin)
        , _dmax(dmax)
        , _kdebug(false)
        , _hasRecovered(false)
        , _unmerged_count(0)
        , _max_unmerged(max_unmerged) {
    for (size_t i = 1; i <= _nsamples; i++) {
        _h1.push_back(Halton(i, 2));
        _h2.push_back(Halton(i, 3));
        _h3.push_back(Halton(i, 5));
    }
}

float MMPointCloudIOU::Halton(size_t i, size_t b) {
    float f = 1;
    float r = 0;

    while (i > 0) {
        f = f / b;
        r = r + f * (i % b);
        i = i/b;
    }

    return r;
}

void MMPointCloudIOU::setDebugFlag(bool flag) {
    _kdebug = flag;
}

void MMPointCloudIOU::updateLost(DataPacket* dp) {}

void MMPointCloudIOU::updateTracking(DataPacket* dp) {
    cv::Mat cv_xyz = dp->_mats["PointCloudImg"];
    _last_point_cloud = cv_xyz.clone();
    _last_pose = dp->_eigs["Tcw"];
    _t0 = dp->tframe;
}

void MMPointCloudIOU::updateTrackingUnmerged(DataPacket* dp) {
    if (!_hasRecovered) {
        _mapFinished = false;
        cv::Mat cv_xyz = dp->_mats["PointCloudImg"];
        _point_cloud_to_merge = cv_xyz.clone();
        _t1 = dp->tframe;
        recoverTransform();
        _unmerged_count = 1;
    }
    if (_hasRecovered) {
        _unmerged_count++;
        updateTracking(dp);
    }
}

void MMPointCloudIOU::updateLostUnmerged(DataPacket* dp) {
    _mapFinished = true;
}

bool MMPointCloudIOU::isMergeReady() {
   return _hasRecovered && (_mapFinished || _unmerged_count >= _max_unmerged);
}

void MMPointCloudIOU::getSampleTransforms(std::chrono::microseconds frame_to,
                                          std::chrono::microseconds frame_from,
                                          std::vector<Eigen::Vector3f>& sample) {
    float t0 = 1e-6*static_cast<float>(frame_to.count());
    float t1 = 1e-6*static_cast<float>(frame_from.count());
    float dt = t1 - t0;
    std::cout << "dt = " << dt << std::endl;

    float angle_max = _manglv*dt;
    angle_max = std::min((float) M_PI, angle_max);

    float lin_max = _mlinv*dt;
    lin_max = std::min(_max_radius, lin_max);

    std::cout << "max angle: " << angle_max << std::endl;
    std::cout << "max linear: " << lin_max << std::endl;

    for (size_t i = 0; i < _nsamples; i++) {
        float r = lin_max*(2*_h1[i] - 1);
        float theta = angle_max*(2*_h2[i] - 1);
        float phi = angle_max*(2*_h3[i] - 1);

        float x = r * std::sin(theta);
        float z = r * std::cos(theta);
        sample.push_back(Eigen::Vector3f(x,z,phi));
    }
}

float MMPointCloudIOU::getIOU(Eigen::MatrixX4f& test, NFPC& dst) {
    size_t matched = 0;
    size_t src_unmatched = test.rows();
    size_t dst_unmatched = dst.kdtree_get_point_count();
    std::unordered_set<Eigen::Index> seen;
    
    for (size_t i = 0; i < test.rows(); i++) {
        const float query_pt[3] = {test(i,0),test(i,1),test(i,2)};
        std::vector<std::pair<Eigen::Index, float>> ret_matches;
        nanoflann::SearchParams params;
        const size_t nMatches = dst.index->radiusSearch(
            &query_pt[0], _near_thr, ret_matches, params);
        if (nMatches > 0) {
            matched++;
            src_unmatched--;
            for (size_t j = 0; j < nMatches; j++) {
                if (seen.find(ret_matches[j].first) == seen.end()) {
                    seen.insert(ret_matches[j].first);
                    dst_unmatched--;
                }
            }
        }
    }

    return static_cast<float>(matched) / static_cast<float>(matched + src_unmatched + dst_unmatched);
}

void MMPointCloudIOU::showDebugMap(Eigen::MatrixX4f& src,
                                   NFPC& dst) {
    OccupationMap2D src_map(1000, 1000);
    OccupationMap2D dst_map(1000, 1000);
    src_map.SetResolution(0.01);
    dst_map.SetResolution(0.01);
    src_map.CenterMap();
    dst_map.CenterMap();

    for (size_t i = 0; i < src.rows(); i++) {
        float y = src(i,1);
        if (y < -0.2 || y > 1.0) {
            continue;
        }
        float x = src(i,0);
        float z = src(i,2);
        if (src_map.isInBounds(z,x)) {
            src_map.SetValue(z,x,OCCUPIED);
        }
    }

    for (size_t i = 0; i < dst.kdtree_get_point_count(); i++) {
        float y = dst.kdtree_get_pt(i, 1);
        if (y < -0.2 || y > 1.0) {
            continue;
        }
        float x = dst.kdtree_get_pt(i, 0);
        float z = dst.kdtree_get_pt(i, 2);
        if (dst_map.isInBounds(z,x)) {
            // std::cout << "projecting to point: " << z << ", " << x << std::endl;
            dst_map.SetValue(z,x,OCCUPIED);
        }
    }

    cv::Mat rchannel(1000, 1000, CV_8UC1, dst_map.GetBuffer());
    // cv::Mat rchannel(1000, 1000, CV_8UC1, cv::Scalar(0));
    cv::Mat gchannel(1000, 1000, CV_8UC1, cv::Scalar(0));
    // cv::Mat bchannel(1000, 1000, CV_8UC1, cv::Scalar(0));
    cv::Mat bchannel(1000, 1000, CV_8UC1, src_map.GetBuffer());

    cv::threshold(rchannel, rchannel, 127, 255, CV_8UC1);
    cv::threshold(bchannel, bchannel, 127, 255, CV_8UC1);

    vector<cv::Mat> channels;
    channels.push_back(bchannel);
    channels.push_back(gchannel);
    channels.push_back(rchannel);

    cv::Mat display;
    cv::merge(channels, display);
    cv::imshow("map", display);
    cv::waitKey(0);
}

Eigen::Matrix4f MMPointCloudIOU::CompassSearch(Eigen::Vector3f& init,
                                               float init_iou,
                                               Eigen::Vector3f& init_step,
                                               Eigen::MatrixX4f& src,
                                               NFPC& dst) {
    if (_kdebug) {
        showDebugMap(src, dst);
    }
    float scores[7] = {init_iou, -1, -1, -1, -1, -1, -1};
    Eigen::Vector3f step = init_step;
    Eigen::Vector3f center = init;

    while (std::fabs(step[0]) > _lin_step_min
        && std::fabs(step[1]) > _lin_step_min
        && std::fabs(step[2]) > _ang_step_min) {
        std::cout << "center point: " << center.transpose() << ", iou: " << scores[0] << std::endl;
        std::cout << "step: " << step.transpose() << std::endl;
        std::vector<Eigen::Vector3f> candidates;
        for(size_t i = 0; i < 3; i++) {
            if (scores[2*i+1] < 0) {
                Eigen::Vector3f test = center;
                test[i] += step[i];
                Eigen::Matrix4f trans = SE2toMat(test);
                Eigen::MatrixX4f pc = src * trans.transpose();
                scores[2*i+1] = getIOU(pc, dst);
            }
            if (scores[2*i+2] < 0) {
                Eigen::Vector3f test = center;
                test[i] -= step[i];
                Eigen::Matrix4f trans = SE2toMat(test);
                Eigen::MatrixX4f pc = src * trans.transpose();
                scores[2*i+2] = getIOU(pc, dst);
            }
        }
        // std::cout << "iou scores: " << scores[0] << ", "
        //                             << scores[1] << ", "
        //                             << scores[2] << ", "
        //                             << scores[3] << ", "
        //                             << scores[4] << ", "
        //                             << scores[5] << ", "
        //                             << scores[6] << std::endl;

        float best_score = scores[0];
        uint8_t best_idx = 0;
        for (uint8_t i = 1; i < 7; i++) {
            if (scores[i] > best_score) {
                best_score = scores[i];
                best_idx = i;
            }
        }

        uint8_t exclude;
        float old_score = scores[0];
        scores[0] = best_score;
        if (best_idx == 0) {
            step *= _step_decay;
            exclude = 0;
        } else {
            uint8_t axis = (best_idx - 1)/2;
            if (best_idx % 2 == 1) {
                scores[best_idx+1] = old_score;
                exclude = best_idx+1;
                center[axis] += step[axis];
            } else if (best_idx % 2 == 0) {
                scores[best_idx-1] = old_score;
                exclude = best_idx-1;
                center[axis] -= step[axis];
            }
        }
        if (fabs(old_score - best_score) < 0.002 && exclude != 0) {
            break;
        }
        for (uint8_t i = 1; i < 7; i++) {
            if (i != exclude) {
                scores[i] = -1;
            }
        }
    }

    if (_kdebug) {
        Eigen::Matrix4f trans = SE2toMat(center);
        Eigen::MatrixX4f pc = src * trans.transpose();
        showDebugMap(pc, dst);
    }
    return SE2toMat(center);
}

void MMPointCloudIOU::recoverTransform() {
    Eigen::MatrixX4f src, dst;
    cv_xyzToEig(_last_point_cloud, src);
    cv_xyzToEig(_point_cloud_to_merge, dst);
    Eigen::MatrixX3f dst_const = dst.block(0,0,dst.rows(),3);
    // std::cout << "dst_const shape: " << dst_const.rows() << "x" << dst_const.cols() << std::endl;
    NFPC nano_dst(3, std::cref(dst_const));
    // std::cout << "nano_dst initialized\n";
    // NFPC nano_dst(4, std::cref(dst));
    nano_dst.index->buildIndex();

    std::vector<Eigen::Vector3f> sample;
    getSampleTransforms(_t0, _t1, sample);
    // std::cout << "got sample transforms\n";
    size_t best_idx;
    float best_iou = 0;
    Eigen::Vector3f mean(0,0,0);
    float iou_sum = 0;
    std::vector<float> ious;
    for (size_t i = 0; i < _nsamples; i++) {
        Eigen::Matrix4f trans = SE2toMat(sample[i]);
        Eigen::MatrixX4f test = src*trans.transpose();
        float iou = getIOU(test, nano_dst);
        // std::cout << "SE2 vec " << sample[i].transpose() << " has iou score: " << iou << std::endl;
        if (iou > best_iou) {
            best_idx = i;
            best_iou = iou;
        }
        mean += iou*sample[i];
        iou_sum += iou;
        ious.push_back(iou);
    }
    mean = mean / iou_sum;

    Eigen::Vector3f var(0,0,0);
    for (size_t i = 0; i < _nsamples; i++) {
        Eigen::Vector3f diff = (sample[i] - mean)*ious[i];
        Eigen::Vector3f diffsq(diff[0]*diff[0],
                               diff[1]*diff[1],
                               diff[2]*diff[2]);
        var += diffsq;
    }
    var = var / (iou_sum*iou_sum);

    Eigen::Vector3f init_trans = sample[best_idx];
    Eigen::Vector3f init_step(std::sqrt(var[0]),
                              std::sqrt(var[1]),
                              std::sqrt(var[2]));
    // std::cout << "initial transform: " << init_trans.transpose() << std::endl;
    // std::cout << "initial step: " << init_step.transpose() << std::endl;
    Eigen::Matrix4f dst2src = CompassSearch(init_trans, best_iou, init_step, src, nano_dst);
    std::cout << "found relative pose:\n" << dst2src << std::endl;
    std::cout << "source frame pose:\n" << _last_pose << std::endl;
    // Eigen::Matrix4f result = _last_pose * src2dst;
    // std::cout << "resulting pose:\n" << result.inverse() << std::endl;
    // _last_pose gives src <- origin transform
    // dst2src gives src <- dst transform
    // return new map origin <- dst transform
    _recovered_pose = dst2src*_last_pose;
    std::cout << "recovered pose:\n" << _recovered_pose << std::endl;
    _hasRecovered = true;
}

Eigen::Matrix4f MMPointCloudIOU::getTransform() {
    _hasRecovered = false;
    std::cout << "returning transform:\n" << _recovered_pose << std::endl;
    return _recovered_pose;
}

void MMPointCloudIOU::cv_xyzToEig(cv::Mat cv_xyz, Eigen::MatrixX4f& result) {
    size_t h = cv_xyz.rows;
    size_t w = cv_xyz.cols;
    std::cout << "xyz shape: " << h << "x" << w << std::endl;
    cv::Mat cv_flat_cloud = cv_xyz.reshape(1,h*w);
    std::cout << "flattened shape: " << cv_flat_cloud.rows << "x" << cv_flat_cloud.cols << std::endl;
    const int type = cv_flat_cloud.type();
    cv::Mat cv_float_cloud;
    if (type == CV_16S) {
        cv::Mat cv_filtered_cloud;
        FilterByRow<int16_t>(cv_flat_cloud, cv_filtered_cloud, _dmin, _dmax);
        cv_filtered_cloud.convertTo(cv_float_cloud, CV_32FC1, 1e-3);
    } else if (type == CV_32F) {
        FilterByRow<float>(cv_flat_cloud,
                            cv_float_cloud,
                            1e-3*static_cast<float>(_dmin),
                            1e-3*static_cast<float>(_dmax));
    } else {
        std::cerr << "cv::Mat type " << type << " not supported for point cloud merging\n";
        return;
    }
    cv::Mat augment = cv::Mat::ones(cv_float_cloud.rows, 1, CV_32FC1);
    cv::hconcat(cv_float_cloud, augment, cv_float_cloud);
    std::cout << "augmented shape: " << cv_float_cloud.rows << "x" << cv_float_cloud.cols << std::endl;
    Eigen::MatrixXf temp;
    cv::cv2eigen(cv_float_cloud, temp);
    result = temp;
}

Eigen::Matrix4f MMPointCloudIOU::SE2toMat(Eigen::Vector3f& se2) {
    Eigen::Vector3f t(se2[0], 0, se2[1]);
    Eigen::Matrix3f R = Eigen::AngleAxisf(se2[2], Eigen::Vector3f::UnitY()).matrix();
    Eigen::Matrix4f trans = Eigen::MatrixXf::Identity(4,4);
    trans.block<3,3>(0,0) = R;
    trans.block<3,1>(0,3) = t;

    return trans.inverse();
}
