#include "placard_database.h"

#include <iostream>
#include <fstream>

PlacardDatabase::PlacardDatabase(float dist_thr,
                                 float angl_thr,
                                 std::string fappend): _dist_thr(dist_thr)
                                                       , _angl_thr(angl_thr)
                                                       , _fappend(fappend)
                                                       , _append(fappend != "") {}

size_t PlacardDatabase::ManualSearch(Placard& placard) {
    for (size_t i = 0; i < _avg.size(); i++) {
        Placard& avg = _avg[i];
        float dist = (avg.position_ - placard.position_).squaredNorm();
        Eigen::Quaternionf& q1 = placard.orientation_;
        Eigen::Quaternionf& q2 = avg.orientation_;
        float angl = q1.w()*q2.w()
                   + q1.x()*q2.x()
                   + q1.y()*q2.y()
                   + q1.z()*q2.z();
        if (dist <= _dist_thr*_dist_thr && angl >= _angl_thr) {
            return i;
        }
    }
    return _avg.size();
}

void PlacardDatabase::UpdateAvg(size_t idx) {
    std::vector<Placard> obsv_set = _placards[idx];
    Placard total;
    total.position_ = Eigen::Vector3f(0,0,0);
    total.orientation_.w() = 0;
    total.orientation_.x() = 0;
    total.orientation_.y() = 0;
    total.orientation_.z() = 0;

    float weight_tot = 0;

    for (size_t i = 0; i < obsv_set.size(); i++) {
        Placard& p = obsv_set[i];
        float w = p.conf_;
        if (p.text_ == "") {
            w /= 2;
        }
        weight_tot += w;
        total.position_ += w*p.position_;
        total.orientation_.w() += w*p.orientation_.w();
        total.orientation_.x() += w*p.orientation_.x();
        total.orientation_.y() += w*p.orientation_.y();
        total.orientation_.z() += w*p.orientation_.z();
    }

    _avg[idx].position_ = total.position_ / weight_tot;
    _avg[idx].orientation_ = total.orientation_.normalized();
    _avg[idx].conf_ = weight_tot / obsv_set.size();
    _lastUpdated[idx] = obsv_set.size();
}

void PlacardDatabase::AddPlacard(Placard& placard) {
    size_t idx;
    if (placard.text_ != "") {
        // placard has valid text
        auto iter = _label2idx.find(placard.text_);
        if (iter == _label2idx.end()) {
            // placard text not found in database
            idx = ManualSearch(placard);
            if (idx >= _placards.size()) {
                // placard did not match any unlabelled placards
                // create new placard
                _placards.push_back(std::vector<Placard>());
                _placards[idx].push_back(placard);
                _avg.push_back(placard);
                _lastUpdated.push_back(1);
                _label2idx[placard.text_] = idx;
            } else {
                // placard manually matched to unlabelled set
                // update set with label
                _placards[idx].push_back(placard);
                _avg[idx].text_ = placard.text_;
                _label2idx[placard.text_] = idx;
            }
        } else {
            // placard text identified in database
            // add placard to set
            idx = iter->second;
            _placards[idx].push_back(placard);
        }
    } else {
        // placard does not have valid text
        idx = ManualSearch(placard);
        if (idx >= _placards.size()) {
            _placards.push_back(std::vector<Placard>());
            _placards[idx].push_back(placard);
            _avg.push_back(placard);
            _lastUpdated.push_back(1);
        } else {
            // place placard in unlabelled set
            _placards[idx].push_back(placard);
        }
    }

    if (_placards[idx].size() - _lastUpdated[idx] > 10) {
        UpdateAvg(idx);
    }
    if (_append) {
        write_append(placard);
    }
}

void PlacardDatabase::write(std::string& fname) {
    std::ofstream fmap;
    fmap.open(fname);
    for (size_t i = 0; i < _avg.size(); i++) {
        UpdateAvg(i);
        std::string label = _avg[i].text_ == "" ? "-" : _avg[i].text_;
        fmap << label << ","
             << _avg[i].conf_ << ","
             << _avg[i].position_.transpose() << ","
             << _avg[i].orientation_.x() << ","
             << _avg[i].orientation_.y() << ","
             << _avg[i].orientation_.z() << ","
             << _avg[i].orientation_.w() << std::endl;
    }
    fmap.close();
}

void PlacardDatabase::write_all(std::string& fname) {
    std::ofstream fmap;
    fmap.open(fname);
    for (size_t j = 0; j < _placards.size(); j++) {
        std::vector<Placard>& observations = _placards[j];
        UpdateAvg(j);
        for (size_t i = 0; i < observations.size(); i++) {
            std::string label = observations[i].text_ == "" ? "-" : observations[i].text_;
            fmap << j << ","
                 << label << ","
                 << observations[i].conf_ << ","
                 << observations[i].obsv_time_.count() << ","
                 << observations[i].position_.transpose() << ","
                 << observations[i].orientation_.x() << ","
                 << observations[i].orientation_.y() << ","
                 << observations[i].orientation_.z() << ","
                 << observations[i].orientation_.w() << std::endl;
        }
    }
    fmap.close();
}

void PlacardDatabase::write_append(Placard& placard) {
    std::ofstream fmap;
    fmap.open(_fappend, std::ios::app);
    std::string label = placard.text_ == "" ? "-" : placard.text_;
    fmap << label << ","
         << placard.conf_ << ","
         << placard.obsv_time_.count() << ","
         << placard.position_.transpose() << ","
         << placard.orientation_.x() << ","
         << placard.orientation_.y() << ","
         << placard.orientation_.z() << ","
         << placard.orientation_.w() << std::endl;
    fmap.close();
}
