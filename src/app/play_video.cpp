#include "System.h"
#include "settings.h"

#include <eigen3/Eigen/Core>
#include <unistd.h>

#include <boost/filesystem.hpp>

// #include "map_k4a.hpp"
// #include "map_ros.hpp"
#include "orbslam_utils.h"

void runORBSLAM(Settings& settings, ORB_SLAM3::System* SLAM) {
    std::string sensor;
    settings.getRequiredParameter("Sensor.Type", sensor);

    if (sensor == std::string("K4A")) {
        // runK4AORBSLAM(settings, SLAM);
    } else if (sensor == std::string("ROS")) {
        // runROSORBSLAM(settings, SLAM);
    } else {
        std::cerr << "No implementation available for sensor type " << sensor << std::endl;
        exit(1);
    }
}

void saveMap(Settings& settings, ORB_SLAM3::System* SLAM, std::string tfile) {
    std::string sensor;
    settings.getRequiredParameter("Sensor.Type", sensor);

    if (sensor == std::string("K4A")) {
        // saveMapK4A(settings, SLAM, tfile);
    } else if (sensor == std::string("ROS")) {
        // saveMapROS(settings, SLAM, tfile);
    } else {
        std::cerr << "No implementation available for sensor type " << sensor << std::endl;
        exit(1);
    }
}

int main(int argc, char**argv) {
    std::cout << "This executable is deprecated, use either map_k4a or map_ros\n";
    return 1;

    if (argc < 2) {
        std::cerr << "\nUsage: " << argv[0]
                  << " path_to_settings\n";
        return 1;
    }
    Settings settings(argv[1]);

    settings.readSensorSettings();
    settings.readORBSettings();
    std::string orb_voc;
    std::string orb_set;
    settings.getRequiredParameter("ORB.Vocab", orb_voc);
    settings.getRequiredParameter("ORB.Settings", orb_set);

    ORB_SLAM3::System SLAM(orb_voc, orb_set, ORB_SLAM3::System::RGBD, true);
    runORBSLAM(settings, &SLAM);

    const std::vector<ORB_SLAM3::Map*>& maps = SLAM.GetAllMaps();
    std::cout << "There are " << maps.size() << " maps to process\n";

    std::string trial_dir;
    settings.getRequiredParameter("Save.Path", trial_dir);
    boost::filesystem::create_directories(trial_dir.c_str());

    for (size_t i = 0; i < maps.size(); i++) {
        std::vector<ORB_SLAM3::KeyFrame*> kfs = maps[i]->GetAllKeyFrames();
        sort(kfs.begin(),kfs.end(),ORB_SLAM3::KeyFrame::lId);
        const std::vector<ORB_SLAM3::MapPoint*>& mps = maps[i]->GetAllMapPoints();
        std::string map_dir = trial_dir + "/map" + std::to_string(i+1);
        boost::filesystem::create_directories(map_dir);
        std::cout << "map " << i+1 << " has " << kfs.size() << " keyframes and " << mps.size() << " map points\n";
        std::string mfile = map_dir + "/orb_map_points.csv";
        std::string kfile = map_dir + "/keyframe_trajectory.csv";
        std::string ofile = map_dir + "/octomap.bt";
        std::string fmap = map_dir + "/occupation.csv";
        std::string fplacs = map_dir + "/placards.csv";
        SaveMapToFile(mfile, mps);
        SaveKeyframeTrajectoryToFile(kfile, kfs);
    }

    if (maps.size() == 1) {
        std::string tfile = trial_dir + "/full_trajectory.csv\n";
        saveMap(settings, &SLAM, tfile);
    }

    SLAM.Shutdown();

    std::cout << "finished\n";
}