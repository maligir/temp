from nis import match
import cv2
import numpy as np
import bisect
import queue
import pandas
from collections import Counter
from bisect import bisect_left ,bisect
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from sympy import true
from traitlets import observe
from plot_map import CameraPoses2MapFrame, drawPoints, drawArrows, getMapImg, getFramePosesInMapFrame, ResizeWithAspectRatio

global valid_rooms

def angleDiff(angle):
    sign = np.sign(angle)
    while (np.sum(np.abs(angle) > np.pi) > 0):
        angle = angle - 2*sign*(np.abs(angle) > np.pi)*np.pi
    return angle

def read_rooms():
    global valid_rooms
    frooms = open('utils/valid_rooms.txt', 'r')
    for line in frooms:
        valid_rooms += [line.strip()]
    valid_rooms.sort()

def isLabelValid(label):
    global valid_rooms
    return bisect_left(valid_rooms, label)!=bisect(valid_rooms, label)

def parse_obsrv(line, verify_label=True):
    lsplit = line.split(',')
    label = lsplit[0]
    if (verify_label):
        isValid = isLabelValid(label)
        if (not isValid):
            label = label[:1] + '.' + label[1:]
            isValid = isLabelValid(label)
        if (not isValid):
            label = '-'
    
    conf = float(lsplit[1])
    tstamp = int(lsplit[2])
    center = np.array(lsplit[3].split()).astype(np.float32)
    quat = np.array(lsplit[4:]).astype(np.float32)

    return label, tstamp, conf, center, quat

def parse_frame(line):
    lsplit = line.split(',')
    fid = int(lsplit[0])
    tstamp = int(1e6*float(lsplit[1])+0.5)
    center = np.array(lsplit[2:5]).astype(np.float32)
    quat = np.array(lsplit[5:])
    R = Rotation.from_quat(quat).as_matrix()

    return fid, tstamp, center, R


def isEquivelant(obsv1, obsv2, project = True):
    dist = 0
    if project:
        dist = np.linalg.norm(obsv1[3][:2] - obsv2[3][:2])
    else:
        dist = np.linalg.norm(obsv1[3] - obsv2[3])
    cosd = obsv1[4].dot(obsv2[4])

    return dist < 0.04 and cosd > .9

def group_observations(obsrvs):
    placard_ids = []
    count_unique = 0
    for i in range (len(obsrvs)):
        obsrvi = obsrvs[i]
        for j in range(i):
            obsrvj = obsrvs[j]
            if isEquivelant(obsrvi, obsrvj):
                placard_ids += [placard_ids[j]]
                break
        if len(placard_ids) == i:
            count_unique += 1
            placard_ids += [count_unique]
    return np.array(placard_ids), count_unique

def condense_poses(poses, quats, map_width, labels, times):
    # map idx: (count, x, y, [label list], [times list], [theta list])
    poses_dict = {}
    for i in range(len(poses)):
        idx = map_width*poses[i,0] + poses[i,1]
        theta = np.arctan2(2*quats[i,2]*quats[i,3], 1-2*quats[i,2]**2)
        if (poses_dict.get(idx) == None):
            poses_dict[idx] = (1,poses[i,0],poses[i,1],[labels[i]],[times[i]], [theta])
        else:
            poses_dict[idx] = (poses_dict[idx][0]+1,
                               poses[i,0],
                               poses[i,1],
                               poses_dict[idx][3] + [labels[i]],
                               poses_dict[idx][4] + [times[i]],
                               poses_dict[idx][5] + [theta])
    return poses_dict

def filter_by_count(hist, ids, obsrvs, thresh=5):
    for i in range(1,len(hist)+1):
        if hist[i-1] < thresh:
            indxs = np.where(ids == i)
            ids = np.delete(ids, indxs)
            obsrvs = np.delete(obsrvs, indxs, axis=0)
            hist[i-1] = 0
    return hist, ids, obsrvs

def filter_by_wall(poses_dict, cv_map, r=1):
    poses_filtered = {}
    for (key, value) in poses_dict.items():
        bbxmin = max(value[1] - r, 0)
        bbxmax = min(value[1] + r + 1, cv_map.shape[0])
        bbymin = max(value[2] - r, 0)
        bbymax = min(value[2] + r + 1, cv_map.shape[1])
        if bbxmin >= bbxmax or bbymin >= bbymax:
            continue
        # if (cv_map[bbxmin:bbxmax,bbymin:bbymax] == []):
        try: 
            occupation = np.min(cv_map[bbxmin:bbxmax,bbymin:bbymax])
        except:
            print("bounding box: [{}:{},{}:{}]".format(bbxmin,bbxmax,bbymin,bbymax))
            continue
        if occupation < 100:
            poses_filtered[key] = value
    return poses_filtered

def filter_by_adjacency(poses_dict, map_shape, r=4):
    blobs = {}
    idxs_seen = []
    for (key, _) in poses_dict.items():
        if key in idxs_seen:
            continue
        seen = set()
        q = queue.Queue()
        q.put(key)
        blob = []
        while q.qsize() != 0:
            center_idx = q.get()
            if (center_idx in seen):
                continue
            seen.add(center_idx)
            if center_idx not in poses_dict:
                continue
            value = poses_dict[center_idx]
            blob += [center_idx]
            idxs_seen += [center_idx]
            xmin = max(value[1] - r, 0)
            xmax = min(value[1] + r + 1, map_shape[0])
            ymin = max(value[2] - r, 0)
            ymax = min(value[2] + r + 1, map_shape[1])
            for x in range(xmin, xmax):
                for y in range(ymin, ymax):
                    if x == value[1] and y == value[2]:
                        continue
                    q.put(map_shape[1]*x+y)
        blobs[len(blobs)] = blob
    
    poses_filtered = {}
    for blob in blobs.values():
        csum = 0
        xsum = 0
        ysum = 0
        labels = []
        times = []
        theta_sum = 0
        for idx in blob:
            c = poses_dict[idx][0]
            csum += c
            xsum += c*poses_dict[idx][1]
            ysum += c*poses_dict[idx][2]
            labels += poses_dict[idx][3]
            times += poses_dict[idx][4]
            theta_sum += np.sum(poses_dict[idx][5])
        x = np.round((xsum/csum) + 0.5).astype(np.uint16)
        y = np.round((ysum/csum) + 0.5).astype(np.uint16)
        theta = theta_sum/csum
        poses_filtered[x*map_shape[1]+y] = (csum, x, y, labels, times, theta)
    return poses_filtered

def choose_label(poses_dict):
    for (key, value) in poses_dict.items():
        labels = list(filter(lambda x : x != '-', value[3]))
        if len(labels) == 0:
            poses_dict[key] = value[0:3] + ('-',) + (value[4],) + (value[5],)
            continue
        counts = Counter(labels)
        label = counts.most_common(1)[0]
        poses_dict[key] = value[0:3] + (label[0],) + (value[4],) + (value[5],)
    return

def select_times(poses):
    for i in range(poses.shape[0]):
    # i = 31
    # if True:
        times = poses[i,4]
        times.sort()
        diffs = np.diff(times)
        groups = []
        group = []
        for j in range(len(times)):
            group += [times[j]]
            if j == len(times) - 1:
                groups += [group]
                break
            if diffs[j] > 1e6:
                groups += [group]
                group = []
        for j in range(len(groups)):
            group = groups[j]
            mid_idx = len(group) // 2
            mid_time = group[mid_idx]
            groups[j] = mid_time
        poses[i,4] = groups

def find_keyframes_at_time(sorted_kfs, kf_times, sorted_times):
    kfs = []
    i = 0
    for time in sorted_times:
        while (kf_times[i] < 1e-6*time):
            i += 1
            continue
        kfs += [sorted_kfs[i,:]]
    return kfs

def plot_data(fobsrv, fframe, fmap, res, offset):
    obsrvs = []
    for line in fobsrv:
        parsed = np.array(parse_obsrv(line), dtype=object)
        obsrvs += [parsed]
    obsrvs = np.array(list(filter(lambda x : x[2] > 0.9, obsrvs)))
    # obsrvs = np.array(list(filter(lambda x : x[0] != '-', obsrvs)))
    # dists = np.array(list(map(lambda x : np.linalg.norm(x[3]), np.array(obsrvs))))
    obsrvs = np.array(list(obsrvs))
    times = np.stack(np.array(obsrvs)[:,1])
    poses = np.stack(np.array(obsrvs)[:,3])
    quats = np.stack(np.array(obsrvs)[:,4])
    poses = CameraPoses2MapFrame(poses, np.array(offset), res, False)

    cv_map = getMapImg(fmap)
    poses_dict = condense_poses(poses, quats, cv_map.shape[1], obsrvs[:,0], obsrvs[:,1])
    # poses_filtered = poses_dict # placeholder line while below is commented out
    poses_filtered = filter_by_wall(poses_dict, cv_map, 0)
    poses_filtered = filter_by_adjacency(poses_filtered, cv_map.shape, round(0.151/res))
    poses_dict = filter_by_adjacency(poses_dict, cv_map.shape, round(0.151/res))
    choose_label(poses_filtered)
    poses_filtered = np.array(list(filter(lambda x : int(x[0]) > 0, list(poses_filtered.values()))), dtype=object)
    poses_filtered[:,0:3] = poses_filtered[:,0:3].astype(np.uint16)
    poses_dict = list(filter(lambda x : int(x[0]) > 0, list(poses_dict.values())))
    poses_dict.sort(key=lambda x: x[4])
    poses_dict = np.array(poses_dict, dtype=object)
    poses_dict[:,0:3] = poses_dict[:,0:3].astype(np.uint16)
    select_times(poses_filtered)
    # poses_condensed = np.array(list(poses_filtered.values()))

    kf_poses, kf_times = getFramePosesInMapFrame(fframe.name, offset, res)
    cv_map = drawPoints(cv_map, kf_poses, color=(0,0,255))
    cv_map = drawPoints(cv_map, poses_dict[:,1:], color=(0,165,255), sizes=7)
    cv_map = drawArrows(cv_map, poses_dict[:,1:], poses_dict[:,5], color=(0,165,255))
    cv_map = drawPoints(cv_map, poses_filtered[:,1:], color=(255,0,0), sizes=10)
    cv_map = drawArrows(cv_map, poses_filtered[:,1:], poses_filtered[:,5], color=(255,0,0))
    # cv2.imwrite("map.jpg", cv_map)

    # cv_copy = np.copy(cv_map)
    # cv_copy = ResizeWithAspectRatio(cv_copy, width=720)
    # cv2.imshow("map", cv_copy)
    # cv2.waitKey(0)

    for i in range(poses_filtered.shape[0]):
        cv_copy = np.copy(cv_map)
        cv_copy = drawPoints(cv_copy, np.array([poses_filtered[i,1:]]), color=(0,255,0), sizes=5)

        kfs = find_keyframes_at_time(kf_poses, kf_times, np.sort(poses_filtered[i,4]))
        cv_copy = drawPoints(cv_copy, np.array(kfs), color=(0,255,255), sizes=5)

        cv_copy = ResizeWithAspectRatio(cv_copy, width=720)

        print(poses_filtered[i,:])
        # print("Label: {}; times: {}".format(poses_filtered[i,3], poses_filtered[i,4]))

        cv2.imshow("map", cv_copy)
        cv2.waitKey(0)

    # plt.scatter(poses[:,0], poses[:,1])
    # plt.plot(frame_dists)
    # plt.show()

    # histo = np.histogram(ids, bins=nunique)
    # hist, ids, valid_obsrvs = filter_by_count(histo[0], ids, valid_obsrvs)
    # nunique = np.sum(hist > 0)
    # print("There are {} unique placards with minimum observations.".format(nunique))
    return poses_filtered, cv_map

def test_plot(fobsrv, fframe, fmap, res, offset):
    tstamp = 344214577

    frame_data = parse_frame(fframe.readline())
    obsrv_data = np.array(parse_obsrv(fobsrv.readline()), dtype=object)


    while (frame_data[1] < tstamp):
        frame_data = parse_frame(fframe.readline())
    while (obsrv_data[1] < tstamp):
        obsrv_data = np.array(parse_obsrv(fobsrv.readline()), dtype=object)

    print(frame_data[2])
    print(obsrv_data[3])
    # obsrv_data[3] = frame_data[3].dot(obsrv_data[3]) + frame_data[2]
    # obsrv_data[4] = frame_data[3].dot(obsrv_data[4])

    # obsrv_point = np.array([[obsrv_data[3][0], obsrv_data[3][1]]], dtype=np.float32)
    kf_pose = CameraPoses2MapFrame(np.array([frame_data[2]]), offset, res)
    obsrv_point = CameraPoses2MapFrame(np.array([obsrv_data[3]]), offset, res, False)
    # obsrv_point = CameraPoses2MapFrame(np.array([[0.642264,  1.35041, 0.153103]]), offset, res, False)
    origin = CameraPoses2MapFrame(np.array([[0,0,0]]), offset, res)
    print(kf_pose)
    print(obsrv_point)
    cv_map = getMapImg(fmap)
    cv_map = drawPoints(cv_map, origin, (0,0,255), size=10)
    cv_map = drawPoints(cv_map, kf_pose, (0,0,255))
    cv_map = drawPoints(cv_map, obsrv_point, (255,0,0))

    cv_mapS = ResizeWithAspectRatio(cv_map, width=720)

    cv2.imshow("map", cv_mapS)
    cv2.waitKey(0)

def plot_trial(trial, res, offset):
    # fnobsrv = "data/trials/trial{}/placard_observations.csv".format(trial)
    # fnkeyframe = "data/trials/trial{}/map1/keyframe_trajectory.csv".format(trial)
    fnobsrv = "data/trial{}/placard_observations.csv".format(trial)
    fnkeyframe = "data/trials/trial{}/map1/keyframe_trajectory.csv".format(trial)
    fnoccupation = "data/trial{}/occupation.csv".format(trial)

    poses, cv_map = plot_data(open(fnobsrv), open(fnkeyframe), fnoccupation, res, offset)

    print("There are {} unique placards in trial {}.".format(poses.shape[0], trial))
    np.savetxt("placards_trial{}.csv".format(trial), poses[:,[0,1,2,3,5]], fmt=['%u','%u','%u','%s','%.4f'], delimiter=',')
    cv2.imwrite("Map{}.jpg".format(trial), cv_map)

def get_baseline_metrics(trial, plot=False):
    fplacards = "data/placards/placards_trial{}.csv".format(trial)
    placards = np.genfromtxt(fplacards, delimiter=',',
                             dtype=[('count','u2'),('x','u2'),('y','u2'),('label', 'U5'),('theta', 'f4')])
    _, bin_edges = np.histogram(placards['theta'], 64, range=(-np.pi,np.pi))
    filtered = list(filter(lambda x : x > 0 and x < np.pi/8, placards['theta']))
    orig = np.sum(filtered)/len(filtered)
    valid_angles = [orig, orig - np.pi, orig + np.pi/2, orig - np.pi/2]
    if plot:
        plt.hist(placards['theta'], bins=bin_edges)
        plt.vlines(valid_angles[0], 0, 10, colors=(0,1,0))
        plt.vlines(valid_angles[1:], [0,0,0], [10,10,10], colors=[(1,0,0),(1,0,0),(1,0,0)])
        plt.xlabel("Placard Normal Angle (radians)")
        plt.ylabel("Placard Count")
        plt.title("Angle Clustering to Determine Baseline")
        plt.legend(['determined baseline', 'predicted clusters', 'observed clusters'])
        plt.show()
    true_labels = ['MEN','WOMEN','2.102','2.108','2.106','2.104','2.128','2.124','2.118','2.124',
                   '2.120','2.122','2.308','2.304','2.306','STAIR','2.328','2.326','2.304','2.324',
                   '2.322','2.320','2.318','2.304','2.304','STAIR','2.202','2.204','2.206','2.208',
                   '2.202','2.204','2.204','2.202']
    true_angle_ids = [0,0,1,0,0,0,0,0,1,0,0,0,0,2,0,3,3,3,2,3,3,3,3,2,2,2,3,2,2,2,3,1,2,1]
    baseline_poses = np.vstack((placards['x'],placards['y']))
    return np.array(valid_angles), true_labels, true_angle_ids, baseline_poses

def get_matchings(trial):
    if trial == 1:
        return np.array([14,15,13,16,0,17,18,19,20,
                         21,22,23,21,19,18,16,11,
                         10,9,8,7,6,5,4,3,2,1,26,
                         27,28,29,30,31,32,28,34]) - 1
    elif trial == 2:
        return np.array([1, 2, 3, 5, 6, 9, 8,10,
                        12,13,14,15,16,17,18,20,
                        21,22,23,25,20,19,16,11,10,
                         9, 8, 7, 6, 5, 4, 3, 2,
                         1,26,27,28,29,30,31,32,
                         28,34]) - 1
    elif trial == 3:
        return np.array([1, 2, 3, 4, 5, 6, 7, 8, 9,10,
                        11,12,13,14,15,16,17,18,19,20,
                        21,22,23,25,25,26,27,28,29,30,
                        31,32,28,34]) - 1
    elif trial == 4:
        return np.array([1, 2, 3, 4, 5, 6, 7, 9, 8,
                        12,13,14,15,16, 0,17,18,20,
                        21,22,23,19,11,10,26,27,28,
                        29,30,31,29,32,28,34]) - 1

def get_placard_matchings(trial):
    matching = get_matchings(trial).astype(int)

    fplacards = "data/placards/placards_trial{}.csv".format(trial)
    placards = np.genfromtxt(fplacards, delimiter=',',
                             dtype=[('count','u2'),('x','u2'),('y','u2'),('label', 'U5'),('theta', 'f4')])
    placards = placards[matching>=0]
    matching = matching[matching>=0]

    return placards, matching

def get_metrics(trial, baseline_packet):
    placards, matching = get_placard_matchings(trial)
    valid_angles = baseline_packet[0]

    # print(baseline_packet[1])
    true_labels = np.array(baseline_packet[1])[matching]
    true_angles = np.array(baseline_packet[2])[matching]
    true_angles = valid_angles[true_angles]
    true_poses = np.array(baseline_packet[3])[:,matching]

    dtheta = np.abs(angleDiff(true_angles-placards['theta']))
    dtheta_mean = np.mean(dtheta)
    dtheta_sig = np.std(dtheta)

    poses = np.vstack((placards['x'],placards['y']))
    dpose = true_poses - poses
    sqdist = np.sum(np.square(dpose), axis=0)
    dist_mean = np.mean(0.038*np.sqrt(sqdist))
    dist_std = np.std(0.038*np.sqrt(sqdist))

    ncLabels = np.sum(true_labels == placards['label'])
    nObsrv = len(matching)
    nUnique = len(np.unique(matching))
    print("Trial{} has {} correct labels of {} distinct observations.".format(trial, ncLabels, nObsrv))
    print("There are {} unique placards observed with {} missing from observation and {} duplicates".format(nUnique, 34-nUnique, nObsrv - nUnique))
    print("The Placard orientation error is {0:.3f}+/-{1:.3f} radians ({2:.3f}+/-{3:.3f} degrees) on average".format(dtheta_mean, dtheta_sig, 180*dtheta_mean/np.pi, 180*dtheta_sig/np.pi))
    print("The Placard displacement error is {0:.3f}+/-{1:.3f} meters on average".format(dist_mean, dist_std))

    # diff_mat = valid_angles[np.newaxis,:] - angles[:,np.newaxis]
    # diff_mat = angleDiff(diff_mat)
    # assg = np.argmin(np.abs(diff_mat), axis=1)
    # print(angles)
    # print(assg)

def get_errors_dists(trial, baseline_packet):
    placards, matching = get_placard_matchings(trial)
    true_poses = np.array(baseline_packet[3])[:,matching]
    poses = np.vstack((placards['x'],placards['y']))
    dpose = true_poses - poses
    errors = np.sqrt(np.sum(np.square(dpose), axis=0))*0.038
    dists = np.sqrt(np.sum(np.square(true_poses), axis=0))*0.038
    return errors, dists

def plot_errors(baseline_packet):
    err1, dists1 = get_errors_dists(1, baseline_packet)
    err2, dists2 = get_errors_dists(2, baseline_packet)
    err4, dists4 = get_errors_dists(4, baseline_packet)

    plt.scatter(dists1, err1)
    plt.scatter(dists2, err2)
    plt.scatter(dists4, err4)
    plt.xlabel("Distance from Origin (m)")
    plt.ylabel("Localization Error (m)")
    plt.legend(['Trial 1', 'Trial 2', 'Trial 4'])
    plt.show()

if __name__ == "__main__":
    valid_rooms = []
    read_rooms()

    # plot_trial(1, 0.0381623, np.array([-33.108, -71.6745]))
    # plot_trial(2, 0.0380047, np.array([-33.2835, -71.1495]))
    # plot_trial(3, 0.037989, np.array([-33.159, -71.118]))
    # plot_trial(4, 0.0379575, np.array([-33.183, -71.115]))
    # plot_trial(5, 0.0380047, np.array([-33.2835, -71.1495]))
    # plot_trial(6, 0.0332167, np.array([-7.9455, -41.3835]))
    # plot_trial(7, 0.0239873, np.array([-40.8795, -30.7545]))
    # plot_trial(8, 0.039753, np.array([-47.5965, -76.506]))
    # plot_trial(9, 0.0225855, np.array([-39.321, -37.545]))
    # plot_trial(10, 0.029295, np.array([-39.63, -36.9975]))
    # plot_trial(11, 0.0342562, np.array([-41.706, -62.6925]))
    # plot_trial(12, 0.023499, np.array([-38.868, -37.806]))

    baseline = get_baseline_metrics(3)
    # get_metrics(1, baseline)
    # get_metrics(2, baseline)
    # get_metrics(3, baseline)
    # get_metrics(4, baseline)

    plot_errors(baseline)
