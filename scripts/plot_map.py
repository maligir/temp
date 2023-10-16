import numpy as np
import numbers
import matplotlib.pyplot as plt
import cv2

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

def getMapImg(fmap):
  data = 255 - np.loadtxt(open(fmap, "rb"), delimiter=",").astype(np.uint8)
  return cv2.cvtColor(data,cv2.COLOR_GRAY2BGR)

def drawPoints(cv_map, poses, color=(0,0,0), sizes=5):
  variateSize = False
  if isinstance(sizes, numbers.Number):
    size = min(sizes, 10)
  elif len(sizes) != len(poses):
    size = min(sizes[0],10)
  else:
    variateSize = True
  for i in range(len(poses)):
    if variateSize:
      size = min(sizes[i], 10)
    cv_map = cv2.circle(cv_map, (poses[i,1], poses[i,0]), size, color, -1)
  return cv_map

def drawArrows(cv_map, poses, thetas, color=(0,0,0), length=25):
  for i in range(len(thetas)):
    theta = thetas[i]
    direct = np.array([np.sin(theta), np.cos(theta)])
    startpt = np.array([poses[i,1], poses[i,0]])
    endpt = np.round(startpt+length*direct).astype(np.int)
    cv_map = cv2.arrowedLine(cv_map, startpt, endpt, color=color, thickness=5)
  return cv_map

def CameraPoses2MapFrame(poses, offset, res, doFlip=True):
  if len(poses.shape) == 1:
    poses = np.expand_dims(poses, 0)

  if doFlip:
    axis_flip = np.array([[1, 0, 0],
                          [0, 0, 1],
                          [0,-1, 0]])
    poses = poses.dot(axis_flip.transpose())
  poses = np.floor((poses[:,0:2] - offset)/res + 0.5).astype(np.uint16)
  return poses

def getFramePosesInMapFrame(fframes, offset, res):
  keyframes = np.loadtxt(open(fframes, "rb"), delimiter=",").astype(np.float32)
  kf_poses = CameraPoses2MapFrame(keyframes[:,2:5], offset, res)
  times = keyframes[:,1]
  return kf_poses, times

def plotTrajectory(fmap, fframes, res, offset):
  cv_map = getMapImg(fmap)

  # data = 255 - np.loadtxt(open(fmap, "rb"), delimiter=",").astype(np.uint8)
  # plt.figure("Current")
  # plt.imshow(data, cmap='gray')
  # plt.show()

  kf_poses, _ = getFramePosesInMapFrame(fframes, offset, res)
  origin = np.floor((-offset)/res + 0.5).astype(np.uint16)

  print(origin)
  cv_map = drawPoints(cv_map, kf_poses, color=(0,0,255))
  cv_map = drawPoints(cv_map, np.array([origin]), color=(0,255,0))
  cv_mapS = ResizeWithAspectRatio(cv_map, width=720)

  cv2.imwrite("map.jpg", cv_map)

  cv2.imshow("map", cv_mapS)
  cv2.waitKey(0)

def test_mapping():
  poses = np.array([[13.2544,-65.0815],[15.1706,-64.9111],[15.2544,-65.0815],[15.906, -66.4778]])
  offset = np.array([-33.268, -71.123])
  res = 0.0380415
  print(CameraPoses2MapFrame(poses, offset, res, False))

if __name__ == "__main__":
  test_mapping()
  # plotTrajectory("data/trial0/occupation.csv", "data/trial0/map1/keyframe_trajectory.csv", 0.0381938, np.array([-33.189, -71.6175]))
  # plotTrajectory("data/trial1/occupation.csv", "data/trials/trial1/map1/keyframe_trajectory.csv", 0.014931, np.array([-21.7305, -2.502]))
  # plotTrajectory("data/trial2/occupation.csv", "data/trials/trial2/map1/keyframe_trajectory.csv", 0.03801, np.array([-33.309, -71.16]))
  # plotTrajectory("data/trial3/occupation.csv", "data/trials/trial3/map1/keyframe_trajectory.csv", 0.0380415, np.array([-33.268, -71.123]))
  # plotTrajectory("data/trial4/occupation.csv", "data/trial4/map1/keyframe_trajectory.csv", 0.03801, np.array([-33.309, -71.16]))
