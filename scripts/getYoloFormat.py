
import os
import re
import sys
from os import path
import shutil
import random

import PIL
from PIL import Image
import numpy as np

#import caffe
#import surgery, score

import numpy as np

import glob

import skimage
from skimage import measure
from shutil import copyfile

import cv2

data_dir = './data/'
labeled_dir = data_dir + 'hand_labeled/All/'

def getTrainValSets():
  images = glob.glob(labeled_dir+'raw_images/*.jpg')
  random.shuffle(images)
  directory = data_dir + 'Yolo/data/'
  for i in range(len(images)):
    fname = images[i].split('/')[-1]
    prefix = fname.split('.')[0]
    if (i < np.floor(0.8*len(images))):
      f = open(data_dir+'Yolo/image_sets/train_real.txt', 'a+')
      subdr = 'train'
    else :
      f = open(data_dir+'Yolo/image_sets/val_real.txt', 'a+')
      subdr = 'valid'
    f.write(directory + fname + '\n')
    f.close()

    f = open(data_dir+'Yolo/image_sets/all_real.txt', 'a+')
    f.write(directory + fname + "\n")
    f.close()

    flabel = data_dir+'Yolo/all/labels/'+prefix+'.txt'
    fimg = data_dir+'Yolo/all/images/'+fname
    flabel_copy = data_dir+'Yolo/'+subdr+'/labels/'+prefix+'.txt'
    fimg_copy = data_dir+'Yolo/'+subdr+'/images/'+fname
    
    shutil.copyfile(flabel, flabel_copy)
    shutil.copyfile(fimg, fimg_copy)

def getBBoxLines(img_file):
  fname = img_file.split('/')[-1]
  img = cv2.imread(img_file)
  img_arr = np.expand_dims(np.asarray(img)[:,:,0], axis=2)
  img_arr = 255*(img_arr == 1).astype(np.uint8)
  label_mask = measure.label(img_arr)
  props = measure.regionprops(label_mask)
  
  lines = ''
  for prop in props:
    obj_id = prop.label
    minr, minc, null, maxr, maxc, null= prop.bbox
    x = (minc + maxc) / 2 / img.shape[1]
    y = (minr + maxr) / 2 / img.shape[0]
    w = (maxc - minc) / img.shape[1]
    h = (maxr - minr) / img.shape[0]
    if (h >= 1 and w >= 1):
        continue
    line = '{} {} {} {} {}\n'.format(0, x, y, w, h)
    lines = lines + line
  # classify everything else as non-placard
  # line = '{} {} {} {} {}\n'.format(0, 0.5, 0.5, 1, 1)
  # lines = lines + line
  return lines

def getYoloFormat():
  images = glob.glob(labeled_dir+'raw_images/*.jpg')
  for i in range(len(images)):
    fname = images[i].split('/')[-1]
    prefix = fname.split('.')[0]
    mask_fname = labeled_dir+'/watershed_mask/{}_watershed_mask.png'.format(prefix)
    if (path.exists(mask_fname)) :
        lines = getBBoxLines(mask_fname)
    else :
        lines = line = '{} {} {} {} {}\n'.format(0, 0.5, 0.5, 1, 1)
    copy = data_dir+'Yolo/all/images/{}'.format(fname)
    shutil.copyfile(images[i], copy)
    f = open(data_dir+'Yolo/all/labels/{}.txt'.format(prefix), 'w+')
    f.write(lines)
    f.close()

def deleteUnlabeled():
  images = glob.glob(labeled_dir+'raw_images/*.jpg')
  images.sort()
  for i in range(len(images)):
    fname = images[i].split('/')[-1]
    prefix = fname.split('.')[0]
    mask_fname = labeled_dir+'/watershed_mask/{}_watershed_mask.png'.format(prefix)
    if (not path.exists(mask_fname)) :
      os.remove(images[i])
      copy = labeled_dir+'pixel_annotation/'+fname
      if (path.exists(copy)):
        os.remove(copy)
    else:
      cv_img = cv2.imread(mask_fname)[:,:,0]
      cv_img = 255*(cv_img == 1).astype(np.uint8)
      if np.max(cv_img) == 0:
        os.remove(images[i])
        os.remove(mask_fname)
        copy = labeled_dir+'pixel_annotation/'+fname
        mask_copy = labeled_dir+'/pixel_annotation/{}_watershed_mask.png'.format(prefix)
        color_fname = labeled_dir+'/color_mask/{}_color_mask.png'.format(prefix)
        color_copy = labeled_dir+'/pixel_annotation/{}_color_mask.png'.format(prefix)
        manual = labeled_dir+'/mask/{}_mask.png'.format(prefix)
        manual_copy = labeled_dir+'/pixel_annotation/{}_mask.png'.format(prefix)
        if (path.exists(color_fname)):
          os.remove(color_fname)
        if (path.exists(manual)):
          os.remove(manual)
        if (path.exists(copy)):
          os.remove(copy)
        if (path.exists(mask_copy)):
          os.remove(mask_copy)
        if (path.exists(color_copy)):
          os.remove(color_copy)
        if (path.exists(manual_copy)):
          os.remove(manual_copy)

def renameForSortOrder():
  subdirs = ['pixel_annotation/', 'raw_images/', 'mask/', 'color_mask/', 'watershed_mask/']
  for subdir in subdirs:
    images = glob.glob(labeled_dir+subdir+'*')
    for i in range(len(images)):
      fname = images[i].split('/')[-1]
      match = re.search('^frame\d{4}[\._].*$', fname)
      if match:
        new_fname = fname[:5] + '0' + fname[5:]
        new_path = labeled_dir+subdir+new_fname
        os.rename(images[i], new_path)

def showLabels():
  images = glob.glob(labeled_dir+'raw_images/*.jpg')
  images.sort()
  for i in range(len(images)):
    fname = images[i].split('/')[-1]
    prefix = fname.split('.')[0]
    mask_fname = labeled_dir+'/watershed_mask/{}_watershed_mask.png'.format(prefix)
    if(not path.exists(mask_fname)):
      print("Warning: file path {} does not exist!".format(mask_fname))
      continue
    cv_img = cv2.imread(images[i])
    cv_mask = 155*(cv2.imread(mask_fname)[:,:,0] == 1).astype(np.uint8)
    label_mask = measure.label(cv_mask)
    props = measure.regionprops(label_mask)
    
    for prop in props:
      minr, minc, maxr, maxc= prop.bbox
      cv2.rectangle(cv_img, (minc,minr), (maxc,maxr), color=(0,0,255))
    cv2.imshow("image", cv_img)
    cv2.waitKey(200)

# deleteUnlabeled()
# renameForSortOrder()
# showLabels()
getYoloFormat()
getTrainValSets()