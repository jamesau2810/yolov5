# YOLOv5 ð€ by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/LNwODJXcvt4'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""
import time
import argparse
import csv
import os
import platform
import sys
from pathlib import Path
import cv2
import numpy as np
# import torch
# import socket
# import usb.core
# import usb.util
import serial
# import nuc_usb_test
import library
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams,CaptureImages
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode







def main(opt):
    # dev = usb.core.find(idVendor=0x045e, idProduct=0x028e)
    # # If the device is not found, raise an error
    # if dev is None:
    #     raise ValueError('Device not found')

    # Set the configuration of the USB device
    # dev.set_configuration()
    cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
    cap.set(cv2.CAP_PROP_FPS, 36)
    # serialObj = serial.Serial(library.pixhawk_path)
    time.sleep(3)
    # check_requirements(ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
    # library.run_yolo_loop(**vars(opt))
    # ROOT / '516heli014_jpg.rf.32d59be86a560186676fe6c309d1b913.jpg'

    while True:
        ret, image = cap.read()
        filename = ROOT / 'temp.jpg'
        cv2.imwrite(filename, image)
        xyxy_best,x,y = library.run_yolo_loop(
            weights=ROOT / 'best.pt',
            source=filename,
            source_image= image,
            # dev=dev,
            # serialObj = 
            )
        # library.Box2Send(xyxy_best,serialObj,x,y)
    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     ret, image = cap.read()
    #     filename = ROOT / 'temp.jpg'
    #     cv2.imwrite(filename, image)
    #     # check_requirements(ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
    #     # library.run_yolo_loop(**vars(opt))
    #     # ROOT / '516heli014_jpg.rf.32d59be86a560186676fe6c309d1b913.jpg'
    #     library.run_yolo_loop(weights=ROOT / 'best.pt',source=filename,dev=dev)
    #     library.run_yolo_loop(weights=ROOT / 'best.pt',source=filename) # ,socket = s
    #/home/jamesau/Downloads/yolov5-master/best.pt
    #/home/jamesau/Downloads/yolov5-master/516heli014_jpg.rf.32d59be86a560186676fe6c309d1b913.jpg


if __name__ == '__main__':
    # tic = time.perf_counter()
    opt = library.parse_opt()
    main(opt)
    # toc = time.perf_counter()
    # print(f"Downloaded the tutorial in {toc - tic:0.4f} seconds")


"""
import usb.core
import usb.util

# Find the USB device
dev = usb.core.find(idVendor=0x045e, idProduct=0x028e)

# If the device is not found, raise an error
if dev is None:
    raise ValueError('Device not found')

# Set the configuration of the USB device
dev.set_configuration()

# Write data to the USB port
dev.write(1, b'Hello, World!')

# Read data from the USB port
data = dev.read(0x81, 1024)

# Print the data
print(data)



import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello, world")
    data = s.recv(1024)

print(f"Received {data!r}")
"""
