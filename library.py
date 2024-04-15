"""
Read this documents to see How to use the API method

https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-global-int

https://mavlink.io/en/messages/common.html

"""
# from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import dronekit_sitl
# import socket
# try:
#     import exceptions
# except ImportError:
#     pass
# import cv2
import numpy as np
# import torch
import math
import cmath
import argparse
import pymavlink
import pymavlink.mavutil as mavutil
import time


import time
import argparse
import csv
import os
import platform
import sys
from pathlib import Path
import numpy as np
import torch
import socket
# import usb.core
# import usb.util
import serial
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


# Yolo section
# pixhawk_path = '/dev/cu.usbmodem141301'
# pixhawk_path = '/dev/ttyACM0'
pixhawk_path = '/dev/ttyUSB0'


@smart_inference_mode()
def run_yolo_loop(
        weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
        source_image = np.zeros((640,640)),
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_csv=False,  # save results in CSV format
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
        # dev = usb.core.find(idVendor=0x045e, idProduct=0x028e),
        # serialObj = serial.Serial(pixhawk_path)
        # socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM),
):
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
    screenshot = source.lower().startswith('screen')
    if is_url and is_file:
        source = check_file(source)  # download
    # Directories
    # save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    # (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size
    if webcam:
        view_img = check_imshow(warn=True)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        bs = len(dataset)
    elif screenshot:
        dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
    else:
        # dataset = CaptureImages(img_added=source_image,img_size=imgsz,stride=stride,auto=pt,vid_stride=vid_stride)
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
    have_result = False
    xyxy_best = [25,25,75,75]
    x_point = 100
    y_point = 100
    for path1, im, im0s, vid_cap, s in dataset:
        # path = path1

        im,pred = func_a(dt,im,model,augment,conf_thres,iou_thres,classes,agnostic_nms,max_det)

        # toc = time.perf_counter()
        # print(f"Downloaded the tutorial in {toc - tic:0.4f} seconds")
        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
        # pred : [:,:,:4]
        # Define the path for the CSV file
        # csv_path = save_dir / 'predictions.csv'

        # Create or append to the CSV file
        # def write_to_csv(image_name, prediction, confidence):
        #     data = {'Image Name': image_name, 'Prediction': prediction, 'Confidence': confidence}
        #     with open(csv_path, mode='a', newline='') as f:
        #         writer = csv.DictWriter(f, fieldnames=data.keys())
        #         if not csv_path.is_file():
        #             writer.writeheader()
        #         writer.writerow(data)
        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            im0,s_pred_pros = pred_pro_1(webcam,im0s,i)
            # im0,save_path,txt_path,gn, imc,annotator ,s_pred_pros  = pred_processing(webcam,im0s,i,save_dir,dataset,frame,im,save_crop,line_thickness,names)
            s += s_pred_pros
            # Print time (inference-only)
            LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                # XYXY:0,1,2,3. confidence score:4. Class:5
                # Extracted Helipad object
                det_New = [a for a in det if names[int(a[5])]=='helipad']
                # cdcv = 0
                # Sorted according to determinant score
                # if len(det_New):
                det_New.sort(key=lambda x: x[4])
                det_best = det_New[0]
                # if det_best
                xyxy_best_chose = det_best[:4]
                # Yogesh, start of here
                # x,y =  im.shape[:2]
                x,y =  im0.shape[:2]
                have_result = True
                xyxy_best =  xyxy_best_chose
                x_point = x
                y_point = y
                # Box2Send(xyxy_best,serialObj,x,y)
                # # Read data from the USB port
                # data = dev.read(0x81, 1024)

                # Print the data
                # print(data)

                # The result is :
                # det
                # Pick here
                s_print_res = print_result_yolo(det,names)
                s += s_print_res
                # Pick Here
            # stream_result(annotator,view_img,p,windows)
            # write_result(det,names,hide_conf,save_csv,write_to_csv,save_txt,gn,save_conf,txt_path,save_img,save_crop,view_img,hide_labels,annotator,imc,save_dir,p)
            # save_result(save_img,dataset,im0,vid_path,vid_writer,vid_cap,i)
    # print_results_end_yolo(seen,dt,save_txt,save_img,save_dir,imgsz,update,weights)
    return have_result,xyxy_best,x_point,y_point



# end method
@smart_inference_mode()
def pred_processing(webcam,im0s,i,save_dir,dataset,frame,im ,save_crop,line_thickness,names):
    s = ""
    im0,s1 = pred_pro_1(webcam,im0s,i)
    save_path,txt_path,gn, imc,annotator, s2 = pred_pro_2(save_dir,dataset,frame,im,im0 ,save_crop,line_thickness,names)
    s = s1 + s2
    return im0,save_path,txt_path,gn, imc,annotator ,s
@smart_inference_mode()
def pred_pro_2(save_dir,dataset,frame,im ,im0,save_crop,line_thickness,names):
    s = ""
    p = Path(p)  # to Path
    save_path = str(save_dir / p.name)  # im.jpg
    txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
    s += '%gx%g ' % im.shape[2:]  # print string
    gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
    imc = im0.copy() if save_crop else im0  # for save_crop
    annotator = Annotator(im0, line_width=line_thickness, example=str(names))
    return save_path,txt_path,gn, imc,annotator, s
@smart_inference_mode()
def pred_pro_1(webcam,im0s,i):
    s = ""
    if webcam:  # batch_size >= 1
        im0 = im0s[i].copy()
        # p,  frame = path[i],  dataset.count
        s += f'{i}: '
    else:
        im0 = im0s.copy()
        # p,  frame = path, getattr(dataset, 'frame', 0)
    return im0,s
@smart_inference_mode()
def print_results_end_yolo(seen,dt,save_txt,save_img,save_dir,imgsz,update,weights):
    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)

@smart_inference_mode()
def func_a(dt,im,model,augment,conf_thres,iou_thres,classes,agnostic_nms,max_det):
    with dt[0]:
        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        if model.xml and im.shape[0] > 1:
            ims = torch.chunk(im, im.shape[0], 0)
        # print("image Size",im.size())
    # tic = time.perf_counter()
    # Inference
    with dt[1]:
        visualize = False
        # visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        if model.xml and im.shape[0] > 1:
            pred = None
            for image in ims:
                if pred is None:
                    pred = model(image, augment=augment, visualize=visualize).unsqueeze(0)
                else:
                    pred = torch.cat((pred, model(image, augment=augment, visualize=visualize).unsqueeze(0)), dim=0)
            pred = [pred, None]
        else:
            pred = model(im, augment=augment, visualize=visualize)
    # NMS
    with dt[2]:
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    return im,pred
@smart_inference_mode()
def print_result_yolo(det,names):
    s = ""
    # Print results
    for c in det[:, 5].unique():
        n = (det[:, 5] == c).sum()  # detections per class
        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
    return s
@smart_inference_mode()
def stream_result(annotator,view_img,p,windows):
    # Stream results
    im0 = annotator.result()
    if view_img:
        if platform.system() == 'Linux' and p not in windows:
            windows.append(p)
            cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
            cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
        cv2.imshow(str(p), im0)
        cv2.waitKey(1)  # 1 millisecond
        return
@smart_inference_mode()
def save_result(save_img,dataset,im0,vid_path,vid_writer,vid_cap,i):
    # Save results (image with detections)
    if save_img:
        if dataset.mode == 'image':
            cv2.imwrite(save_path, im0)
        else:  # 'video' or 'stream'
            if vid_path[i] != save_path:  # new video
                vid_path[i] = save_path
                if isinstance(vid_writer[i], cv2.VideoWriter):
                    vid_writer[i].release()  # release previous video writer
                if vid_cap:  # video
                    fps = vid_cap.get(cv2.CAP_PROP_FPS)
                    w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                else:  # stream
                    fps, w, h = 30, im0.shape[1], im0.shape[0]
                save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
            vid_writer[i].write(im0)
    return
@smart_inference_mode()
def write_result(det,names,hide_conf,save_csv,write_to_csv,save_txt,gn,save_conf,txt_path,save_img,save_crop,view_img,hide_labels,annotator,imc,save_dir,p):
    # Write results
    for *xyxy, conf, cls in reversed(det):
        c = int(cls)  # integer class
        label = names[c] if hide_conf else f'{names[c]}'
        confidence = float(conf)
        confidence_str = f'{confidence:.2f}'

        if save_csv:
            write_to_csv(p.name, label, confidence_str)

        if save_txt:  # Write to file
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
            line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
            with open(f'{txt_path}.txt', 'a') as f:
                f.write(('%g ' * len(line)).rstrip() % line + '\n')

        # # To Be commented out
        if save_img or save_crop or view_img:  # Add bbox to image
            c = int(cls)  # integer class
            label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
            annotator.box_label(xyxy, label, color=colors(c, True))
        if save_crop:
            save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
@smart_inference_mode()
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path or triton URL')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob/screen/0(webcam)')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-csv', action='store_true', help='save results in CSV format')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt



# Nuc Pixhawk Section
def capture_and_yolo_read(cap,weightpath):
    ret, image = cap.read()
    filename = ROOT / 'temp.jpg'
    cv2.imwrite(filename, image)
    print("a")
    have_result,xyxy_best,x,y = run_yolo_loop(weights=weightpath,source=filename,source_image= image)
    return have_result,xyxy_best,x,y




def ArduinoSent(left, up, width_x, width_y, serialObj):
    SendItem = '{0:0=3d}'.format(left) + '{0:0=3d}'.format(up) + '{0:0=3d}'.format(width_x) + '{0:0=3d}'.format(width_y)
    enc = SendItem.encode(encoding="utf-8")
    ied = serialObj.write(enc)
    return ied







def printStatus(vehicle):
    # vehicle is an instance of the Vehicle class
    print("Autopilot Firmware version: %s" % vehicle.version)
    print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)  # NED
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("GPS: %s" % vehicle.gps_0)
    print("Groundspeed: %s" % vehicle.groundspeed)
    print("Airspeed: %s" % vehicle.airspeed)
    print("Gimbal status: %s" % vehicle.gimbal)
    print("Battery: %s" % vehicle.battery)
    print("EKF OK?: %s" % vehicle.ekf_ok)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print("Rangefinder: %s" % vehicle.rangefinder)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print("Heading: %s" % vehicle.heading)
    print("Is Armable?: %s" % vehicle.is_armable)
    print("System status: %s" % vehicle.system_status.state)
    print("Mode: %s" % vehicle.mode.name)  # settable
    print("Armed: %s" % vehicle.armed)  # settable








# def land_local(vehicle,altitude ):

#     message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
#                                               mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL, 0, 1, 0, 0, 0, 0, 0, altitude)

#     vehicle.mav.send(message)
#     response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
#     print(response)
#     if response and response.command == mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
#         print("Command accepted")
#     else:
#         print("Command failed")

# Other Component
def deploy_tube():
    return

def waypoint_file_read(filePath):
    db =open(filePath)
    myline = db.readlines()
    dataLine = myline[1:]
    words = [i.rstrip("\n").split("\t")  for i in dataLine]
    points = [{"point":int(i[0]),"latitude":int(float(i[8])* 1e7),"longitude":int(float(i[9])* 1e7),"altitude":float(i[10])}  for i in words if int(i[3])==16]

    return words,points

# Support Math Equation

def compute_direction(heading, x, y, x_mid, y_mid):

    angle = math.radians(heading)
    xa = x - x_mid
    ya = y - y_mid
    cmplx_coor = complex(xa, ya)
    length, box_angle = cmath.polar(cmplx_coor)
    correct_angle = (box_angle + angle) % (2 * np.pi)
    # We use log as a speed fmla
    res = cmath.rect(np.log10(length), correct_angle)
    x_velo = res.real
    y_velo = res.imag
    return x_velo, y_velo
    # send_int_velocity(vehicle, x_velo, y_velo, 0)

def Box2Send(xyxy_best, x, y):
    xmin = int(xyxy_best[0])
    ymin = int(xyxy_best[1])
    xmax = int(xyxy_best[2])
    ymax = int(xyxy_best[3])
    centre_point_x = (xmin + xmax) / 2
    centre_point_y = (ymin + ymax) / 2
    # print(type(xmin))
    width_x = int((xmax - xmin) / x)
    width_y = int((ymax - ymin) / y)
    # width_x = int(torch.round((xmax-xmin)/ x))
    # width_y = int(torch.round((ymax-ymin)/ y))
    left = int(centre_point_x * 100 / x)
    up = int(centre_point_y * 100 / y)
    # left = int(torch.round(centre_point_x*100 / x))#centre_point_x - (x/2)
    # up = int(torch.round(centre_point_y*100 / y))#centre_point_y - (y/2)
    # width_x,width_y
    # SendItem=str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)
    # ArduinoSent(left,up,width_x,width_y,serialObj)
    # # Write data to the USB port
    # dev.write(1, b'Hello, World!')
    # serialObj.write(SendItem.encode('UTF-8'))
    return left,up,width_x,width_y

def Box2Speed(heading,xyxy_best, x, y):
    xmin = int(xyxy_best[0])
    ymin = int(xyxy_best[1])
    xmax = int(xyxy_best[2])
    ymax = int(xyxy_best[3])
    centre_point_x = (xmin + xmax) / 2
    centre_point_y = (ymin + ymax) / 2
    # print(type(xmin))
    width_x = int((xmax - xmin) / x)
    width_y = int((ymax - ymin) / y)
    # width_x = int(torch.round((xmax-xmin)/ x))
    # width_y = int(torch.round((ymax-ymin)/ y))
    left = int(centre_point_x * 100 / x)
    up = int(centre_point_y * 100 / y)
    x_velo, y_velo = compute_direction(heading,left,up,centre_point_x,centre_point_y)
    # left = int(torch.round(centre_point_x*100 / x))#centre_point_x - (x/2)
    # up = int(torch.round(centre_point_y*100 / y))#centre_point_y - (y/2)
    # width_x,width_y
    # SendItem=str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)
    # ArduinoSent(left,up,width_x,width_y,serialObj)
    # # Write data to the USB port
    # dev.write(1, b'Hello, World!')
    # serialObj.write(SendItem.encode('UTF-8'))
    return x_velo, y_velo

def Box2Speed_Helipad_Land(heading,xyxy_best, x, y):
    xmin = int(xyxy_best[0])
    ymin = int(xyxy_best[1])
    xmax = int(xyxy_best[2])
    ymax = int(xyxy_best[3])
    centre_point_x = (xmin + xmax) / 2
    centre_point_y = (ymin + ymax) / 2
    # print(type(xmin))
    width_x = (xmax - xmin)*100 / x
    width_y = (ymax - ymin)*100 / y
    # width_x = int(torch.round((xmax-xmin)/ x))
    # width_y = int(torch.round((ymax-ymin)/ y))
    left = int(centre_point_x * 100 / x)
    up = int(centre_point_y * 100 / y)

    x_velo, y_velo = compute_direction(heading,left,up,centre_point_x,centre_point_y)
    return x_velo, y_velo, left, up, width_x, width_y
def Helipad_land_speed_factor(x_velo_old, y_velo_old,alt,benchmark,hori_grad,vert_grad):
    # sdf = zip(benchmark,)
    satis = np.sum(list(map(lambda x: alt_fmla(alt,x,2),benchmark)))
    hori_fac = hori_grad[satis]
    land_speed = vert_grad[satis]
    x_velo_new = x_velo_old * hori_fac
    y_velo_new = y_velo_old * hori_fac
    return x_velo_new,y_velo_new,land_speed
def Helipad_margin(left, up, width_x,width_y):
    centre_enough = False
    close_enough = False
    if np.abs(left) <= 10 and np.abs(up) <= 10:
        centre_enough = True
    if width_x >=80 and  width_y >= 80:
        close_enough = True
    return centre_enough, close_enough
def Box2Send(xyxy_best, x, y ,serialObj):
    xmin = int(xyxy_best[0])
    ymin = int(xyxy_best[1])
    xmax = int(xyxy_best[2])
    ymax = int(xyxy_best[3])
    centre_point_x = (xmin + xmax) / 2
    centre_point_y = (ymin + ymax) / 2
    # print(type(xmin))
    width_x = int((xmax - xmin) / x)
    width_y = int((ymax - ymin) / y)
    # width_x = int(torch.round((xmax-xmin)/ x))
    # width_y = int(torch.round((ymax-ymin)/ y))
    left = int(centre_point_x * 100 / x)
    up = int(centre_point_y * 100 / y)
    # left = int(torch.round(centre_point_x*100 / x))#centre_point_x - (x/2)
    # up = int(torch.round(centre_point_y*100 / y))#centre_point_y - (y/2)
    # width_x,width_y
    # SendItem=str(left)+"&"+str(up)+"&"+str(width_x)+"&"+str(width_y)
    ArduinoSent(left,up,width_x,width_y,serialObj)
    # # Write data to the USB port
    # dev.write(1, b'Hello, World!')
    # serialObj.write(SendItem.encode('UTF-8'))
    return left,up,width_x,width_y

def intv_check(a,b,intv):
    c = (a >= b-intv) and (a <= b+intv)
    return c
def alt_fmla(curr_alt,target_alt = 0,mode = 0):
    ratio  = 0.05
    scale = 1000
    min_rat = (1-ratio) *scale
    max_rat = (1+ratio) * scale
    if mode ==0:
        return curr_alt >= target_alt * min_rat
    elif mode == 1:
        return curr_alt >= target_alt * min_rat and curr_alt <= target_alt * max_rat
    elif mode == 2:
        return curr_alt <= target_alt * max_rat
    else:
        print("Error")
        return True

# Combined Loop

def Helipad_Land_and_send(vehicle,velocity_x, velocity_y,loc,benchmark,hori_grad,vert_grad):
    velocity_x_new,velocity_y_new,land_speed = Helipad_land_speed_factor(velocity_x, velocity_y,loc.relative_alt,benchmark,hori_grad,vert_grad)
    send_int_velocity(vehicle,0, 0,land_speed)

def Helipad_Track_Land(vehicle,cap,weightPath):
    time_stamping = 0
    velocity_x, velocity_y = 0,0
    while True:
        land_speed = 0.5
        loc = checklocation(vehicle)
        have_result,xyxy_best,x,y = capture_and_yolo_read(cap,weightPath)
        print("Altitude: ",loc.relative_alt, ", Latitude: ",loc.lat ,", Longtitude: ",loc.lon )
        # dev=dev,
        if have_result:
            
            velocity_x, velocity_y, left, up, width_x,width_y = Box2Speed_Helipad_Land(loc.hdg,xyxy_best,x,y)
            centre_enough, close_enough = Helipad_margin(left, up, width_x,width_y)
            print("Centre Enough: ",centre_enough,", xyxy: ", xyxy_best,", Left: ", left,", Up: ",up, "x:y:",x,y)
            if alt_fmla(loc.relative_alt,1,2) and centre_enough:
                print("Land Now")
                land(vehicle)
                return
            elif loc.relative_alt <= 0:
                print("Hit ground")
                disarm(vehicle)
                return
            else:
                if centre_enough:
                    velocity_x_new,velocity_y_new,land_speed = Helipad_land_speed_factor(velocity_x, velocity_y,loc.relative_alt,[5,2,-1],[0.8,0.8,0.4],[2,1.2,0.8])
                else:
                    velocity_x_new,velocity_y_new,land_speed = Helipad_land_speed_factor(velocity_x, velocity_y,loc.relative_alt,[5,1,-1],[1,1,1],[1,0.5,0])
                # send_int_velocity(vehicle,velocity_x_new,velocity_y_new,land_speed)
                send_int_velocity(vehicle,0,0,land_speed)
                print("run one loop")
                time_stamping = time.time()
        else:
            Helipad_Land_and_send(vehicle,0,0,loc,[5,2,-1],[1,1,1],[1,0.25,0])
            time_stamping = time.time()
        if  time.time()>= time_stamping + 2:
            # velocity_x_new,velocity_y_new,land_speed = Helipad_land_speed_factor(velocity_x, velocity_y,loc.relative_alt,)
            # send_int_velocity(vehicle,0, 0,land_speed)
            Helipad_Land_and_send(vehicle,0,0,loc,[5,2,-1],[1,1,1],[0.5,0.25,0])

def Helipad_track(vehicle,cap,weightPath):
    time_stamping = 0
    velocity_x, velocity_y = 0,0
    while True:
        
        have_result,xyxy_best,x,y = capture_and_yolo_read(cap,weightPath)
        # dev=dev,
        if have_result:
            loc = checklocation(vehicle)
            velocity_x, velocity_y = Box2Speed(loc.hdg,xyxy_best,x,y)
            send_int_velocity(vehicle,velocity_x, velocity_y,0)
            print("run one loop")
            time_stamping = time.time()
        else:
            send_int_velocity(vehicle,0, 0,0)
        if  time.time()>= time_stamping + 2:
            send_int_velocity(vehicle,velocity_x, velocity_y,0)
        # time.sleep(1)

def SurveyScan_with_stop(vehicle,waypoints,cap,weightPath):
    result = SurveyScan_with_stop_Loop(vehicle,waypoints,cap,weightPath)
    if result:
        loc = checklocation(vehicle)
        print("Deploy tube")
        deploy_tube()
        return

def SurveyScan(vehicle,waypoints,cap,weightPath):
    scan_location_list = []
    for i in waypoints:
        path_scan_list = waypoint_with_scan(vehicle,i["latitude"],i["longitude"],i["altitude"],cap,weightPath,hold=10,acptrad=0,passrad=0,yaw = 0)
        scan_location_list.append(path_scan_list)
    return scan_location_list
def waypoint_with_scan(vehicle,latitude,longitude,altitude,cap,weightPath,hold=10,acptrad=0,passrad=0,yaw = 0):
    Scan_Location_List = []
    send_int_velo_pos_cmd(vehicle,0,latitude, longitude, altitude, 0, 0, 0,  0, 0, 0,1.57, 0.5)
    interval =2
    while True:
        loc = checklocation(vehicle)
        print("Latitude: ",loc.lat ,", Longtitude: ",loc.lon ,", Altitude: ",loc.relative_alt)
        # Break and return from function just below target altitude.
        if alt_fmla(loc.relative_alt,altitude,1) and intv_check(loc.lon,longitude,interval) and intv_check(loc.lat,latitude,interval):
            print("Reached location")
            break
        else:
            have_result,xyxy_best,x,y = capture_and_yolo_read(cap,weightPath)
            if have_result:
                loc = checklocation(vehicle)
                Scan_Location_List.append(loc)
        time.sleep(1)
    return Scan_Location_List

def instr_2_takeoff(vehicle,modeUsed,altitude = 10):
    
    set_mode(vehicle,modeUsed)


    #
    arm(vehicle)
    stream_location(vehicle)
    # ori_loc = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    # offset = ori_loc.alt
    # print(ori_loc)

    # library.set_mode(vehicle,modeUsed)
    print(checklocation(vehicle))
    takeoff(vehicle,altitude)





#  Copter communication Method
def takeoff(vehicle,altitude ):
    # Take Off to an altitude Vehicle: CopterObject, altitude: metre
    original_loc = checklocation(vehicle)
    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, 0, altitude)

    vehicle.mav.send(message)
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    # print(vehicle.recv_match(type='SYS_STATUS', blocking=True))
    if response and response.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
        check_alt_arrived(vehicle,original_loc,altitude)
        return
    else:
        print("Command failed")
        return

def land(vehicle):
    # Land in Local position
    altitude = 0
    original_loc = checklocation(vehicle)
    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, 0, altitude)

    vehicle.mav.send(message)
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    # print(vehicle.recv_match(type='SYS_STATUS', blocking=True))
    if response and response.command == mavutil.mavlink.MAV_CMD_NAV_LAND and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
        while True:
            loc = checklocation(vehicle)
            print(" Altitude: ", loc.relative_alt)
            # Break and return from function just below target altitude.
            if loc.relative_alt <=0+0.05:
                print("Reached target altitude")
                break
            time.sleep(1)
        return
    else:
        print("Command failed")
        return

def waypoint(vehicle,latitude,longitude,altitude,hold=10,acptrad=0,passrad=0,yaw = 0):
    # Goto a position

    # message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
    #                                           mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0,0,0 , passrad, yaw,latitude,longitude,altitude  )
    send_int_velo_pos_cmd(vehicle,0,latitude, longitude, altitude, 0, 0, 0,  0, 0, 0,1.57, 0.5) #hold, acptrad
    # vehicle.mav.send(message)
    # response = send_int_velo_pos_cmd(vehicle,0,latitude, longitude, altitude, 0, 0, 0,  0, 0, 0,1.57, 0.5)
    # response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    # print(response)
    check_location_arrived(vehicle,latitude,longitude,altitude,20)
    return
    # if response and response.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    #     print("Command accepted")
    #     check_location_arrived(vehicle,latitude,longitude,altitude,20)
    #     return
    # else:
    #     print("Command failed")
    #     return

def return_to_launch(vehicle):
    # Auto return to launch position
    # At a cost of being change to RTL mode
    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    vehicle.mav.send(message)
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    if response and response.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")


def send_int_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
    #           the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), 40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
    # 86
    
    # accel_x,accel_y,accel_z,yaw,yaw_rate
    send_int_velo_pos_cmd(vehicle,1, 0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0,0, 0)
    # response = send_int_velo_pos_cmd(vehicle,1, 0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0,0, 0)
    # print(response)
    # msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #     0,  # time_boot_ms (not used)
    #     0, 0,  # target system, target component
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
    #     0b0000111111000111,  # type_mask (only speeds enabled)
    #     0, 0, 0,  # x, y, z positions (not used)
    #     velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
    #     0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    #     0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # # vehicle.
    # # send command to vehicle on 1 Hz cycle
    # for x in range(0, duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)

def waitMessage(vehicle,msgid):
    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, msgid, 0, 0, 0, 0, 0, 0)
    vehicle.mav.send(message)
    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    return response



def connectMyCopter():
    MachineType = 0
    connection_string = ""
    if MachineType == 0:
        parser = argparse.ArgumentParser(description="commands")
        parser.add_argument("--connect")
        args = parser.parse_args()
        #connection_string = args.connect
        connection_string = "127.0.0.1:14550"
        # connection_string = "127.0.0.1:14551"
    elif MachineType == 1:
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        # connection_string = "127.0.0.1:14550"
    elif MachineType == 2:
        connection_string ="/dev/cu.usbserial-14110"
    elif MachineType == 3:
        connection_string = "/dev/ttyUSB0"
    print("Connect On:", connection_string)

    # baud_rate = 57600
    # vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)

    # Start a connection listening on a UDP port
    vehicle = mavutil.mavlink_connection(connection_string, baud=57600)
    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    vehicle.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

    return vehicle

def set_mode(vehicle,mode):
    # https://ardupilot.org/copter/docs/parameters.html#fltmode1-flight-mode-1
    # Not working
    mode_dict = {"GUIDED":4,"STABILIZE":0,"AUTO":3}
    mode_num = mode_dict[mode]
    vehicle.set_mode( mode_num, custom_mode = 0, custom_sub_mode = 0)
    # message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
    #                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mode_num, 0, 0, 0, 0, 0, 0)

    # vehicle.mav.send(message)
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    if response and response.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")

def arm(vehicle):
    print("Arm command")
    arm_disarm_command(vehicle,1)

    # print(vehicle.recv_match( blocking=True))
    # vehicle.mav.send()
    # Safety check UNCOMMENT BEFORE DEPLOYMENT
    # while vehicle.is_armable == False:
    #     print(f"Waiting for vehicles to become armable {vehicle.is_armable}")
    #     time.sleep(1)
    # print("Vehicle is now armable")
    # print("")
    # vehicle.armed = True
    vehicle.motors_armed_wait()
    # while vehicle.armed == False:
    #     print("Waiting for drone to become armed ")
    #     time.sleep(1)
    # print("Vehicle is now armed")
    # print("props are spinning, LOOK OUT!")
    # return vehicle
    return
def disarm(vehicle):
    # vehicle.mav.
    #  400
    print("Disarm command")
    arm_disarm_command(vehicle,0)

def check_alt_arrived(vehicle,original_pos,target_alt):
    # msg = mavutil.mavlink.GLOBAL_POSITION_INT
    while True:
        loc = checklocation(vehicle)
        print(" Altitude: ", loc.relative_alt)
        # Break and return from function just below target altitude.
        if alt_fmla(loc.relative_alt,target_alt,0):
            print("Reached target altitude")
            break
        time.sleep(1)

def check_location_arrived(vehicle,lat, lon, alt, interval):

    while True:
        loc = checklocation(vehicle)
        print("Latitude: ",loc.lat ,", Longtitude: ",loc.lon ,", Altitude: ",loc.relative_alt)
        # Break and return from function just below target altitude.
        if alt_fmla(loc.relative_alt,alt,1) and intv_check(loc.lon,lon,interval) and intv_check(loc.lat,lat,interval):
            print("Reached location")
            break
        time.sleep(1)




def stream_location(vehicle):
    # msd_id = mavutil.mavlink.GLOBAL_POSITION_INT
    msd_id = 33
    stream_msg(vehicle,msd_id)



# Copter Command Abstract
def send_int_velo_pos_cmd(vehicle,type_mask_name,postion_x,postion_y,postion_z, velocity_x, velocity_y, velocity_z,accel_x,accel_y,accel_z,yaw,yaw_rate):
    # pos x:	Latitude * 1e7 pos y: Longitude * 1e7, pos z: alt, velocity_x: m/s, velocity_y: m/s, velocity_z: m/s,accel_x: m/s/s,accel_y: m/s/s,accel_z: m/s/s,
    type_mask = 0
    if type_mask_name == 0:
        type_mask = int(0b110111111000)# Use position
    elif type_mask_name == 1:
        type_mask = int(0b110111000111)# USe velocity
    elif type_mask_name == 2:
        type_mask = int(0b110111000000)# USe both
    else:
        return
    # message = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, vehicle.target_system,
    #                     vehicle.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask ,
    #                     postion_x,postion_y,postion_z,
    #                     velocity_x, velocity_y, velocity_z,accel_x,accel_y,accel_z,yaw,yaw_rate
    #                     )
    # SET_POSITION_TARGET_GLOBAL_INT
    # message = mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, vehicle.target_system,
    #                     vehicle.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, type_mask ,
    #                     postion_x,postion_y,postion_z,
    #                     velocity_x, velocity_y, velocity_z,accel_x,accel_y,accel_z,yaw,yaw_rate
    #                     )
    # vehicle.mav.send(message)
    # https://mavlink.io/zh/mavgen_python/
    vehicle.mav.set_position_target_global_int_send(10, vehicle.target_system,
                        vehicle.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, type_mask ,
                        postion_x,postion_y,postion_z,
                        velocity_x, velocity_y, velocity_z,accel_x,accel_y,accel_z,yaw,yaw_rate
                        )
    return
    # return vehicle.recv_match(type='SYS_STATUS', blocking=True)
    # return vehicle.recv_match(type='COMMAND_ACK', blocking=True)

def stream_msg(vehicle,msd_id):
    # msd_id = mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS
    # Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
    # param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
    # param2: 1000000 (Stream interval in microseconds)
    message = vehicle.mav.command_long_encode(
            vehicle.target_system,  # Target system ID
            vehicle.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            msd_id,  # param1: Message ID to be streamed
            1000000, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            0        # param6 (unused)
            )
    vehicle.mav.send(message)
    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command Set Message Stream accepted")
    else:
        print("Command failed")

def arm_disarm_command(vehicle,bit):
    message = vehicle.mav.command_long_encode(vehicle.target_system, vehicle.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, bit, 0, 0, 0, 0, 0, 0)
    # Send the COMMAND_LONG
    vehicle.mav.send(message)
    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(response)
    print(response.command, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print(response.result, mavutil.mavlink.MAV_RESULT_ACCEPTED)
    if response and response.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")
    return
def checklocation(vehicle):
    # response = waitMessage(vehicle,33)
    return vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

def SurveyScan_with_stop_Loop(vehicle,waypoints,cap,weightPath):
    for i in waypoints:
        path_scan_result = waypoint_with_scan_stop(vehicle,i["latitude"],i["longitude"],i["altitude"],cap,weightPath,hold=10,acptrad=0,passrad=0,yaw = 0)
        if path_scan_result:
            return True
    return False
def waypoint_with_scan_stop(vehicle,latitude,longitude,altitude,cap,weightPath,hold=10,acptrad=0,passrad=0,yaw = 0):
    send_int_velo_pos_cmd(vehicle,0,latitude, longitude, altitude, 0, 0, 0,  0, 0, 0,1.57, 0.5)
    interval =2
    while True:
        loc = checklocation(vehicle)
        print("Latitude: ",loc.lat ,", Longtitude: ",loc.lon ,", Altitude: ",loc.relative_alt)
        # Break and return from function just below target altitude.
        if alt_fmla(loc.relative_alt,altitude,1) and intv_check(loc.lon,longitude,interval) and intv_check(loc.lat,latitude,interval):
            print("Reached location")
            return False
        else:
            have_result,xyxy_best,x,y = capture_and_yolo_read(cap,weightPath)
            if have_result:
                # loc = checklocation(vehicle)
                return True
        time.sleep(1)
