#!/home/rm/miniforge3/envs/yolov5/bin/python3
# coding: utf-8
from roborts_msgs.msg import armor_detecte_result
import pyrealsense2 as rs
from models.experimental import attempt_load
from utils.datasets import *
from utils.general import *
from utils.torch_utils import *
import rospy

fps = 0.0
# 连接传感器
pipeline = rs.pipeline()
# 配置传感器的参数
config = rs.config()
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)
# 获取第一个连接的设备
depth_sensor = profile.get_device().first_depth_sensor()
# 获取深度像素与真实距离的换算比例
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

frames = pipeline.wait_for_frames()
color = frames.get_color_frame()

# -----------------------------------------------
target_xy_pixel = [0, 0]
color_profile = color.get_profile()
cvsprofile = rs.video_stream_profile(color_profile)
color_intrin = cvsprofile.get_intrinsics()
color_intrin_part = [color_intrin.ppx, color_intrin.ppy, color_intrin.fx, color_intrin.fy]
# -----------------------------------------------

# target_depth = aligned_depth_frame.get_distance(target_xy_pixel[0], target_xy_pixel[1])
ppx = color_intrin_part[0]
ppy = color_intrin_part[1]
fx = color_intrin_part[2]
fy = color_intrin_part[3]

# -----------------------------------------------
frames = pipeline.wait_for_frames()
color = frames.get_color_frame()
# -----------------------------------------------

# 1米
clipping_distance_in_meters = 1
clipping_distance = clipping_distance_in_meters / depth_scale

# d2c 深度图对齐到彩色图
align_to = rs.stream.color
align = rs.align(align_to)

weights = '/home/rm/wust_ws/src/my_roborts_camera/src/weights/527-2587-300.pt'
device = select_device("0,1")
model = attempt_load(weights, map_location=device)
model.half()

rospy.init_node('armor_position_node', anonymous=True)
# 创建一个Publisher，发布名为/car_position_info的topic，消息类型为Float32MultiArray，队列长度10
position_info_pub = rospy.Publisher('/armor_position_info', armor_detecte_result, queue_size=10)


def detect(color_frame, aligned_depth_frame, color_intrin_part, model, mode=1):
    global target_xy_pixel, position_info_pub
    # create a car_armor_position msg class
    r_car_armor_Msgs = armor_detecte_result()
    b_car_armor_Msgs = armor_detecte_result()
    # record the enemy armor position
    temp_array = []

    out = "inference/output"
    img0 = color_frame
    imgsz = 416

    # Initialize
    # set_logging()
    device = select_device("0,1")
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    # model = attempt_load(weights, map_location=device)  # load FP32 model
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    if half:
        model.half()  # to FP16
    # Get names and colors
    names = ['BLUE', 'RED']
    # colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
    colors = [[0, 0, 222], [222, 0, 0]]

    # Run inference
    t0 = time.time()

    img = letterbox(img0, new_shape=imgsz)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, 0.5, 0.5, None, agnostic=False)
    t2 = time_synchronized()
    # Process detections
    r_car_armor_Msgs.is_detected = False
    r_car_armor_Msgs.color = r_car_armor_Msgs.RED
    b_car_armor_Msgs.is_detected = False
    b_car_armor_Msgs.color = b_car_armor_Msgs.BLUE
    for i, det in enumerate(pred):  # detections per image
        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            # Write results
            ppx = color_intrin_part[0]
            ppy = color_intrin_part[1]
            fx = color_intrin_part[2]
            fy = color_intrin_part[3]
            for *xyxy, conf, cls in reversed(det):
                left, top, right, bottom = xyxy
                top = max(0, np.array(np.floor(top.detach().cpu() + 0.5)).astype('int32'))
                left = max(0, np.array(np.floor(left.detach().cpu() + 0.5)).astype('int32'))
                bottom = min(img0.shape[0], np.array(np.floor(bottom.detach().cpu() + 0.5)).astype('int32'))
                right = min(img0.shape[1], np.array(np.floor(right.detach().cpu() + 0.5)).astype('int32'))
                if(int(cls) == 1):
                    r_car_armor_Msgs.is_detected = True
                    if aligned_depth_frame and color_intrin_part:
                        width = aligned_depth_frame.width
                        height = aligned_depth_frame.height
                        if mode == 1:
                            center_x = int(round((left + right) / 2))
                            center_y = int(round((top + bottom) / 2))

                            center_x = min(max(1, center_x), width - 1)
                            center_y = min(max(1, center_y), height - 1)

                            target_xy_pixel = [center_x, center_y]
                            target_depth = aligned_depth_frame.get_distance(target_xy_pixel[0], target_xy_pixel[1])
                            strDistance = "%.2f m" % target_depth
                            target_xy_true = [(target_xy_pixel[0] - ppx) * target_depth / fx,
                                              (target_xy_pixel[1] - ppy) * target_depth / fy]

                            r_car_armor_Msgs.armor_x = target_xy_true[0] * 100
                            r_car_armor_Msgs.armor_y = target_xy_true[1] * 100
                            r_car_armor_Msgs.armor_z = target_depth * 100
                    else:
                        strDistance = "\n 0 m"
                    label = '{} {:.2f}'.format(names[int(cls)], conf)
                    plot_one_box(xyxy, img0, label=label, strDistance=strDistance, color=colors[int(cls)],
                                     line_thickness=3)


                elif(int(cls) == 0):
                    b_car_armor_Msgs.is_detected = True
                    if aligned_depth_frame and color_intrin_part:
                        width = aligned_depth_frame.width
                        height = aligned_depth_frame.height
                        if mode == 1:
                            center_x = int(round((left + right) / 2))
                            center_y = int(round((top + bottom) / 2))

                            center_x = min(max(1, center_x), width - 1)
                            center_y = min(max(1, center_y), height - 1)

                            target_xy_pixel = [center_x, center_y]
                            target_depth = aligned_depth_frame.get_distance(target_xy_pixel[0], target_xy_pixel[1])
                            strDistance = "%.2f m" % target_depth
                            target_xy_true = [(target_xy_pixel[0] - ppx) * target_depth / fx,
                                              (target_xy_pixel[1] - ppy) * target_depth / fy]

                            b_car_armor_Msgs.armor_x = target_xy_true[0] * 100
                            b_car_armor_Msgs.armor_y = target_xy_true[1] * 100
                            b_car_armor_Msgs.armor_z = target_depth * 100
                
                            
                    else:
                        strDistance = "\n 0 m"
                    label = '{} {:.2f}'.format(names[int(cls)], conf)
                    plot_one_box(xyxy, img0, label=label, strDistance=strDistance, color=colors[int(cls)],
                                     line_thickness=3)
    position_info_pub.publish(r_car_armor_Msgs)
    position_info_pub.publish(b_car_armor_Msgs)


def main():
    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        # 采集一帧数据，并且对齐图像
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        # 获取对齐后的深度图
        aligned_depth_frame = aligned_frames.get_depth_frame()
        if not aligned_depth_frame:
            continue
        depth_frame = np.asanyarray(aligned_depth_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.07), cv2.COLORMAP_JET)
        # applyColorMap 颜色映射函数

        color_frame = aligned_frames.get_color_frame()
        # -----------------------------------------------

        if not color_frame:
            continue
        color_frame = np.asanyarray(color_frame.get_data())
        # 格式转变 BGR2RGB
        color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        # 转变成Image

        # 进行检测
        detect(color_frame, aligned_depth_frame, color_intrin_part, model=model, mode=1)

        # RGB2BGR满足opencv显示格式
        color_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("Result", color_frame)
        # cv2.imshow('depth', depth_colormap)
    pipeline.stop()


try:
    main()
finally:
    cv2.destroyAllWindows()

