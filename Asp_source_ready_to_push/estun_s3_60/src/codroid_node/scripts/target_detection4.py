#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
target_detection4.py (Robust OBB + continuous yaw)

目标：
- 框选逻辑为旋转外接矩形（OBB）：minAreaRect + boxPoints（图1效果）
- 输出角度为图像平面内摆放朝向：长边相对图像 x 轴方向角，deg in [0,180)
- 避免“只有 0/90”与方向严重偏移：剔除与 ROI 边界连通的边缘/轮廓，防止 ROI 方框主导方向
- /target_yaw_deg 发布（std_msgs/Float32, deg）

依赖：
- 仍使用你工程里的 yolov5: models.experimental / utils.general
"""

import rclpy
import cv2 as cv
import numpy as np
import fcntl
import os
import torch
torch.set_num_threads(16)
torch.set_num_interop_threads(1)
import json
import tempfile
import time
import math
import threading
from rclpy.node import Node
from torchvision import transforms
from std_msgs.msg import Float32

from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# -------------------------
# 路径与原子写入
# -------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LATEST_POINT_PATH = os.path.join(SCRIPT_DIR, "data", "latest_point.json")
LATEST_DETECTIONS_PATH = os.path.join(SCRIPT_DIR, "data", "latest_detections.json")
os.makedirs(os.path.dirname(LATEST_POINT_PATH), exist_ok=True)
os.makedirs(os.path.dirname(LATEST_DETECTIONS_PATH), exist_ok=True)


def atomic_write_json(path: str, data: dict):
    d = os.path.dirname(path) or "."
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix="tmp_", suffix=".json", dir=d)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except Exception:
            pass


def write_latest_point_m(x=None, y=None, z=None, path=LATEST_POINT_PATH):
    atomic_write_json(path, {"x_m": x, "y_m": y, "z_m": z, "ts": time.time()})


# -------------------------
# 类别
# -------------------------
def load_class_names(file_path="my.txt"):
    try:
        with open(file_path, "r", encoding="utf-8-sig") as f:
            names = [ln.strip() for ln in f if ln.strip()]
        print(f"[classes] {file_path}: {len(names)} classes")
        return names
    except Exception as e:
        print(f"[classes] load failed: {e}, fallback ['object']")
        return ["object"]


class_names = load_class_names("my.txt")
rng = np.random.default_rng(2025)
COLORS = (rng.random((max(len(class_names), 1), 3)) * 255).astype(np.uint8)


# -------------------------
# YOLOv5
# -------------------------
try:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[yolo] device: {device}")
    model = attempt_load("best.pt", map_location=device)
    model.to(device).eval()

    names_in_model = getattr(model, "names", None) or getattr(getattr(model, "model", None), "names", None)
    if isinstance(names_in_model, dict):
        names_in_model = [names_in_model[i] for i in range(len(names_in_model))]
    if isinstance(names_in_model, (list, tuple)) and len(names_in_model) > 0 and len(names_in_model) != len(class_names):
        class_names = list(names_in_model)
        COLORS = (rng.random((len(class_names), 3)) * 255).astype(np.uint8)

    print("[yolo] model loaded, classes =", len(class_names))
except Exception as e:
    print(f"[yolo] load failed: {e}")
    raise SystemExit(1)

preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Resize((320, 320)),
])


# -------------------------
# OBB / 辅助函数
# -------------------------
def _morph_ell(k: int):
    k = int(max(1, k))
    return cv.getStructuringElement(cv.MORPH_ELLIPSE, (k, k))


def _wrap_180(a_deg: float) -> float:
    a = float(a_deg) % 180.0
    if a < 0:
        a += 180.0
    return a


def _choose_nearest_angle_180(curr_deg: float, prev_deg: float) -> float:
    """
    在 0~180 的等价角空间做连续化：选择最接近 prev 的等价表示
    """
    a = float(curr_deg)
    p = float(prev_deg)
    cands = [a, a + 180.0, a - 180.0]
    best = min(cands, key=lambda x: abs(x - p))
    return _wrap_180(best)


def _normalize_minarearect_angle(rect):
    """
    OpenCV rect angle 常见为 [-90,0)，这里统一为：
    - 以长边为方向
    - 相对图像 x 轴方向角 in [0,180)
    """
    (cx, cy), (w, h), ang = rect
    w = float(w)
    h = float(h)
    ang = float(ang)
    if w < h:
        ang += 90.0
        w, h = h, w
    return (cx, cy), (w, h), (ang % 180.0)


def _edges_remove_border_components(edges_u8: np.ndarray, border: int = 4):
    """
    剔除与 ROI 边界连通的边缘连通域，避免 ROI 方框/背景边缘主导 minAreaRect
    """
    if edges_u8 is None:
        return None
    h, w = edges_u8.shape[:2]
    if h < 8 or w < 8:
        return edges_u8

    bin_u8 = (edges_u8 > 0).astype(np.uint8)
    num, lab, stats, _ = cv.connectedComponentsWithStats(bin_u8, connectivity=8)
    if num <= 1:
        return edges_u8

    b = int(max(1, border))
    border_mask = np.zeros((h, w), dtype=bool)
    border_mask[:b, :] = True
    border_mask[-b:, :] = True
    border_mask[:, :b] = True
    border_mask[:, -b:] = True

    touch_ids = np.unique(lab[border_mask])

    out = np.zeros_like(edges_u8)
    for i in range(1, num):
        if i in touch_ids:
            continue
        out[lab == i] = 255
    return out


def _obb_from_contour_global(cnt_g):
    if cnt_g is None or len(cnt_g) < 5:
        return None
    rect = cv.minAreaRect(cnt_g)
    (cx, cy), (w, h), ang = _normalize_minarearect_angle(rect)
    if w * h < 800.0:
        return None
    box = cv.boxPoints(((cx, cy), (w, h), ang)).astype(np.int32)
    return {
        "center": {"x": float(cx), "y": float(cy)},
        "size": {"w": float(w), "h": float(h)},
        "angle_deg": float(ang),
        "box_pts": [{"x": int(p[0]), "y": int(p[1])} for p in box],
    }


def _obb_from_edges(roi_bgr: np.ndarray, x1: int, y1: int):
    """
    强兜底：靠边缘提取主轮廓 -> minAreaRect
    """
    if roi_bgr is None or roi_bgr.size == 0:
        return None

    gray = cv.cvtColor(roi_bgr, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5, 5), 0)

    # 你场景里（物体摆在平面上）边缘通常比 GrabCut 更稳定
    edges = cv.Canny(gray, 40, 140)

    # 去除与 ROI 边界连通的边缘（核心）
    edges = _edges_remove_border_components(edges, border=4)
    if edges is None or cv.countNonZero(edges) < 120:
        return None

    edges = cv.dilate(edges, _morph_ell(3), iterations=1)

    cnts, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    cnt = max(cnts, key=cv.contourArea)
    if cv.contourArea(cnt) < 600:
        return None

    # ROI -> global
    cnt_g = cnt.copy()
    cnt_g[:, 0, 0] += int(x1)
    cnt_g[:, 0, 1] += int(y1)

    return _obb_from_contour_global(cnt_g)


# -------------------------
# Node
# -------------------------
class YOLOV5ObjectDetection(Node):
    def __init__(self):
        super().__init__("yolov5_object_detection")
        self.bridge = CvBridge()

        self.depth_image = None
        self.depth_lock = threading.Lock()

        self.rgb_count = 0
        self.depth_count = 0
        self.last_point_write = 0.0

        self.prev_target = None
        self.prev_yaw_deg = None

        # 内参：你原工程若要 CameraInfo 动态更新，把 camera_info_received 改 False
        self.camera_info_received = True
        self.camera_matrix = np.array([
            [607.49244, 0, 324.9144],
            [0, 607.73413, 262.0061],
            [0, 0, 1],
        ], dtype=np.float32)

        # 深度单位（按你的设备）
        self.depth_scale = 0.001  # mm -> m（若你的深度本来就是 m，这里改 1.0）

        # 输出
        if not os.path.exists("pos.txt"):
            with open("pos.txt", "w") as f:
                f.write("none,0,0,0,0,0,0,0\n")

        # 订阅/发布
        self.depth_subscription = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 1
        )
        self.rgb_subscription = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.rgb_callback, 1
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.camera_info_callback, 10
        )

        self.debug_img_pub = self.create_publisher(Image, "/target_detection/debug_image", 10)
        self.yaw_pub = self.create_publisher(Float32, "/target_yaw_deg", 10)

        self.timer = self.create_timer(5.0, self.check_topics_status)
        self.get_logger().info("target_detection4: robust OBB + continuous yaw")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            try:
                self.camera_matrix = np.array(msg.k).reshape(3, 3).astype(np.float32)
                self.camera_info_received = True
                self.get_logger().info("camera intrinsics received")
            except Exception as e:
                self.get_logger().error(f"camera_info parse failed: {e}")

    def check_topics_status(self):
        self.get_logger().info(f"RGB:{self.rgb_count} Depth:{self.depth_count} intrinsics:{self.camera_info_received}")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.depth_lock:
                self.depth_image = depth.copy()
            self.depth_count += 1
        except Exception as e:
            self.get_logger().error(f"depth convert failed: {e}")

    def publish_debug_image(self, bgr_img):
        try:
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8"))
        except Exception:
            pass

    def pixel_to_camera_coordinates(self, px, py, depth_m):
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        Z = float(depth_m)
        X = (float(px) - cx) * Z / fx
        Y = (float(py) - cy) * Z / fy
        return float(X), float(Y), float(Z)

    def calculate_distance_and_angle(self, x, y, z):
        dist = math.sqrt(x * x + y * y + z * z)
        ha = math.degrees(math.atan2(x, z))
        va = math.degrees(math.atan2(y, z))
        return dist, ha, va

    def write_pos_txt(self, center_x, center_y, depth_value, class_name, camera_x, camera_y, camera_z, distance):
        try:
            with open("pos.txt", "w") as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                f.write(
                    f"{class_name},{int(center_x)},{int(center_y)},{int(depth_value)},"
                    f"{camera_x:.3f},{camera_y:.3f},{camera_z:.3f},{distance:.3f}\n"
                )
                fcntl.flock(f, fcntl.LOCK_UN)
        except Exception as e:
            self.get_logger().error(f"write pos.txt failed: {e}")

    def rgb_callback(self, msg: Image):
        self.rgb_count += 1
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            original_shape = color_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f"rgb convert failed: {e}")
            return

        with self.depth_lock:
            depth = None if self.depth_image is None else self.depth_image.copy()
        if depth is not None and depth.shape[:2] != color_image.shape[:2]:
            try:
                depth = cv.resize(depth, (color_image.shape[1], color_image.shape[0]))
            except Exception:
                depth = None

        # YOLO
        img_rgb = cv.cvtColor(color_image, cv.COLOR_BGR2RGB)
        input_tensor = preprocess(img_rgb).unsqueeze(0).to(device)

        with torch.no_grad():
            raw_pred = model(input_tensor)[0]

        preds = non_max_suppression(raw_pred, conf_thres=0.15, iou_thres=0.3,
                                    classes=None, agnostic=False, max_det=200)

        detected = []
        Himg, Wimg = color_image.shape[:2]

        for det in preds:
            if not len(det):
                continue
            det[:, :4] = scale_coords(input_tensor.shape[2:], det[:, :4], original_shape)

            for *xyxy, conf, cls in det:
                cls_id = int(cls.item() if hasattr(cls, "item") else cls)
                if not (0 <= cls_id < len(class_names)):
                    continue

                x1, y1, x2, y2 = [int(round(float(v))) for v in xyxy]
                x1 = max(0, min(x1, Wimg - 1))
                y1 = max(0, min(y1, Himg - 1))
                x2 = max(0, min(x2, Wimg - 1))
                y2 = max(0, min(y2, Himg - 1))
                if x2 <= x1 or y2 <= y1:
                    continue

                w0, h0 = x2 - x1, y2 - y1
                cx0, cy0 = x1 + w0 // 2, y1 + h0 // 2

                roi = color_image[y1:y2, x1:x2]

                # 关键：不再依赖“mask 成功才有 OBB”
                # mask 在你场景里经常会被 ROI 边界/背景污染，所以直接强兜底用 edges 生成 OBB
                obb = _obb_from_edges(roi, x1, y1)

                use_cx, use_cy = cx0, cy0
                yaw_deg = None
                if obb is not None:
                    use_cx = int(round(obb["center"]["x"]))
                    use_cy = int(round(obb["center"]["y"]))
                    yaw_deg = float(obb["angle_deg"])

                # 深度与三维点
                depth_raw = 0.0
                cam_x = cam_y = cam_z = 0.0
                dist_m = 0.0
                h_deg = v_deg = 0.0
                if depth is not None and 0 <= use_cy < depth.shape[0] and 0 <= use_cx < depth.shape[1]:
                    depth_raw = float(depth[use_cy, use_cx])
                    if np.isfinite(depth_raw) and depth_raw > 0:
                        depth_m = depth_raw * float(self.depth_scale)
                        cam_x, cam_y, cam_z = self.pixel_to_camera_coordinates(use_cx, use_cy, depth_m)
                        dist_m, h_deg, v_deg = self.calculate_distance_and_angle(cam_x, cam_y, cam_z)

                detected.append({
                    "class_id": cls_id,
                    "class_name": class_names[cls_id],
                    "confidence": float(conf),
                    "pixel": {"x": int(use_cx), "y": int(use_cy)},
                    "yaw_deg": float(yaw_deg) if yaw_deg is not None else None,
                    "depth_raw": float(depth_raw),
                    "camera": {"x_m": float(cam_x), "y_m": float(cam_y), "z_m": float(cam_z)},
                    "distance_m": float(dist_m),
                    "angles_deg": {"h": float(h_deg), "v": float(v_deg)},
                    "bbox_yolo": {"x": int(x1), "y": int(y1), "w": int(w0), "h": int(h0)},
                    "obb": obb,
                })

        atomic_write_json(LATEST_DETECTIONS_PATH, {"ts": time.time(), "num": len(detected), "detections": detected})

        # Top1：pos.txt + latest_point + /target_yaw_deg（角度连续化）
        if detected:
            detected.sort(key=lambda d: d["confidence"], reverse=True)
            top = detected[0]

            SMOOTH = 0.3
            if self.prev_target:
                cx = SMOOTH * top["pixel"]["x"] + (1 - SMOOTH) * self.prev_target["cx"]
                cy = SMOOTH * top["pixel"]["y"] + (1 - SMOOTH) * self.prev_target["cy"]
                cam_x_s = SMOOTH * top["camera"]["x_m"] + (1 - SMOOTH) * self.prev_target["cam_x"]
                cam_y_s = SMOOTH * top["camera"]["y_m"] + (1 - SMOOTH) * self.prev_target["cam_y"]
                cam_z_s = SMOOTH * top["camera"]["z_m"] + (1 - SMOOTH) * self.prev_target["cam_z"]
            else:
                cx, cy = float(top["pixel"]["x"]), float(top["pixel"]["y"])
                cam_x_s, cam_y_s, cam_z_s = float(top["camera"]["x_m"]), float(top["camera"]["y_m"]), float(top["camera"]["z_m"])

            self.prev_target = {"cx": cx, "cy": cy, "cam_x": cam_x_s, "cam_y": cam_y_s, "cam_z": cam_z_s}
            self.write_pos_txt(cx, cy, top["depth_raw"], top["class_name"], cam_x_s, cam_y_s, cam_z_s, top["distance_m"])

            now = time.time()
            if now - self.last_point_write >= 0.05:
                write_latest_point_m(cam_x_s, cam_y_s, cam_z_s)
                self.last_point_write = now

            if top.get("yaw_deg", None) is not None:
                yaw = float(top["yaw_deg"])
                if self.prev_yaw_deg is None:
                    self.prev_yaw_deg = yaw
                else:
                    yaw = _choose_nearest_angle_180(yaw, self.prev_yaw_deg)
                    self.prev_yaw_deg = yaw
                self.yaw_pub.publish(Float32(data=float(yaw)))
                top["yaw_deg"] = float(yaw)
        else:
            self.write_pos_txt(0, 0, 0, "none", 0, 0, 0, 0)
            self.prev_target = None
            self.prev_yaw_deg = None
            write_latest_point_m(None, None, None)

        # 可视化：OBB 优先（图1逻辑），失败才回退 AABB
        for t in detected:
            color = tuple(int(c) for c in COLORS[t["class_id"]].tolist())
            obb = t.get("obb", None)

            if obb is not None:
                pts = np.array([[p["x"], p["y"]] for p in obb["box_pts"]], dtype=np.int32).reshape(-1, 1, 2)
                cv.polylines(color_image, [pts], True, color, 2)

                x0 = int(np.min(pts[:, 0, 0]))
                y0 = int(np.min(pts[:, 0, 1]))
                yaw_show = t.get("yaw_deg", None)
                if yaw_show is None:
                    yaw_show = float(obb["angle_deg"])

                cv.putText(
                    color_image,
                    f"{t['class_name']} {t['confidence']:.2f} {yaw_show:.1f}deg",
                    (x0, max(0, y0 - 8)),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )

                # 方向箭头（沿长边方向）
                cx = int(round(obb["center"]["x"]))
                cy = int(round(obb["center"]["y"]))
                ang = math.radians(float(yaw_show))
                L = 90
                x2 = int(round(cx + L * math.cos(ang)))
                y2 = int(round(cy + L * math.sin(ang)))  # 图像 y 轴向下
                cv.arrowedLine(color_image, (cx, cy), (x2, y2), color, 2, tipLength=0.18)

            else:
                b = t["bbox_yolo"]
                cv.rectangle(color_image, (b["x"], b["y"]), (b["x"] + b["w"], b["y"] + b["h"]), color, 2)
                cv.putText(
                    color_image,
                    f"{t['class_name']} {t['confidence']:.2f}",
                    (b["x"], max(0, b["y"] - 7)),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

            cv.circle(color_image, (t["pixel"]["x"], t["pixel"]["y"]), 4, (255, 0, 0), -1)

        self.publish_debug_image(color_image)

        try:
            cv.imshow("target_detection4 (robust OBB + continuous yaw)", color_image)
            key = cv.waitKey(1)
            if key == ord("q") or key == 27:
                cv.destroyAllWindows()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"imshow failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YOLOV5ObjectDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
