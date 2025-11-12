#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, argparse, yaml, cv2, numpy as np, csv
from glob import glob

# ----------------- CLI -----------------
def parse_args():
    ws = os.path.expanduser('~/ws')
    def latest(pat): 
        f = sorted(glob(pat))
        return f[-1] if f else ''
    ap = argparse.ArgumentParser()
    ap.add_argument('--img',     default=latest(os.path.join(ws, 'data/rellis/samples/frame_*.png')))
    ap.add_argument('--pcd',     default=latest(os.path.join(ws, 'data/rellis/samples/cloud_*.pcd')))
    ap.add_argument('--camyaml', default=os.path.join(ws, 'src/cloud2seg/config/calib/rellis_camera_info.yaml'))
    ap.add_argument('--extyaml', default=os.path.join(ws, 'src/cloud2seg/config/calib/rellis_extrinsics.yaml'))
    ap.add_argument('--out',     default=os.path.join(ws, 'data/rellis/samples/overlay.png'))
    ap.add_argument('--outdbg',  default=os.path.join(ws, 'data/rellis/samples/overlay_debug.png'))
    ap.add_argument('--csv',     default=os.path.join(ws, 'data/rellis/samples/uv_samples.csv'))
    ap.add_argument('--maxpts',  type=int, default=0, help='лимит точек (0=все)')
    ap.add_argument('--nodist',  action='store_true', help='игнорировать дисторсию (D=0) для диагностики')
    ap.add_argument('--thick',   type=int, default=3,  help='радиус точки в пикселях')
    return ap.parse_args()

# --------------- IO helpers ---------------
def read_cam_yaml(path):
    cam = yaml.safe_load(open(path, 'r'))
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
    D = np.array(cam['distortion_coefficients']['data'], dtype=np.float64).reshape(-1,)
    return K, D

def read_ext_yaml(path):
    ext = yaml.safe_load(open(path, 'r'))
    t = np.array(ext['translation_m'], dtype=np.float64).reshape(3,1)
    q = np.array(ext['quaternion_xyzw'], dtype=np.float64)  # Порядок XYZW!
    return t, q

def quat_xyzw_to_R(q):
    q = q / np.linalg.norm(q)
    x,y,z,w = q
    xx,yy,zz = x*x, y*y, z*z
    xy,xz,yz = x*y, x*z, y*z
    wx,wy,wz = w*x, w*y, w*z
    return np.array([
        [1-2*(yy+zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz), 1-2*(xx+zz),   2*(yz - wx)],
        [2*(xz - wy), 2*(yz + wx),   1-2*(xx+yy)]
    ], dtype=np.float64)

def read_ascii_pcd_xyz(path):
    pts = []
    with open(path, 'r') as f:
        header = True
        for line in f:
            if header:
                if line.strip().upper().startswith('DATA'):
                    header = False
                continue
            sp = line.strip().split()
            if len(sp) >= 3:
                x,y,z = map(float, sp[:3])
                # пропускаем нули как невалидные возвраты
                if (x != 0.0) or (y != 0.0) or (z != 0.0):
                    pts.append((x,y,z))
    return np.asarray(pts, dtype=np.float64)  # (N,3)

# --------------- Main -----------------
def main():
    a = parse_args()
    assert os.path.isfile(a.img), f'no image: {a.img}'
    assert os.path.isfile(a.pcd), f'no pcd:   {a.pcd}'

    img = cv2.imread(a.img, cv2.IMREAD_COLOR);  assert img is not None
    H, W = img.shape[:2]

    K, D = read_cam_yaml(a.camyaml)
    if a.nodist:
        D = np.zeros_like(D)

    t_cam_lidar, q_cam_lidar = read_ext_yaml(a.extyaml)
    R_cam_lidar = quat_xyzw_to_R(q_cam_lidar)             # lidar -> camera

    # Переход к optical-осям (REP-105): x→right, y→down, z→forward
    R_opt_cam = np.array([[0,0,1],
                          [-1,0,0],
                          [0,-1,0]], dtype=np.float64)

    # Экстринсики в optical: T_opt<-lidar
    R_opt_lidar = R_opt_cam @ R_cam_lidar
    t_opt_lidar = R_opt_cam @ t_cam_lidar

    P_l = read_ascii_pcd_xyz(a.pcd)                       # (N,3) в lidar
    if a.maxpts > 0 and P_l.shape[0] > a.maxpts:
        step = max(1, P_l.shape[0] // a.maxpts)
        P_l = P_l[::step, :]

    # Быстрый контроль «Z>0» (в optical СК камеры)
    Xc = (R_opt_lidar @ P_l.T) + t_opt_lidar              # (3, N)
    Z = Xc[2, :]
    mask_front = Z > 0.05
    if not np.any(mask_front):
        print('[FATAL] нет точек с Z>0.05 в СК камеры — проверь экстринсики/оси/пару данных')
        sys.exit(2)

    # Проекция через OpenCV (исходные точки дают в СК объекта: lidar)
    rvec, _ = cv2.Rodrigues(R_opt_lidar)
    tvec     = t_opt_lidar
    P_obj = P_l[mask_front, :].astype(np.float64).reshape(-1,1,3)
    uv, _ = cv2.projectPoints(P_obj, rvec, tvec, K, D)    # (N,1,2)
    uv = uv.reshape(-1,2)

    # Диагностика
    uv_int = np.round(uv).astype(int)
    inmask = (uv_int[:,0] >= 0) & (uv_int[:,0] < W) & (uv_int[:,1] >= 0) & (uv_int[:,1] < H)
    print(f'[STAT] img={a.img}')
    print(f'[STAT] pcd={a.pcd} (used {P_l.shape[0]} pts; in-front {mask_front.sum()} pts)')
    print(f'[STAT] projected={uv.shape[0]} | in-frame={inmask.sum()}')
    if inmask.any():
        iu = uv[inmask,0]; iv = uv[inmask,1]
        print(f'[STAT] u[min..max]=({iu.min():.1f}..{iu.max():.1f})  v[min..max]=({iv.min():.1f}..{iv.max():.1f})')
        print('[STAT] sample uv in-frame (10):', np.round(uv[inmask][:10],1))

    # CSV 300 примеров
    os.makedirs(os.path.dirname(a.csv), exist_ok=True)
    with open(a.csv, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['u','v','in_frame'])
        n = min(300, uv.shape[0])
        for i in range(n):
            w.writerow([float(uv[i,0]), float(uv[i,1]), bool(inmask[i])])
    print('[OK] csv:', a.csv)

    # Рисуем (жирные точки + debug на белом)
    img_overlay = img.copy()
    debug = np.full_like(img, 255)
    drawn = 0
    for (u,v), ok in zip(uv, inmask):
        if not ok: 
            continue
        ui, vi = int(round(u)), int(round(v))
        cv2.circle(img_overlay, (ui,vi), a.thick, (255,0,255), -1)
        cv2.circle(debug,       (ui,vi), a.thick, (0,0,0),     -1)
        drawn += 1

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    cv2.imwrite(a.out, img_overlay)
    cv2.imwrite(a.outdbg, debug)
    print('[OK] overlay:', a.out)
    print('[OK] debug  :', a.outdbg)
    print(f'[DONE] drawn={drawn}, thick={a.thick}, nodist={a.nodist}')

if __name__ == '__main__':
    main()
