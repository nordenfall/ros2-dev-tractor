#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD (x y z) -> проекция на изображение по K/D и экстринсикам T_cam<-lidar,
с правильными осями, фильтром Z>0 и раскраской точек (JET) по глубине или дальности.
"""
import os, sys, argparse, yaml, cv2, numpy as np
from glob import glob

def latest(pat):
    f = sorted(glob(pat))
    return f[-1] if f else ''

def parse_args():
    ws = os.path.expanduser('~/ws')
    ap = argparse.ArgumentParser()
    ap.add_argument('--img',     default=latest(os.path.join(ws,'data/rellis/samples/frame_*.png')))
    ap.add_argument('--pcd',     default=latest(os.path.join(ws,'data/rellis/samples/cloud_*.pcd')))
    ap.add_argument('--camyaml', default=os.path.join(ws,'src/cloud2seg/config/calib/rellis_camera_info.yaml'))
    ap.add_argument('--extyaml', default=os.path.join(ws,'src/cloud2seg/config/calib/rellis_extrinsics.yaml'))
    ap.add_argument('--out',     default=os.path.join(ws,'data/rellis/samples/overlay.png'))
    ap.add_argument('--outdbg',  default=os.path.join(ws,'data/rellis/samples/overlay_debug.png'))
    ap.add_argument('--thick',   type=int, default=3)
    ap.add_argument('--nodist',  action='store_true')
    ap.add_argument('--opt',     choices=['I','REP105'], default='I')
    ap.add_argument('--extr',    choices=['direct','inverse'], default='direct')
    ap.add_argument('--colormode', choices=['z','range'], default='z')
    ap.add_argument('--maxpts',  type=int, default=0)
    ap.add_argument('--zmin',    type=float, default=0.05)
    ap.add_argument('--zmax',    type=float, default=200.0)
    ap.add_argument('--rmax',    type=float, default=0.0)
    return ap.parse_args()

def read_cam_yaml(path):
    cam = yaml.safe_load(open(path,'r'))
    K = np.array(cam['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
    D = np.array(cam['distortion_coefficients']['data'], dtype=np.float64).reshape(-1,)
    return K, D

def read_ext_yaml(path):
    ext = yaml.safe_load(open(path,'r'))
    t = np.array(ext['translation_m'], dtype=np.float64).reshape(3,1)
    q = np.array(ext['quaternion_xyzw'], dtype=np.float64)  # XYZW
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

def read_pcd_xyz(path):
    pts=[]
    with open(path,'r') as f:
        head=True
        for line in f:
            if head:
                if line.strip().upper().startswith('DATA'):
                    head=False
                continue
            sp=line.strip().split()
            if len(sp)>=3:
                x,y,z = map(float, sp[:3])
                if (x!=0.0) or (y!=0.0) or (z!=0.0):
                    pts.append((x,y,z))
    return np.asarray(pts, dtype=np.float64)

def apply_colormap(vals, vmin=None, vmax=None):
    """vals -> [0..1] -> JET -> (N,3) BGR"""
    vals = np.asarray(vals, dtype=np.float64)
    if vmin is None: vmin = np.percentile(vals, 5)
    if vmax is None: vmax = np.percentile(vals, 95)
    if vmax <= vmin: vmax = vmin + 1e-6
    x = np.clip((vals - vmin) / (vmax - vmin), 0, 1)
    cm = cv2.applyColorMap((x*255).astype(np.uint8), cv2.COLORMAP_JET)  # (N,1,3)
    cm = cm.reshape(-1,3)  # <-- ключевая правка: сделать (N,3)
    return cm, float(vmin), float(vmax)

def draw_legend(img, vmin, vmax, label=''):
    legend = np.linspace(0,255,256,dtype=np.uint8)[:,None]
    legend = cv2.applyColorMap(legend, cv2.COLORMAP_JET)
    leg = cv2.resize(legend, (30,256), interpolation=cv2.INTER_NEAREST)
    H, W = img.shape[:2]
    y0, x0 = 10, 10
    y1, x1 = y0+256, x0+30
    if y1 <= H and x1 <= W:
        img[y0:y1, x0:x1] = leg
        cv2.putText(img, f'{vmin:.1f}', (x0+2, y0+12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
        cv2.putText(img, f'{vmax:.1f}', (x0+2, y0+246), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
        if label:
            cv2.putText(img, label, (x0, y1+18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

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
    R_cam_lidar = quat_xyzw_to_R(q_cam_lidar)

    if a.opt == 'I':
        R_opt_cam = np.eye(3, dtype=np.float64)
    else:
        R_opt_cam = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=np.float64)

    if a.extr == 'direct':
        R_cam_l, t_cam_l = R_cam_lidar, t_cam_lidar
    else:
        R_cam_l, t_cam_l = R_cam_lidar.T, -R_cam_lidar.T @ t_cam_lidar

    R_opt_lidar = R_opt_cam @ R_cam_l
    t_opt_lidar = R_opt_cam @ t_cam_l

    P_l = read_pcd_xyz(a.pcd)
    if a.maxpts>0 and P_l.shape[0] > a.maxpts:
        stride = max(1, P_l.shape[0] // a.maxpts)
        P_l = P_l[::stride, :]

    Xc = (R_opt_lidar @ P_l.T) + t_opt_lidar
    Zc = Xc[2, :]
    Rng = np.linalg.norm(P_l, axis=1)

    mask = (Zc > a.zmin)
    if a.zmax > a.zmin:
        mask &= (Zc < a.zmax)
    if a.rmax > 0.0:
        mask &= (Rng < a.rmax)
    if not np.any(mask):
        print('[FATAL] после фильтров нет точек'); sys.exit(2)

    rvec, _ = cv2.Rodrigues(R_opt_lidar)
    P_obj = P_l[mask, :].reshape(-1,1,3).astype(np.float64)
    uv, _  = cv2.projectPoints(P_obj, rvec, t_opt_lidar, K, D)
    uv     = uv.reshape(-1,2)
    Zc_m   = Zc[mask]
    Rng_m  = Rng[mask]

    uvi = np.round(uv).astype(int)
    inframe = (uvi[:,0]>=0)&(uvi[:,0]<W)&(uvi[:,1]>=0)&(uvi[:,1]<H)
    uvi = uvi[inframe]
    Zc_m = Zc_m[inframe]
    Rng_m = Rng_m[inframe]

    vals = Zc_m if a.colormode=='z' else Rng_m
    colors, vmin, vmax = apply_colormap(vals)  # (N,3) uint8

    overlay = img.copy()
    debug   = np.full_like(img, 255)
    for (u,v), c in zip(uvi, colors):
        u=int(u); v=int(v)
        b,g,r = int(c[0]), int(c[1]), int(c[2])
        cv2.circle(overlay,(u,v), a.thick, (b,g,r), -1)
        cv2.circle(debug,  (u,v), a.thick, (b,g,r), -1)

    draw_legend(overlay, vmin, vmax, 'Z depth (m)' if a.colormode=='z' else 'Range (m)')
    draw_legend(debug,   vmin, vmax, 'Z depth (m)' if a.colormode=='z' else 'Range (m)')

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    cv2.imwrite(a.out, overlay)
    cv2.imwrite(a.outdbg, debug)

    print(f'[MODE] opt={a.opt} extr={a.extr} nodist={a.nodist} colormode={a.colormode}')
    print(f'[STAT] projected={len(uvi)} | img={a.img}')
    print(f'[LIMS] Zc[min..max]=({Zc_m.min():.2f}..{Zc_m.max():.2f})  R[min..max]=({Rng_m.min():.2f}..{Rng_m.max():.2f})')
    print(f'[OUT] overlay={a.out}')
    print(f'[OUT] debug  ={a.outdbg}')

if __name__ == '__main__':
    main()
