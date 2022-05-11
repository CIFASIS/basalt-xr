#!/usr/bin/env python3

# Run with ./wmr2bslt_calib.py your_wmrcalib.json > your_calib.json

import json
import argparse
import numpy as np
from numpy.linalg import inv

from math import sqrt


def get(j, name):
    assert name in ["HT0", "HT1", "Gyro", "Accelerometer"]
    is_imu = name in ["Gyro", "Accelerometer"]
    calib = j["CalibrationInformation"]
    sensors = calib["InertialSensors" if is_imu else "Cameras"]
    name_key = "SensorType" if is_imu else "Location"
    sensor = next(filter(lambda s: s[name_key].endswith(name), sensors))
    return sensor


def rt2mat(rt):
    R33 = np.array(rt["Rotation"]).reshape(3, 3)
    t31 = np.array(rt["Translation"]).reshape(3, 1)
    T34 = np.hstack((R33, t31))
    T44 = np.vstack((T34, [0, 0, 0, 1]))
    return T44


def rmat2quat(r):
    w = sqrt(1 + r[0, 0] + r[1, 1] + r[2, 2]) / 2
    w4 = 4 * w
    x = (r[2, 1] - r[1, 2]) / w4
    y = (r[0, 2] - r[2, 0]) / w4
    z = (r[1, 0] - r[0, 1]) / w4
    return np.array([x, y, z, w])


def project(intrinsics, x, y, z):
    fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2 = (
        intrinsics["fx"],
        intrinsics["fy"],
        intrinsics["cx"],
        intrinsics["cy"],
        intrinsics["k1"],
        intrinsics["k2"],
        intrinsics["k3"],
        intrinsics["k4"],
        intrinsics["k5"],
        intrinsics["k6"],
        intrinsics["p1"],
        intrinsics["p2"],
    )

    xp = x / z
    yp = y / z
    r2 = xp * xp + yp * yp
    cdist = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (
        1 + r2 * (k4 + r2 * (k5 + r2 * k6))
    )
    deltaX = 2 * p1 * xp * yp + p2 * (r2 + 2 * xp * xp)
    deltaY = 2 * p2 * xp * yp + p1 * (r2 + 2 * yp * yp)
    xpp = xp * cdist + deltaX
    ypp = yp * cdist + deltaY
    u = fx * xpp + cx
    v = fy * ypp + cy
    return u, v


def extrinsics(j, cam):
    # NOTE: The `Rt` field seems to be a transform from the sensor to HT0 (i.e.,
    # from HT0 space to sensor space). For basalt we need the transforms
    # expressed w.r.t IMU origin.

    # NOTE: The gyro and magnetometer translations are 0, probably because an
    # HMD is a rigid body. Therefore the accelerometer is considered as the IMU
    # origin.

    imu = get(j, "Accelerometer")
    T_i_c0 = rt2mat(imu["Rt"])

    T = None
    if cam == "HT0":
        T = T_i_c0
    elif cam == "HT1":
        cam1 = get(j, "HT1")
        T_c1_c0 = rt2mat(cam1["Rt"])
        T_c0_c1 = inv(T_c1_c0)
        T_i_c1 = T_i_c0 @ T_c0_c1
        T = T_i_c1
    else:
        assert False

    q = rmat2quat(T[0:3, 0:3])
    p = T[0:3, 3]
    return {
        "px": p[0],
        "py": p[1],
        "pz": p[2],
        "qx": q[0],
        "qy": q[1],
        "qz": q[2],
        "qw": q[3],
    }


def resolution(j, cam):
    camera = get(j, cam)
    width = camera["SensorWidth"]
    height = camera["SensorHeight"]
    return [width, height]


def intrinsics(j, cam):
    # https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/2feb3425259bf803749065bb6d628c6c180f8e77/include/k4a/k4atypes.h#L1024-L1046
    camera = get(j, cam)
    model_params = camera["Intrinsics"]["ModelParameters"]
    assert (
        camera["Intrinsics"]["ModelType"]
        == "CALIBRATION_LensDistortionModelRational6KT"
    )
    width = camera["SensorWidth"]
    height = camera["SensorHeight"]
    return {
        "camera_type": "pinhole-radtan8",
        "intrinsics": {
            "fx": model_params[2] * width,
            "fy": model_params[3] * height,
            "cx": model_params[0] * width,
            "cy": model_params[1] * height,
            "k1": model_params[4],
            "k2": model_params[5],
            "p1": model_params[13],
            "p2": model_params[12],
            "k3": model_params[6],
            "k4": model_params[7],
            "k5": model_params[8],
            "k6": model_params[9],
            "rpmax": model_params[14],
        },
    }


def view_offset(j):
    """
    This is a very rough offset in pixels between the two cameras. Originally we
    needed to manually estimate it like explained and shown here
    https://youtu.be/jyQKjyRVMS4?t=670.
    With this calculation we get a similar number without the need to open Gimp.

    In reality this offset changes based on distance to the point, nonetheless
    it helps to get some features tracked in the right camera.
    """

    # Rough approximation of how far from the cameras features will likely be in your room
    DISTANCE_TO_WALL = 2  # In meters

    cam1 = get(j, "HT1")
    width = cam1["SensorWidth"]
    height = cam1["SensorHeight"]
    cam1_intrinsics = intrinsics(j, "HT1")["intrinsics"]
    T_c1_c0 = rt2mat(cam1["Rt"])  # Maps a point in c0 space to c1 space
    p = np.array([0, 0, DISTANCE_TO_WALL, 1])  # Fron tof c0, in homogeneous coords
    p_in_c1 = T_c1_c0 @ p  # Point in c1 coordinates
    u, v = project(cam1_intrinsics, *p_in_c1[0:3])
    view_offset = [width / 2 - u, height / 2 - v]  # We used a point in the middle of c0
    return view_offset


def calib_accel_bias(j):
    # https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/2feb3425259bf803749065bb6d628c6c180f8e77/include/k4ainternal/calibration.h#L48-L77
    # https://vladyslavusenko.gitlab.io/basalt-headers/classbasalt_1_1CalibAccelBias.html#details
    # https://gitlab.com/VladyslavUsenko/basalt-headers/-/issues/8
    accel = get(j, "Accelerometer")
    bias = accel["BiasTemperatureModel"]
    align = accel["MixingMatrixTemperatureModel"]
    return [
        -bias[0 * 4],
        -bias[1 * 4],
        -bias[2 * 4],
        align[0 * 4] - 1,  # [0, 0]
        align[3 * 4],  # [1, 0]
        align[6 * 4],  # [2, 0]
        align[4 * 4] - 1,  # [1, 1]
        align[7 * 4],  # [2, 1]
        align[8 * 4] - 1,  # [2, 2]
    ]


def calib_gyro_bias(j):
    # https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/2feb3425259bf803749065bb6d628c6c180f8e77/include/k4ainternal/calibration.h#L48-L77
    # https://vladyslavusenko.gitlab.io/basalt-headers/classbasalt_1_1CalibGyroBias.html#details
    gyro = get(j, "Gyro")
    bias = gyro["BiasTemperatureModel"]
    align = gyro["MixingMatrixTemperatureModel"]
    return [
        -bias[0 * 4],
        -bias[1 * 4],
        -bias[2 * 4],
        align[0 * 4] - 1,  # [0, 0]
        align[3 * 4],  # [1, 0]
        align[6 * 4],  # [2, 0]
        align[1 * 4],  # [0, 1]
        align[4 * 4] - 1,  # [1, 1]
        align[7 * 4],  # [2, 1]
        align[2 * 4],  # [0, 2]
        align[5 * 4],  # [1, 2]
        align[8 * 4] - 1,  # [2, 2]
    ]


def noise_std(j, name):
    imu = get(j, name)
    return imu["Noise"][0:3]


def bias_std(j, name):
    imu = get(j, name)
    return list(map(sqrt, imu["BiasUncertainty"]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("wmr_json_file", help="Input WMR json calibration file")
    args = parser.parse_args()
    in_fn = args.wmr_json_file

    with open(in_fn) as f:
        j = json.load(f)

    # We get 250 packets with 4 samples each per second, totalling 1000 samples per second.
    # But in monado we just average those 4 samples to reduce the noise. So we have 250hz.
    IMU_UPDATE_RATE = 250

    out_calib = {
        "value0": {
            "T_imu_cam": [extrinsics(j, "HT0"), extrinsics(j, "HT1")],
            "intrinsics": [intrinsics(j, "HT0"), intrinsics(j, "HT1")],
            "resolution": [resolution(j, "HT0"), resolution(j, "HT1")],
            "calib_accel_bias": calib_accel_bias(j),
            "calib_gyro_bias": calib_gyro_bias(j),
            "imu_update_rate": IMU_UPDATE_RATE,
            "accel_noise_std": noise_std(j, "Accelerometer"),
            "gyro_noise_std": noise_std(j, "Gyro"),
            "accel_bias_std": bias_std(j, "Accelerometer"),
            "gyro_bias_std": bias_std(j, "Gyro"),
            "cam_time_offset_ns": 0,
            "view_offset": view_offset(j),
            "vignette": [],
        }
    }

    print(json.dumps(out_calib, indent=4))


if __name__ == "__main__":
    main()
