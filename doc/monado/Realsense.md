# Using a RealSense Camera

After making sure that everything works by running the EuRoC datasets, it should
be possible to use the `realsense` driver from Monado to get any RealSense
camera that has an IMU and one or more cameras to get tracked with SLAM.
However, this was only tested on a D455, so if you are having problems with
another device, please open an issue. Also, open an issue if you manage to make
it work with other devices so that I can add it to this README.

## Index

- [Using a RealSense Camera](#using-a-realsense-camera)
  - [Index](#index)
  - [Overview of the Setup (D455)](#overview-of-the-setup-d455)
    - [SLAM-Tracked RealSense Driver](#slam-tracked-realsense-driver)
    - [RealSense-Tracked Qwerty Driver](#realsense-tracked-qwerty-driver)
  - [Non-D455 RealSense Devices](#non-d455-realsense-devices)
    - [Configuring the RealSense Pipeline](#configuring-the-realsense-pipeline)
    - [Configuring Basalt](#configuring-basalt)

## Overview of the Setup (D455)

Let's first assume you have a RealSense D455, which is the one that works with
the defaults. Even if you have another RealSense device follow this section, you
might at least get something working, although not at its best.

### SLAM-Tracked RealSense Driver

Set these environment variables:

- `export RS_HDEV_LOG=debug`: Make our realsense device logs more verbose
- `export RS_SOURCE_INDEX=0`: Indicate that we want to use the first RealSense device connected as data source
- `export RS_TRACKING=2`: Only try to use "host-slam". See other options
  [here](https://gitlab.freedesktop.org/mateosss/monado/-/blob/64e70e76ad6d47e4bd1a0dfa164bff8597a50ce8/src/xrt/drivers/realsense/rs_prober.c#L33-39).
- `export SLAM_CONFIG=$bsltdeps/basalt/data/monado/d455.toml`:
  Configuration file for Basalt and the D455.

### RealSense-Tracked Qwerty Driver

You now have a RealSense device that you can use to track another device, for
example, let's track a keyboard-and-mouse controlled HMD provided by the
`qwerty` driver.

Set these environment variables to enable the qwerty driver and stream to the
SLAM system on start:

```bash
export QWERTY_ENABLE=true QWERTY_COMBINE=true SLAM_SUBMIT_FROM_START=true
```

And then modify your tracking overrides in your monado configuration file
(`~/.config/monado/config_v0.json`) by updating the json object with:

```js
{
  "tracking": {
    "tracking_overrides": [
      {
        "target_device_serial": "Qwerty HMD", // Or "Qwerty Left Controller"
        "tracker_device_serial": "Intel RealSense Host-SLAM",
        "type": "direct",
        "offset": {
          "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 },
          "position": { "x": 0, "y": 0, "z": 0 }
        },
        "xrt_input_name": "XRT_INPUT_GENERIC_TRACKER_POSE"
      }
    ],
  }
}
```

And that's it! You can now start an OpenXR application with Monado and get your
view tracked with your D455 camera.

## Non-D455 RealSense Devices

While I was unable to test other devices because I don't have access to them, it
should be possible to make them work by:

### Configuring the RealSense Pipeline

[These
fields](https://gitlab.freedesktop.org/mateosss/monado/-/blob/9e1b7e2203ef49abb939cc8fc92afa16fcc9cb3a/src/xrt/drivers/realsense/rs_hdev.c#L118-129)
determine your RealSense streaming configuration, and
[these](https://gitlab.freedesktop.org/mateosss/monado/-/blob/b26a6023226a4623381215fc159da3b4bcb27c9b/src/xrt/drivers/realsense/rs_hdev.c#L47-61)
are their current defaults that work on a D455. You can change those fields by
setting any of them in your `config_v0.json` inside a `config_realsense_hdev`
field. Also note that as we already set `RS_HDEV_LOG=debug`, you should see the
values they are currently taking at the start of Monado.

For example, let's say you have a realsense device which has two fisheye cameras
that support streaming 640x360 at 30fps (a T265 I think), then a configuration
like this should work:

```js
"config_realsense_hdev": {
  "stereo": true,
  "video_format": 9, // 9 gets casted to RS2_FORMAT_Y8 (see https://git.io/Jzkfw), grayscale
  "video_width": 640, // I am assuming the T265 supports 640x360 streams at 30fps
  "video_height": 360,
  "video_fps": 30,
  "gyro_fps": 0, // 0 indicates any
  "accel_fps": 0,
  "stream_type": 4, // 4 gets casted to RS2_STREAM_FISHEYE (see https://git.io/Jzkvq)
  "stream1_index": -1, // If there were more than one possible stream with these properties select them, -1 is for auto
  "stream2_index": -1,
}
```

The particular values you could set here are very dependent on your camera. I
recommend seeing the values that get output by running the [rs-sensor-control
example](https://dev.intelrealsense.com/docs/rs-sensor-control) from the
RealSense API.

### Configuring Basalt

As you might've noticed, we set `SLAM_CONFIG` to
`$bsltdeps/basalt/data/monado/d455.toml` which is [this](data/monado/d455.toml)
config file that I added for the D455. This file points to a [calibration
file](data/d455_calib.json) and a [VIO configuration
file](data/euroc_config.json).

For the tracking to be as good as possible you should set the
intrinsics/extrinsics of the device in a similar calibration file and point to
it with the `SLAM_CONFIG` config file. You can obtain that information from the
previously mentioned
[rs-sensor-control](https://dev.intelrealsense.com/docs/rs-sensor-control)
utility. Issues like [this
(T265)](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and [this
(D435)](https://gitlab.com/VladyslavUsenko/basalt/-/issues/50) provide
configuration files tried by other users. Additionally Basalt provides custom
[calibration
tools](https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md)
that can work for any camera-IMU setup or tools like
[`basalt_rs_t265_record`](https://gitlab.freedesktop.org/mateosss/basalt/-/blob/5a365bf6fb14ce5b044b76f742337e1d6865557e/src/rs_t265_record.cpp#L207) or the `euroc_recorder` in Monado
that can help creating an initial calibration file for RealSense devices.
