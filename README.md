# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) tuned for
tracking XR devices with [Monado](https://gitlab.freedesktop.org/monado/monado).
Many thanks to the Basalt authors.

## Installation

- **Ubuntu 22.04**: Download [latest .deb](https://gitlab.freedesktop.org/mateosss/basalt/-/releases) and install with

  ```bash
  sudo apt install -y ./basalt-monado-ubuntu-22.04-haswell-amd64.deb
  ```

- **From source (minimal)**
  ```bash
  git clone --recursive https://gitlab.freedesktop.org/mateosss/basalt.git
  cd basalt && ./scripts/install_deps.sh
  cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBASALT_BUILD_SHARED_LIBRARY_ONLY=on
  sudo cmake --build build --target install
  ```
- **From source (with extra binaries and debug symbols)**
  ```bash
  git clone --recursive https://gitlab.freedesktop.org/mateosss/basalt.git
  cd basalt && ./scripts/install_deps.sh
  cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBASALT_INSTANTIATIONS_DOUBLE=off -DBUILD_TESTS=off
  sudo cmake --build build --target install
  ```

## Usage

If you want to run OpenXR applications, you should now rebuild Monado so that it finds the Basalt shared library.

If you want to test whether everything is working you can download a sample euroc dataset like `V1_02_medium`:

```bash
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.zip
unzip -d V1_02_medium V1_02_medium.zip
```

- **Try it standalone with a dataset (requires extra binaries)**
  ```bash
  basalt_vio --show-gui 1 --dataset-path V1_02_medium/ --dataset-type euroc --cam-calib /usr/local/etc/basalt/euroc_ds_calib.json --config-path /usr/local/etc/basalt/euroc_config.json
  ```
- **Use a RealSense camera without Monado (requires extra binaries)**
  You'll need to calibrate your camera if you want the best results but meanwhile you can try with these calibration files instead.
  - RealSense D455 (and maybe also D435)
    ```bash
    basalt_rs_t265_vio --is-d455 --cam-calib /usr/local/etc/basalt/d455_calib.json --config-path /usr/local/etc/basalt/euroc_config.json
    ```
  - Realsense T265: Get t265_calib.json from [this issue](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and run
    ```bash
    basalt_rs_t265_vio --cam-calib t265_calib.json --config-path /usr/local/etc/basalt/euroc_config.json
    ```
- **Try it through `monado-cli` with a dataset**
  ```bash
  monado-cli slambatch V1_02_medium/ /usr/local/etc/basalt/euroc.toml results
  ```
- **Try it with `monado`, a dataset, and an OpenXR app**

  ```bash
  # Run monado-service with a fake "euroc device" driver
  export EUROC_PATH=V1_02_medium/ # dataset path
  export EUROC_HMD=false # false for controller tracking
  export EUROC_PLAY_FROM_START=true # produce samples right away
  export SLAM_CONFIG=/usr/local/etc/basalt/euroc.toml # includes calibration
  export SLAM_SUBMIT_FROM_START=true # consume samples right away
  export XRT_DEBUG_GUI=1 # enable monado debug ui
  monado-service &

  # Get and run a sample OpenXR application
  wget https://gitlab.freedesktop.org/wallbraker/apps/-/raw/main/VirtualGround-x86_64.AppImage
  chmod +x VirtualGround-x86_64.AppImage
  ./VirtualGround-x86_64.AppImage normal
  ```

- **Use a real device in Monado**.

  When using a real device driver you might want to enable the `XRT_DEBUG_GUI=1` and `SLAM_UI=1` environment variables to show debug GUIs of Monado and Basalt respectively.

  Monado has a couple of drivers supporting SLAM tracking (and thus Basalt). Most of them should work without any user input.

  - WMR ([troubleshoot](doc/monado/WMR.md))
  - Rift S (might need to press "Submit to SLAM", like the Vive Driver).
  - Northstar / DepthAI ([This hand-tracking guide](https://monado.freedesktop.org/handtracking) has a depthai section).
  - Vive Driver (Valve Index) ([read before using](doc/monado/Vive.md))
  - RealSense Driver ([setup](doc/monado/Realsense.md)).

## Development

If you want to setup your build environment for developing and iterating on Basat [development guide](monado/Development.md).
