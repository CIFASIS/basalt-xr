# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) improved
for tracking XR devices with
[Monado](https://gitlab.freedesktop.org/monado/monado). Many thanks to the
Basalt authors.

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

If you want to run OpenXR application with Monado, you need to set the
environment variable `VIT_SYSTEM_LIBRARY_PATH` to the path of the basalt library.

By default, Monado will try to load the library from `/usr/lib/libbasalt.so` if
the environment variable is not set.

If you want to test whether everything is working you can download a short dataset with [EuRoC (ASL) format](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format like [`MIO09_short_1_updown`] from the [Monado SLAM datasets](https://huggingface.co/datasets/collabora/monado-slam-datasets):

```bash
wget https://huggingface.co/datasets/collabora/monado-slam-datasets/resolve/main/M_monado_datasets/MI_valve_index/MIO_others/MIO09_short_1_updown.zip
unzip -d MIO09_short_1_updown MIO09_short_1_updown.zip
```

- **Try it standalone with a dataset (requires extra binaries)**

  ```bash
  basalt_vio --show-gui 1 --dataset-path MIO09_short_1_updown/ --dataset-type euroc --cam-calib /usr/share/basalt/msdmi_calib.json --config-path /usr/share/basalt/msdmi_config.json
  ```

- **Use a RealSense camera without Monado (requires extra binaries)**
  You'll need to calibrate your camera if you want the best results but meanwhile you can try with these calibration files instead.

  - RealSense D455 (and maybe also D435)

    ```bash
    basalt_rs_t265_vio --is-d455 --cam-calib /usr/share/basalt/d455_calib.json --config-path /usr/share/basalt/msdmi_config.json
    ```

  - Realsense T265: Get t265_calib.json from [this issue](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and run

    ```bash
    basalt_rs_t265_vio --cam-calib t265_calib.json --config-path /usr/share/basalt/msdmi_config.json
    ```

- **Try it through `monado-cli` with a dataset**

  ```bash
  monado-cli slambatch MIO09_short_1_updown/ /usr/share/basalt/msdmi.toml results
  ```

- **Try it with `monado`, a dataset, and an OpenXR app**

  ```bash
  # Run monado-service with a fake "euroc device" driver
  export EUROC_PATH=MIO09_short_1_updown/ # dataset path
  export EUROC_HMD=false # false for controller tracking
  export EUROC_PLAY_FROM_START=true # produce samples right away
  export SLAM_CONFIG=/usr/share/basalt/msdmi.toml # includes calibration
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

If you want to set up your build environment for developing and iterating on Basalt, see the [development guide](doc/monado/Development.md).

[`MIO09_short_1_updown`]: https://cdn-lfs.huggingface.co/repos/67/fd/67fd1ba4199080471752c06591b86b4ac3e99851d141303f981ebf0d29be7519/bc1ea056d711a407611f84f32951eaa562d4ff2f252012eedb673ba976262b59?response-content-disposition=inline%3B+filename*%3DUTF-8%27%27MIO09_short_1_updown.webm%3B+filename%3D%22MIO09_short_1_updown.webm%22%3B&response-content-type=video%2Fwebm&Expires=1706387046&Policy=eyJTdGF0ZW1lbnQiOlt7IkNvbmRpdGlvbiI6eyJEYXRlTGVzc1RoYW4iOnsiQVdTOkVwb2NoVGltZSI6MTcwNjM4NzA0Nn19LCJSZXNvdXJjZSI6Imh0dHBzOi8vY2RuLWxmcy5odWdnaW5nZmFjZS5jby9yZXBvcy82Ny9mZC82N2ZkMWJhNDE5OTA4MDQ3MTc1MmMwNjU5MWI4NmI0YWMzZTk5ODUxZDE0MTMwM2Y5ODFlYmYwZDI5YmU3NTE5L2JjMWVhMDU2ZDcxMWE0MDc2MTFmODRmMzI5NTFlYWE1NjJkNGZmMmYyNTIwMTJlZWRiNjczYmE5NzYyNjJiNTk%7EcmVzcG9uc2UtY29udGVudC1kaXNwb3NpdGlvbj0qJnJlc3BvbnNlLWNvbnRlbnQtdHlwZT0qIn1dfQ__&Signature=zHXBYpMUYN1ZFwMHd5oDhSd9xYzE99bJBd3GrKw9eePKczNYTJ%7Eua5jKqS4PB8%7EExnafPeMqyBE3NryugIiQ-q68mdA-04l%7Epn7fqoC0U4aAyp1tQlNzCmxJ4pdPDzL3kXsPgAcboxyZ90GoFsoJOT0RQo5UDAffKarB%7E8zEi7TWM-Zxk%7E5AFZ5X3uHVATEgYvdUB5CDdvq4bnDYuy9Al4KZugmT-ZYZAsDZwFfwKmf2Ws%7E7c3RLAtp9uykdRggDorEk0CspnYLl7pEb24GcNWhgT3xIdnZQ9u87inXrgwmYKgf6b-LhiEV8QSXWMuGgV3ICX67wkAEpgQ4o-s0TFg__&Key-Pair-Id=KVTP0A1DKRTAX
