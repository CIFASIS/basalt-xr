# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) with some
modifications so that it can be used from Monado for SLAM tracking. Many thanks
to the Basalt authors.

Follow this file for instructions on how to get Basalt up and running with
Monado. This README tries to be as concise as possible, but there are many
details that need to be addressed on it, so please do not skip any section,
otherwise it is likely that it won't work. Having said that, this guide has
been tested in limited setups, so please report any changes you had to make
in order to get it working in different ones.

## Index

- [Basalt for Monado](#basalt-for-monado)
  - [Index](#index)
  - [Installation](#installation)
    - [Build and Install Directories](#build-and-install-directories)
    - [Dependencies](#dependencies)
    - [Build Basalt](#build-basalt)
    - [Running Basalt](#running-basalt)
    - [Monado Specifics](#monado-specifics)
  - [Notes on Basalt Usage](#notes-on-basalt-usage)
  - [Using Real Hardware](#using-real-hardware)

## Installation

This was tested on both Ubuntu 20.04 and 18.04, be sure to open an issue if the
steps don't work for you. The main branch of this fork is
[`xrtslam`](https://gitlab.freedesktop.org/mateosss/basalt/-/tree/xrtslam).

### Build and Install Directories

To not clutter your system directories, let's set two environment variables,
`$bsltdeps` and `$bsltinstall` that point to existing empty build and install
directories respectively. These directories will contain everything produced in
this guide besides installed apt dependencies.

```bash
# Change the paths accordingly
export bsltinstall=/home/mateo/Documents/apps/bsltinstall
export bsltdeps=/home/mateo/Documents/apps/bsltdeps
```

Let's extend our system paths with those.

```bash
export PATH=$bsltinstall/bin:$PATH
export PKG_CONFIG_PATH=$bsltinstall/lib/pkgconfig:$PKG_CONFIG_PATH # for compile time pkg-config
export LD_LIBRARY_PATH=$bsltinstall/lib/:$LD_LIBRARY_PATH # for runtime ld
export LIBRARY_PATH=$bsltinstall/lib/:$LIBRARY_PATH # for compile time gcc
```

### Dependencies

Most dependencies will be automatically built by basalt, however there are some
known issues you might need to deal with (click to open the ones that might
affect you).

<details>
  <summary>Issues with GCC 11</summary>

  If you are using GCC 11 you might also get some issues with pangolin as there is now a
  [name clash with Pagolin `_serialize()` name](https://github.com/stevenlovegrove/Pangolin/issues/657),
  it [should be fixed](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=100438#c12)
  in newer versions of GCC-11. For fixing it yourself, you can cherry-pick
  [these commits](https://github.com/stevenlovegrove/Pangolin/pull/658/commits),
  or use a different GCC version.
  (see
  [this discord thread](https://discord.com/channels/556527313823596604/556527314670714901/904339906288050196)
  in the Monado server for more info).
</details>

### Build Basalt

```bash
cd $bsltdeps
git clone --recursive git@gitlab.freedesktop.org:mateosss/basalt.git
./basalt/scripts/install_deps.sh
sed -i "s#/home/mateo/Documents/apps/bsltdeps/#$bsltdeps/#" basalt/data/monado/*.toml
cd basalt && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTS=off -DBASALT_INSTANTIATIONS_DOUBLE=off
make install -j12
```

### Running Basalt

This step is optional but you can try Basalt without Monado with one of the following methods:

- Through an EuRoC dataset (be sure to [download
  one](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/)
  first): `basalt_vio --dataset-path /path/to/euroc/V1_01_easy --cam-calib
  $bsltdeps/basalt/data/euroc_ds_calib.json --dataset-type euroc --config-path
  $bsltdeps/basalt/data/euroc_config.json --marg-data ~/Desktop/euroc_marg_data
  --show-gui 1`
- With a RealSense T265 (you'll need to get a `t265_calib.json` yourself as
  detailed [below](#configuring-basalt) but meanwhile you can try with [this
  file](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) instead):
  `basalt_rs_t265_vio --cam-calib $bsltdeps/basalt/data/t265_calib.json
  --config-path $bsltdeps/basalt/data/euroc_config.json`
- With a RealSense D455 (and maybe this also works for a D435):
  `basalt_rs_t265_vio --is-d455 --cam-calib
  $bsltdeps/basalt/data/d455_calib.json --config-path
  $bsltdeps/basalt/data/euroc_config.json`

### Monado Specifics

Note: be careful when manually enabling ASan when building Monado as some
crashes have been reported. I'm still trying to figure out why those happen.

Run an OpenXR app like `hello_xr` with the following environment variables set

```bash
export EUROC_PATH=/path/to/euroc/V1_01_easy/ # Set euroc dataset path. You can get a dataset from http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip
export EUROC_LOG=debug
export EUROC_HMD=false # if false, a fake controller will be tracked, else a fake HMD
export EUROC_PLAY_FROM_START=true
export SLAM_LOG=debug
export SLAM_SUBMIT_FROM_START=true
export SLAM_CONFIG=$bsltdeps/basalt/data/monado/euroc.toml # Point to Basalt config file for Euroc
export OXR_DEBUG_GUI=1 # We will need the debug ui to start streaming the dataset
```

Finally, run the XR app and press start in the euroc player debug ui and you
should see a controller being tracked with Basalt from the euroc dataset.

## Notes on Basalt Usage

- Tracking is not perfect, [this](https://youtu.be/mIgRHmxbaC8) and
  [this](https://youtu.be/gxu3Ve8VCnI) show how it looks, as well as the
  problems that it has (difficulties with rotation-only movements, wiggliness on
  fast movements, etc)
- This fork only works with Stereo-IMU setups, but adapting Basalt to work with
  other configurations should feasible (see
  [granite](https://github.com/DLR-RM/granite)).
- Basalt is _fast_. While the standard sampling rate is stereo 640x480 at 30fps
  I've been able to make it work at 848x480 at 60fps without problems on a
  laptop.
- Some things that might cause crashes:
  - Using images with bad exposure and gain values, or being in a dark room.
  - Shaking causes drift that can diverge if maintained for long periods of
    time.
  - Continuously making sudden 90 degree rotations in which the new scene does not share
    features with the previous scene.
  - Moving too fast and/or making rotation only movements over extended periods
    of time.

## Using Real Hardware

Monado has a couple of drivers supporting SLAM tracking (and thus Basalt). Here is how to set them up:

- [RealSense Driver](doc/monado/Realsense.md)
- [WMR Driver](doc/monado/WMR.md)
- [Vive Driver (Valve Index)](doc/monado/Vive.md)
