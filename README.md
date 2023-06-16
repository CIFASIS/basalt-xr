# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) with some
modifications so that it can be used from Monado for SLAM tracking. Many thanks
to the Basalt authors.

Follow this file for instructions on how to get Basalt up and running with
Monado. **Read the instructions carefully**.

## Installation for Ubuntu 22.04

Download the .deb file from the [latest release](https://gitlab.freedesktop.org/mateosss/basalt/-/releases?latest) and then run

```bash
sudo apt install -y ./basalt-monado-ubuntu-22.04-haswell-amd64.deb
```

## Installation from source

This was tested on Ubuntu LTS derivatives from 18.04 and on, be sure to open an
issue if the steps don't work for you. The main branch of this fork is
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
git clone --recursive https://gitlab.freedesktop.org/mateosss/basalt.git
./basalt/scripts/install_deps.sh
sed -i "s#/home/mateo/Documents/apps/bsltdeps/#$bsltdeps/#" basalt/data/monado/*.toml
cd basalt && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTS=off -DBASALT_INSTANTIATIONS_DOUBLE=off
make install -j12
```

## After installing Basalt

You can now rebuild Monado so that it finds Basalt and then run any OpenXR app.

You might want to set the environment variables `XRT_DEBUG_GUI=1` and
`SLAM_UI=1` in Monado to see the debug GUIs of Monado and Basalt respectively.

For most drivers, you don't need to do much. See the next section for optional
info for your driver.

## Drivers

Monado has a couple of drivers supporting SLAM tracking (and thus Basalt). Most
of them should work without any user input.

- WMR ([troubleshoot](doc/monado/WMR.md))
- Rift S (might need to press "Submit to SLAM", like the Vive Driver).
- Northstar / DepthAI.
- Vive Driver (Valve Index) ([read before using](doc/monado/Vive.md))
- RealSense Driver ([setup](doc/monado/Realsense.md))

## Basalt Development

### Trying Basalt without Monado

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

### Running an EuRoC dataset through Monado

Run an OpenXR app like `hello_xr` with the following environment variables set

```bash
export EUROC_PATH=/path/to/euroc/V1_01_easy/ # Set euroc dataset path. You can get a dataset from http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip
export EUROC_LOG=debug
export EUROC_HMD=false # if false, a fake controller will be tracked, else a fake HMD
export EUROC_PLAY_FROM_START=true
export SLAM_LOG=debug
export SLAM_SUBMIT_FROM_START=true
export SLAM_CONFIG=$bsltdeps/basalt/data/monado/euroc.toml # Point to Basalt config file for Euroc
export XRT_DEBUG_GUI=1 # We will need the debug ui to start streaming the dataset
```

Finally, run the XR app and press start in the euroc player debug ui and you
should see a controller being tracked with Basalt from the euroc dataset.

There's also the monado-gui slambatch utility for running euroc datasets in
batch.
