# Windows Mixed Reality Headsets

We'll need to make a Basalt config file for your headset, let's say it's a
Reverb G2.

First, let's get your WMR device json config block. To get that json, set the
environment variable `WMR_LOG=debug` and run Monado with your WMR headset connected.
The headset json is printed on start after the line `DEBUG [wmr_read_config] JSON config:`.
Copy that to a file called `reverbg2_wmrcalib.json`.

Now let's convert this WMR json to a Basalt calibration file with:

```bash
$bsltdeps/basalt/data/monado/wmr-tools/wmr2bslt_calib.py reverbg2_wmrcalib.json > $bsltdeps/basalt/data/reverbg2_calib.json
```

Finally, we'll need to create the main config file for Basalt that references
this calibration file we just created. For that let's copy the config that is
already present for the Odyssey+:

```bash
cp $bsltdeps/basalt/data/monado/odysseyplus_rt8.toml $bsltdeps/basalt/data/monado/reverbg2.toml
```

And edit the `cam-calib` field in the `reverbg2.toml` file to point to your `reverbg2_calib.json` file.

And that's it, now you just need to reference this `reverbg2.toml` in the
`SLAM_CONFIG` environment variable before launching Monado with `export
SLAM_CONFIG=$bsltdeps/basalt/data/monado/reverbg2.toml` and Basalt will use the
appropriate calibration for your headset.

By default, the UI box `SLAM Tracker` has the option `Submit data to SLAM`
disabled so that you first manually configure the exposure and gain values in
the `WMR Camera` box. You can enable it yourself in the UI or enable it at start
by setting the environment variable `SLAM_SUBMIT_FROM_START=true`.

# Video Walkthrough

Here is a 15 minute walkthrough with some tips for using a WMR headset with Monado and Basalt that should help complement the guide found in the [README.md](README.md) file: <https://www.youtube.com/watch?v=jyQKjyRVMS4>
