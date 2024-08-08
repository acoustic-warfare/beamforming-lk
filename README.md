# Beamformer 

This is an adaptation of the beamformer-pipeline for digital signal processing of audio sampling.
Read more and get started on the [Wiki](https://github.com/acoustic-warfare/beamforming-lk/wiki).

A shotcut to installation and getting started can be found [here](https://github.com/acoustic-warfare/beamforming-lk/wiki/Getting-started).

The library has the following features:
* Receiving audio sampling from up to 4 FPGAS, each taking at most 4 microphone arrays.
* Beamforming on data from 2 FPGAS, each taking at most 4 microphone arrays.
* Offline usage for efficient testing (udpreplay).
* Dynamic delay calculation and target directions to enhance signals from specific directions.
* Realtime processing of audio signals.
* Tools for visualizing beamforming results (heatmaps) and configurations.
* Customization of beamforming parameters. 

Current issues that may need to be taken into considiration is found in the [issues](https://github.com/acoustic-warfare/beamforming-lk/issues) tab.

## Documentation
* Documentation for setting up, developers, tips and tricks as well as future workings can all be found on the [Wiki](https://github.com/acoustic-warfare/beamforming-lk/wiki).
* You may see a structural documentation of the project by building the `Doxygen`
page which will generate a documentation page at `doc/html/index.html`. Se how [here](https://github.com/acoustic-warfare/beamforming-lk/wiki/Getting-started).
```
make doc
```
* Also see .h files directly in the repo.

