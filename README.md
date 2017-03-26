# srrg_core

This package contains some essential libraries and utilities extensively used in
the srrg repos. These include
* base defines (2D/3D transforms, points, elementary linear algebra operations)
* base system utilities (monitoring the CPU frequency of a module, printing a banner)
* reading/writing text formatted log files (txt_io)
* computing 2D distance maps for nearest neighbor search
* computing KD trees for nn search in multidimensional spaces
* drawing lines on a 2D map
Besides the libraries, the package provides also a set of utilities
to convert and merge text based log files

## Prerequisites

requires srrg_boss and srrg_cmake_modules

## Applications
The following are some applications provided in the module

* srrg_txt_io_converter_euroc_app: converts from euroc scripts to txt_io format
* srrg_txt_io_merger_app: merges two or more txt_io logs preserving the temporal  order
* srrg_txt_io_converter_kitti_app: converts from kitti_format to txt_io
* srrg_txt_io_sorter_app: sorts a txt_io log based on the timestamp of the records

To get the help launch one of the previous programs with the -h option
Example:
```
rosrun srrg_core srrg_txt_io_merger_app -h
```

## Examples 
SRRG core builds a bunch of examples to help using the libraries.
These include:
* srrg_txt_io_synchronizer_example: example on how to generate an assoc file (for instance for orb slam), containing depth and rgb images
* srrg_open_file_example: how to open and parse a srrg file
* srrg_kdtree_example:    how to use a KD tree

## Authors
* Giorgio Grisetti
* Jacopo Serafin
* Mayte Lazaro
* Maurilio Di Cicco
* Taigo Bonanni
* Bartolomeo Della Corte
* Dominik Schlegel

## License

BSD 2.0
