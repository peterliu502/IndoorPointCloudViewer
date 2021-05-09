# geo1101
GEO1101 Project 5: 3D Representations for Visual Insight
## Team menbers  
[@Runnan Fu](https://github.com/runnanfu)
[@Yuzhen Jin](https://github.com/yuzhenjin3000)
[@Zhenyu Liu](https://github.com/peterliu502)
[@Xenia U. Mainelli]()
[@Theodoros Papakostas](https://github.com/tpapakostas)
[@Linjun Wang](https://github.com/fiodccobw)

## Files structure
```
geo1101
 ├─ data: store the input and output files
 │   ├─ VRR.pcd: input classified file
 │   ├─ *.ply the segmentation result of VRR.pcd
 │   └─ *.txt the text version converted from pcd
 ├─ main.cpp: the main workflow of the whole project
 ├─ preprocess.cpp: the workflow of the preprocess
 ├─ preprocess_methods.cpp: functions and data structures used to form the sub_steps in preprocess.cpp
 ├─ basic_methods.cpp: functions used to support the whole project
 ├─ run.py: the trigger of the whole program
 └─ py_methods.py: the functions and data structures written in Python

```
## How to run this program  
1. Install PCL library  
For `Windows` users, it is highly recommended to use [`Vcpkg`](https://github.com/microsoft/vcpkg/releases) to install [`PCL`](https://pointclouds.org/downloads/).  
For `Mac` and `Linux` users, you can use the official recommended way to install it first. If there are problems in the following steps, yu can also use `Vcpkg`.  
For `Clion` users, `Vcpkg` is best used in conjunction with the `CMake` files, as shown in the tutorial [here](https://github.com/microsoft/vcpkg#vcpkg-with-clion).  
2. Compile the CPP program  
Using Release mode to build the `main.cpp`, then the build folder will be created.  
3. Run run.py  
Run the `run.py` and the output files are in the `./data` folder
