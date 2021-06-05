# geo1101
[!project icon](https://github.com/peterliu502/geo1101/blob/15bc6d1add02f13e49f17f500e70f854ec99e73b/icon.png) GEO1101 Project 5: 3D Representations for Visual Insight
## Team menbers  
[@Runnan Fu](https://github.com/runnanfu)
[@Yuzhen Jin](https://github.com/yuzhenjin3000)
[@Zhenyu Liu](https://github.com/peterliu502)
[@Xenia U. Mainelli](https://github.com/mainelli)
[@Theodoros Papakostas](https://github.com/tpapakostas)
[@Linjun Wang](https://github.com/fiodccobw)

## Files structure
```
geo1101
 ├─ data: store the input and output files
 │   ├─ pointclouds: store the point clouds data
 │   ├─ meshes: store the meshes data
 │   └─ voxels: store the voxels data
 ├─ voxeliser: the module of voxelization, based on the GEO1004 HW01
 ├─ main.cpp: the main workflow of the whole project
 ├─ preprocess.cpp: the workflow of the preprocess
 ├─ preprocess_methods.cpp: functions and data structures used to form the sub_steps in preprocess.cpp
 ├─ basic_methods.cpp: functions used to support the whole project
 ├─ run.py: the trigger of the whole program
 └─ py_methods.py: the functions and data structures written in Python

```
## How to run the pointclouds preprocess program  
1. Install PCL library  
For `Windows` users, it is highly recommended to use [`Vcpkg`](https://github.com/microsoft/vcpkg/releases) to install [`PCL`](https://pointclouds.org/downloads/).  
For `Mac` and `Linux` users, you can use the official recommended way to install it first. If there are problems in the following steps, yu can also use `Vcpkg`.  
For `Clion` users, `Vcpkg` is best used in conjunction with the `CMake` files, as shown in the tutorial [here](https://github.com/microsoft/vcpkg#vcpkg-with-clion).  

2. Install CGAL library  
Make sure that you have installed `Vcpkg`.   
The tutorial for `Windows` click [here](https://doc.cgal.org/latest/Manual/windows.html).  
The tutorial for `Mac` or `Linux` click [here](https://doc.cgal.org/latest/Manual/usage.html).  

3. Compile the CPP program  
Using Release mode to build the `main.cpp`, then the build folder will be created.  

4. Run run.py  
Run the `run.py` and the output files are in the `./data` folder  
The meaning of the 4th attribute values in txt files:  
```
0: roof
1: ground
2: architecture part
3: non-architecture part
```

## How to run the pointclouds preprocess program (release version)  

## How to run the IndoorPCViewer  

## How to run the IndoorPCViewer (release version)  
