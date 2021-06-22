# geo1101
<img src="https://github.com/peterliu502/geo1101/blob/15bc6d1add02f13e49f17f500e70f854ec99e73b/icon.png" width = "200" height = "200" alt="project icon" />   

GEO1101 Project 5: 3D Representations for Visual Insight  

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
 ├─ 3Dengine: UE4 project files
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
## Instructions
### How to run the pointclouds preprocess program  
  1. Install PCL library  
For `Windows` users, it is highly recommended to use [`Vcpkg`](https://github.com/microsoft/vcpkg/releases) to install [`PCL`](https://pointclouds.org/downloads/).  
For `Mac` and `Linux` users, you can use the official recommended way to install it first. If there are problems in the following steps, you can also use `Vcpkg`.  
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


### How to run the IndoorPointCloudsViewer project file in UE4
  1. Install Unreal Engine 4  
Install [Unreal Engine 4 (UE4)](www.unrealengine.com) (`version 4.26` or later).  

  2. Download and unzip the project file  
Download `IndoorPointcloudsViewer_project` from [latest release version](https://github.com/peterliu502/IndoorPointCloudViewer/releases) and unzip it.  

  3. Open project file in UE4  
The path of UE4 project file: `.\IndoorPointcloudsViewer_project\thirdperson.uproject`.

  4. Compile and the project  
The first run may take a long time.  


### How to run the IndoorPointCloudsViewer (release version)  
  1. Download and unzip the game file  
Download `IndoorPointCloudViewer` from [latest release version](https://github.com/peterliu502/IndoorPointCloudViewer/releases) and unzip it.  

  2. Run game file  
Open `.\IndoorPointcloudsViewer\thirdperson.exe` and enjoy it!  
