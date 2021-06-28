# IndoorPointCloudViewer
<div align=center><img src="https://github.com/peterliu502/IndoorPointCloudViewer/blob/master/readme_img/icon.png" width = "200" height = "200" alt = "project icon" /></div>   

Delft University of Technology  
MSc. Geomatics Synthesis (GEO1101) Project  
Project 5: 3D Representations for Visual Insight  
Download the project report and the introduction video from: http://resolver.tudelft.nl/uuid:a6c4f703-b048-40e3-9661-be00c0fab804   

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
If you only need the final software, you can directly look at section [How to run the IndoorPointCloudsViewer (release version)](https://github.com/peterliu502/IndoorPointCloudViewer#how-to-run-the-indoorpointcloudsviewer-release-version).  
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
Download `IndoorPointcloudsViewer_project.7z` from [latest release version](https://github.com/peterliu502/IndoorPointCloudViewer/releases) and unzip it.  

  3. Open project file in UE4  
The path of UE4 project file: `.\IndoorPointcloudsViewer_project\thirdperson.uproject`.

  4. Compile and the project  
The first run may take a long time.  


### How to run the IndoorPointCloudsViewer (release version)  
  1. Download and unzip the game file  
Download `IndoorPointCloudViewer.7z` from [latest release version](https://github.com/peterliu502/IndoorPointCloudViewer/releases) and unzip it.  
__Currently the `IndoorPointCloudViewer` only has `Windows` version.__

  2. Run game file  
Open `.\IndoorPointcloudsViewer\thirdperson.exe` and enjoy it!  

## Functionality
<div align=center><img src="https://github.com/peterliu502/IndoorPointCloudViewer/blob/master/readme_img/function.png" alt = "Functionality" /></div>   

```
(A) View the point cloud data in first-person perspective, and the shape of the points is set as circles.
(B) View the point cloud data in first-person perspective, and the shape of the points is set as squares.
(C) View the point cloud in third-person perspective.
(D) View the point cloud data in bird’s eye view, the positions of the avatar and the red target can be identified.
(E) Set the point size as the smallest size.
(F) Set the point size as the biggest size.
(G) View the point cloud in style 1.
(H) View the point cloud in style 2.
```  

## Citation  
If you want to cite this project in your work, you can use following `bibtex` entry:  
```
@article{fu_3d_2021,
	title    = {{3D} {Representations} for {Visual} {Insight}},
	url      = {https://repository.tudelft.nl/islandora/object/uuid%3Aa6c4f703-b048-40e3-9661-be00c0fab804},
	abstract = {As a method that can accurately represent 3D spatial information, point cloud visualisation for indoor environments is still a relatively unexplored field of research. Our client for this project, the Dutch National Police, requested a variety of potential solutions for visualising (unfamiliar) indoor environments that can be viewed by both external command centres, and internal operations units. Currently, unknown interior layouts (or layouts that are different in practise to what is stated on paper) can have serious, sometimes even life-threatening, consequences in time-sensitive situations. This project uses a game engine to directly visualise point cloud data input of indoor environments. The primary aim is to find ways of clearly communicating a point cloud of an environment to a layman viewer through intuitive visualisations, to aid decision-making in high-stress moments. The final product is a variety of visualisation concepts, hosted within a game engine in order to allow users to navigate throughout (part of) a building, and customise certain interaction features. To aid the layman viewer, various interpretation methods (e.g. cartography) are considered. The Unreal Engine 4 (UE4) project was designed and developed based on the requirements given by Dutch Police, and consisted of 4 modules: data preprocessing, render style, functional module, and User Interface (UI). An indoor point cloud dataset is used for the implementation, while corresponding mesh and voxel models are also respectively generated and evaluated as reference objects. The implemented software product is evaluated based on a Structured Expert Evaluation Method and finally our project result demonstrates that point cloud has unique advantages for visualisation of indoor environments especially in pre-processing efficiency, detail level, and volume perception.},
	language = {en},
	author   = {Fu, Runnan and Jin, Yuzhen and Liu, Zhenyu and Mainelli, Xenia Una and Papakostas, Theodoros and Wang, Linjun},
	year     = {2021}
}
```
