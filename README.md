# Parallel-Structure-for-Meshing

This project is related to the paper: Lv C, Lin W, Zhao B. Voxel Structure-based Mesh Reconstruction from a 3D Point Cloud[J]. IEEE Transactions on Multimedia, 2021.

Created by Chenlei Lv from Nanyang Technological University.

Personal Website: https://aliexken.github.io/

Project Link: https://aliexken.github.io/subSet/MeshReconstruction.html

![image](https://user-images.githubusercontent.com/65271555/128494938-a9bc0097-5dc8-4b4d-88dd-36ceef508f72.png)

## Abstract

Mesh reconstruction from a 3D point cloud is an important topic in the fields of computer graphic, computer vision, and multimedia analysis. In this paper, we propose a voxel structure-based mesh reconstruction framework. It provides the intrinsic metric to improve the accuracy of local region detection. Based on the detected local regions, an initial reconstructed mesh can be obtained. With the mesh optimization in our framework, the initial reconstructed mesh is optimized into an isotropic one with the important geometric features such as external and internal edges. The experimental results indicate that our framework shows great advantages over peer ones in terms of mesh quality, geometric feature keeping, and processing speed.

## Libiary

PCL 1.8.1: https://pointclouds.org/downloads/

CGAL: https://www.cgal.org/

freeglut64: http://freeglut.sourceforge.net/index.php#download

## Updating

2021/04

The related paper, "Voxel Structure-based Mesh Reconstruction from a 3D Point Cloud", is accepted by IEEE Transactions on Multimedia.

2020/11

Project name has been changed to Voxel structure-based mesh reconstruction.
For now, it is a pre-version which includes the inital mesh reconstruction without mesh optimization. When related papers are published, then the complete version will be released.

2020/07

The code of Parallel Structure, which is used to reconstruct mesh from 3D Point Cloud.

The project is build on VS2019.

There have some open support library should be included in the project:

OpenGL:freeglut

CGAL library

PCL library

The project has relationship with AIVS simplification. Therefore the project is called AIVS Remesh.



