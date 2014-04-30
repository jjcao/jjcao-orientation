Code instructions for Mendable consistent normal of point cloud
1. run ONBE/obne.m to estimate normals
2. run CSV\Cao_smi11\matlab\main.m to generate visibility information under different viewpoints
3. run NormalOrientation\Normalorientation.exe to orient a point cloud with the outputs of the above two procedures.

-----------------------------------------
Orientation-benefit Normal Estimation
-----------------------------------------
1.After  executing obne.m we can obtain the result in the directory of OBNE\dateOBNE. For example, fandisk_obne_unorientation.off.
2.The format of result is as follows:
-----------------------------------------
|NOFF                                   |
|number of point  flag 0                       |
|x y z normal_x normal_y normal_y       |
-----------------------------------------

-----------------------------------------
visibility information under different viewpoints
-----------------------------------------
1.we use the result of OBNE, and the input data is complied wth the above format.
2.Running main.m located in the directory of Code\CSV\Cao_smi11\matlab on the operating environment of Matlab.
3.The result is saved in the directory of Code\CSV\Cao_smi11\result and named result_viewpoint.txt.
----------------------------------------
Multi-sources normal propagation
1.Setup the library of the PCL 1.4 and configure its environment variables.
2.The development environment is vs 2008 X86.
3.Compiling the source code located in the directory of NormalOrientation.
4.Running our method as follows:
	1)Normalorientation.exe fandisk_obne_uorientation.off
	Note: Orienting normal of point cloud interactively.
	2)NormalOrientation.exe fandisk_obne_unorientation.off detection fandisk_viewpoint.txt
	Note: Detecting multi-sources automatly and propagating the direction of sources'normals.
	3)NormalOrientation.exe fandisk_muliti_orientation.off fandisk_Ground.off consistent
	Note:the quantities analysis of the result of normal orientation of point cloud.