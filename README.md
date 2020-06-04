# jsk_choreonoid
Introduction
------------
This is a ros packge of [choreonoid](http://choreonoid.org/) plugin for controlling humanoid robots.  
jsk_choreonoid contains several controller: Multi Contact Stabilizer, Resolved Momentum Control, and Preview Control etc.  

Requirements
------------
1. ros indigo or later
1. [choreonoid](https://github.com/kindsenior/choreonoid)

Install
-------
Go to ros workspace  
Clone https://github.com/kindsenior/choreonoid and this package  
catkin build jsk_choreonoid --force-cmake  

Usage
-----
**First, see [choreonoid tutorials](http://choreonoid.org/en/manuals/ or http://choreonoid.org/ja/manuals/)**

- Sample of the multi contact stabilizer
1. run choreonoid with project file

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. select a poseseq item in Item Tree View
1. push "MCS" button
1. push "RMC" button
1. select a motion item in Item Tree View
1. push play button

- Sample of the slide friction control (support jump motions)
1. run choreonoid with project file

  ```
  $ roscd jsk_choreonoid/sample/JumpSample/
  $ choreonoid JAXON_RED_Jump.cnoid
  ```
1. select a poseseq item in Item Tree View
1. push "CC" button
1. push "RMC" button
1. select a motion item
1. push play button
1. export optional hrpsys sequence file (push "export" button)
1. export hrpsys sequence files (select the motion item and File->Export Selected Items)
1. send the exported hrpsys sequence files to hrpsys-base by load-pattern service

- Sample of the preview control
1. run choreonoid with project file

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. select a poseseq in Item Tree View
1. push PreviewControl button
1. select a motion item
1. push play button
