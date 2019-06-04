# jsk_choreonoid
Introduction
------------
This is a ros packge of [choreonoid](http://choreonoid.org/) plugin for controlling humanoid robots.  
jsk_choreonid contains several controller: Multi Contact Stabilizer, Resolved Momentum Control, and Preview Control etc.  

Requirements
------------
1. ros indigo or later
1. [choreonid](https://github.com/kindsenior/choreonoid)

Install
-------
Go to ros workspace  
Clone https://github.com/kindsenior/choreonoid and this package  
catkin build jsk_choreonid --force-cmake  

Usage
-----
**First, see [choreonoid tutorials](http://choreonoid.org/en/manuals/ or http://choreonoid.org/ja/manuals/)**

- Sample of the multi contact stabilizer
1. run choreonoid with project file

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. select poseseq
1. push MCS button
1. push RM button
1. select motion item
1. push play button

- Sample of the preview control
1. run choreonoid with project file

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. select poseseq
1. push PreviewControl button
1. select motion item
1. push play button
