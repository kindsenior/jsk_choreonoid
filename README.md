# jsk_choreonoid
Introduction
------------
This is a ros packge of [choreonoid](http://choreonoid.org/) plugin for controlling humanoid robots.  
jsk_choreonid contains several controller: Multi Contact Stabilizer, Resolved Momentum Control, and Preview Control etc.  

Requirements
------------
1. ros hydro or later
1. [choreonid-ros-pkg](https://github.com/kindsenior/choreonoid-ros-pkg)

Install
-------
Go to ros workspace  
Clone https://github.com/kindsenior/choreonoid-ros-pkg and this package  
catkin build jsk_choreonid --force-cmake  

Usage
-----
**First, see [choreonoid tutorials](http://choreonoid.org/en/manuals/1.5/index.html or http://choreonoid.org/ja/manuals/1.5/index.html)**

1. run choreonoid

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. push MCS button
1. push play button
