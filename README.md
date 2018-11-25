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

1. run choreonoid

  ```
  $ roscd jsk_choreonoid/sample/WalkSample/
  $ choreonoid JAXON_RED_Walk.cnoid
  ```
1. push MCS button
1. push play button
