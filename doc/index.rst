.. rapprentice documentation master file, created by
   sphinx-quickstart on Mon Apr 15 13:35:19 2013.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

********************************************************
Rapprentice: teaching robots by example
********************************************************

.. Contents:
.. 
.. .. toctree::
..    :maxdepth: 2
.. 
   
Dependencies and build procedure
==================================

Download the source from https://github.com/joschu/rapprentice

Dependencies:

- pcl >= 1.6
- `trajopt <https://github.com/joschu/trajopt>`_. devel branch. Build with the option BUILD_CLOUDPROC enabled.
- python modules: scipy, h5py, networkx, cv2 (OpenCV)

Build procedure:

- Build the fastrapp subdirectory using the usual cmake procedure. This includes a boost python module with a few functions, and a program for recording RGB+Depth video (from Primesense cameras). Let's assume now that your build directory is ``fastrapp_build_dir``.
- Add the rapprentice root directory and fastrapp_build_dir/lib to your ``PYTHONPATH``.
- Add ``fastrapp_build_dir/bin`` to your $PATH.
   
Training
================================================

Overview
-----------

The training pipeline is illustrated below.

.. figure:: schematic.png
   :width: 75%
   :alt: schematic for recording and processing demonstrations

File formats:

- RGBD video: a directory with a series of png files (depth images, measured in millimeters) and jpg files (rgb images) and another file with the ROS timestamps.
- annotations file: yaml
- master task file: yaml
- processed demonstration file: hdf5

See the ``sampledata`` directory for examples of these formats.

Teaching procedure
---------------------

Use the ps3 controller to indicate "look", "start", and "stop" times.

.. figure:: ps3-annotated.jpg
  :width: 40%
  :alt: ps3


Processing training data
--------------------------

You'll presumably collect multiple runs of the whole task. Then you run a script to generate an hdf5 file that aggregates all these demonstrations, which are broken into segments.

To see an example of how to run the data processing scripts, see the script ``example_pipeline/overhand.py``, which processes an example dataset, which contains demonstrations of tying an overhand knot in rope. To run the script, you'll need to download the sample data with ``scripts/download_sampledata.py``.


Execution
=============

::

  ./do_task.py h5file
  
You can run this program in various simulation configurations that let you test your algorithm without using the robot.


Tips for debugging execution
-------------------------------

- First make sure plots are enabled for the registration algorithm so you can see the demonstration point cloud (or landmarks) being warped to match the current point cloud. Check that the transformation looks good and the transformation is sending the points to the right place.
- Next, enable plotting for the trajectory optimization algorithm. Look at the purple lines, which indicate the position error. Make sure the target positions and orientations (indicated by axes) are correct.
- Look at the output of the trajectory optimization algorithm, which might tell you if something funny is going on.


Extras
========

Various other scripts are included in the ``scripts`` directory:

- ``view_kinect.py``: view the live rgb+depth images from your Primesense camera.
- ``command_pr2.py``: for conveniently ordering the pr2 around, run ``ipython -i command_pr2.py``. Then you can control the pr2 with ipython by typing commands like ``pr2.rarm.goto_posure(`side`)`` or ``pr2.head.set_pan_tilt(0,1)``.
- ``animate_demo.py`` animates a demonstration.


Miscellaneous notes
=====================

``PR2.py`` is set up so you can send commands to multiple bodyparts simultaneously. So most of the commands, like ``goto_joint_positions`` are non-blocking. If you want to wait until all commands are done, do ``pr2.join_all()``.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

