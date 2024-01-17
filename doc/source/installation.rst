============
Installation
============

To use the NEEM Evaluator you need to install the following dependencies:

 * ROS Noetic
 * Knowrob
 * The Python dependencies in the requirements.txt file

--------------
Installing ROS
--------------
You can find the installation instructions `here <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.

After installing ROS Noetic you need to setup a catkin workspace. This can be done by creating a folder for the
workspace and running the following commands:

.. code-block:: shell

    mkdir -p ~/catkin_ws/src

Afterwards clone the NEEM Evaluator repository into the src folder of the workspace:

.. code-block:: shell

    cd ~/catkin_ws/src
    git clone git@github.com:Tigul/NEEM_Evaluator.git
    cd ~/catkin_ws/
    catkin_make

You need an ssh key in Github to clone the repository. If you do not have an ssh key in GitHub use the following url to
clone the repository: https://github.com/Tigul/NEEM_Evaluator.git


------------------------------
Installing Python Dependencies
------------------------------

You can install the Python dependencies via Pip. To do so, run the following commands in the root folder of the
repository:

.. code-block:: shell

    apt-get install python3-pip
    pip install -r requirements.txt

-------
Knowrob
-------

In order to use the NEEM Evaluator you need to install Knowrob to handle NEEMs. You can find the installation
instructions `here <https://github.com/knowrob/knowrob>`_.

-----------------------
Additional Dependencies
-----------------------

If you want to use the URDF files in the 'resources' folder you need the following repositories:

 * https://github.com/code-iai/iai_maps
    * apartment.urdf
    * kitchen.urdf
* https://github.com/PR2/pr2_common
    * pr2.urdf

These need to be cloned to your catkin workspace and compiled with catkin_make. The catkin workspace should be
`~/catkin_ws/src` if you followed the installation instructions above.
