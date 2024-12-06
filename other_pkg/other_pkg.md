# Pkgs that may take effort to install/work

### PyTorch3D<a name="pytorch3d"></a>

- **[Official installation](https://github.com/facebookresearch/pytorch3d/blob/main/INSTALL.md)**

  - TL;DR

    _**Tips:** `pytorch3d` is supposed to be built upon <u>python 3.8 or 3.9</u>. You may otherwise bump into `cpython` and sharing lib errors_

    ```bash
    # creat and activate env
    # conda create -n pytorch3d python=3.9
    # conda activate pytorch3d
    
    # install dependencies
    conda install pytorch=1.13.0 torchvision pytorch-cuda=11.6 -c pytorch -c nvidia
    conda install -c fvcore -c iopath -c conda-forge fvcore iopath
    
    # install pytorch3d
    conda install pytorch3d -c pytorch3d
    ```

    _Refer to "Build from source" if any problem occurs_



- Build from source: [CSDN blog](https://blog.csdn.net/PascalBUAA/article/details/123408759#t7)

  _**Tips:** you may only include alternative [implementation](./pytorch3d_transforms.py) for `pytorch3d.transforms` to save the effort when not using diffusion-based method_



### PyTorch Robot Kinematics<a name="pyk"></a>

Github: **[pytorch_kinematics](https://github.com/UM-ARM-Lab/pytorch_kinematics)**, `pip install pytorch-kinematics`

_Easy to install, difficult to use..._

#### Err1: device error

**Reason**: device is not specified for kinematics chain build from URDF

**Solution**: add device to functions in `urdf.py` (original [file](https://github.com/UM-ARM-Lab/pytorch_kinematics/blob/7d4e197e6bbc13f8ffd77773adc23631052bb072/src/pytorch_kinematics/urdf.py), modified example [here](./urdf.py))



#### Err2: shape error

**Reason**: batch info needs to be specified with an extra argument in [`Transform3d`](https://github.com/UM-ARM-Lab/pytorch_kinematics/blob/7d4e197e6bbc13f8ffd77773adc23631052bb072/src/pytorch_kinematics/transforms/transform3d.py#L163)


### Kinect setup on 22.04

Follow [Azure Kinect with ROS2 Humble - Robotics Stack Exchange](https://robotics.stackexchange.com/questions/24529/azure-kinect-with-ros2-humble).

1. Install the Azure Kinect Sensor SDK ([reference](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1790)):
   
   * Temporarily switch the source list of apt to 20.04:
     
     * replace the file `/etc/sources.list` with [20.04 source list](https://gist.github.com/ishad0w/788555191c7037e249a439542c53e170)
     
     * run `sudo apt update` and then install `libsoundio1`
     
     * switch back your source list to 22.04
   
   * For Ubuntu 22.04, you can download [k4a-tools_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb), [libk4a1.4-dev_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb) and [libk4a1.4_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb) for 18.04.

2. (done) Clone [Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/humble) to the `src` folder and checkout to the humble branch.

3. run rosdep `rosdep install --from-paths src --ignore-src -r -y`

4. build `colcon build` and `source install/setup.bash`

5. Copy the file [99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules) into `/etc/udev/rules.d/`, run `sudo udevadm control --reload` to reload settings, then detach and reattach the kinect

6. Run the launch file `ros2 launch azure_kinect_ros_driver driver.launch.py`
