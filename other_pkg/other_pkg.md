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
