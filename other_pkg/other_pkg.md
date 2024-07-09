# Pkgs that may take effort to install

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

