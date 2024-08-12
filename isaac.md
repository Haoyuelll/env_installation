# Isaac

### [IsaacSim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html)

```bash
# Setup python
python3 -m venv env_isaacsim
source env_isaacsim/bin/activate

# Update pip
env_isaacsim/bin/python3 -m pip install --upgrade pip

# Isaacsim installation from nvidia
env_isaacsim/bin/python3 -m pip install isaacsim --extra-index-url https://pypi.nvidia.com
```



#### Error

1. **https://pypi.nvidia.com does not have isaacsim**

```bash
ERROR: Could not find a version that satisfies the requirement isaacsim (from versions: none)
ERROR: No matching distribution found for isaacsim
```

**Solution**: install from Omniverse

*<u>Step 1</u>*: Install Omniverse GUI from [appimage](https://developer.nvidia.com/isaac/sim)

```bash
# After downloading
sudo chmod +x omniverse-launcher-linux.AppImage
# Run Omniverse
./omniverse-launcher-linux.AppImage
```

*<u>Step 2</u>*: Search "isaac sim" in apps and install

*Warning: this could take a long time as the package is roughly 9GB*



2. **"Your current region has an age policy that does not allow you to use the system" while logging into nvidia account**

**Solution**: VPN

- [Linux-clash guide](https://github.com/Haoyuelll/env_installation/blob/main/magic/magic.md) 



3. **Python module installed but not found**

```bash
Module xxx/libomni.activity.core.plugin.so remained loaded after unload request.
```

**Reason**: python path is not correctly specified 

**Solution**: documentation for [python installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html) / [anaconda specific](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html?highlight=conda#advanced-running-with-anaconda) 

**Tips**: 

- ***System python***

  Specify python path before running python script:

  ```bash
  $HOME/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh [script-to-run].py
  ```

  

- ***Conda python***

  Create environment following official `environment.yaml`

  ```bash
  cd ~/.local/share/ov/pkg/isaac-sim-2023.1.1
  conda env create -f environment.yml
  
  # Alternatively you may use the command below to update your current venv:
  conda activate [your-env]
  conda env update --file ~/.local/share/ov/pkg/isaac-sim-2023.1.1/environment.yml
  ```

  After virtual env installation:

  ```bash
  conda activate isaac-sim  # replace this with your env name
  source $HOME/.local/share/ov/pkg/isaac-sim-2023.1.1/setup_conda_env.sh
  ```

  Then run your python script as normal (without further need to specify python path)



4. **`xxx/setup_python_env.sh not found` when `source xxx/setup_conda_env.sh`**

**Reason**: not using bash as default shell, while the script takes working dirctory from bash. 

**Solution**: replace bash related variable to with universal one

In `setup_conda_env.sh`:

- Original:

  ```bash
  #!/bin/bash
  SCRIPT_DIR="$( cd "$( dirname "$BASH_SOURCE" )" && pwd )"
  ```

- Replace it with:

  ```bash
  #!/bin/sh
  SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"
  ```

  

In `setup_python_env.sh` 

- Original:

  ```bash
  #!/bin/bash
  SCRIPT_DIR="$( dirname "$BASH_SOURCE" )"
  ```

- Replace it with:

  ```bash
  #!/bin/sh
  SCRIPT_DIR="$( dirname "$0")"
  ```



#### Terminal launch

Start Isaac Sim with `$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1/isaac-sim.sh`



5. `segmentation fault (core dumped)`

**Reason**: check if there is duplicated launch of simulation app in your pipeline.



### [IsaacGym](https://developer.nvidia.com/isaac-gym)

Apply for Isaac Gym [here](https://developer.nvidia.com/isaac-gym)

```bash
# Download and extract with
tar -zxvf IsaacGym_Preview_4_Package.tar.gz

cd isaacgym

# Open docs/index.html for guidance
google-chrome docs/index.html
```

Install with:

```bash
# Install Isaac Gym
cd python && pip install -e .
# Check installation
pip show isaacgym
```



#### RL usage: **[IsaacGymEnvs](https://github.com/isaac-sim/IsaacGymEnvs)**



### [OmniIsaacGymEnvs](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)

Original README with installation guide [here](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/README.md)

```bash
# Clone omniisaacgymenvs
git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git

# Install with current python executable
python3 -m pip install -e .
```



### [Omni](https://pypi.org/project/omni/)

```bash
pip install omni
```

