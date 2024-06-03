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



3. **Python module installed but not found**

```bash
Module xxx/libomni.activity.core.plugin.so remained loaded after unload request.
```

**Reason**: python path is not correctly specified if using conda 

**Solution**: create environment following official `environment.yaml`

[Doc](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html?highlight=conda#advanced-running-with-anaconda)

 ```bash
 cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
 conda env create -f environment.yaml
 ```

After virtual env installation:

```bash
conda activate isaac-sim
source $HOME/.local/share/ov/pkg/isaac_sim-2023.1.1/setup_conda_env.sh
```

Then run your python script as normal (without further need to specify python path)



4. **`xxx/setup_python_env.sh not found` when `source xxx/setup_conda_env.sh`**

**Reason**: not using bash as default shell, while the script takes working dirctory from bash. 

Solution: replace bash related variable to with universal one

Original:

```bash
#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "$BASH_SOURCE" )" && pwd )"
```

Replace it with:

```bash
#!/bin/sh
SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"
```

 

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

