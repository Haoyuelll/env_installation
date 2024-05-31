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

