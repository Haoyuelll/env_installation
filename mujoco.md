# Mujoco & mujoco_py
### General description: 

1. Configure environment (please check the doc below for implicit dependencies)
2. Download Mujoco and move the folder to `[path_to_home]/.mujoco/mujoco[version]`
3. Download mj key and place it under the same home directory
4. Install `mujoco_py`



### Useful reference:

Commands and common errors from pytorch [doc](https://pytorch.org/rl/main/reference/generated/knowledge_base/MUJOCO_INSTALLATION.html)



### What's annoying:

Officially, `mujoco_py` only  supports `mujoco200` and below, which is almost deprecated and kicked out from Mujoco's downloading page [lol].
Fortunately, `mujoco200` package can still be found by directly searching the web (by May 2024).
Installing higher verision and cheating path name work for mujoco_py installation itself, but could lead to dependency error while actually running. Check doc for your installed version to avoid this.



### Path declaration

**Path declaration is only needed for mujoco_py installation.**

```bash
# Path declaration that may or may not work
# Change "${ZDOTDIR:-$HOME}" to your mujoco home
# Change ".bashrc" to your shell config

echo "export LD_LIBRARY_PATH=${ZDOTDIR:-$HOME}/.mujoco/mujoco200/bin/${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ${ZDOTDIR:-$HOME}/.bashrc

echo "export MUJOCO_KEY_PATH=${ZDOTDIR:-$HOME}/.mujoco/${MUJOCO_KEY_PATH}" >> ${ZDOTDIR:-$HOME}/.bashrc
```



Exporting path to `.bashrc` (or `.zshrc`, etc.) may not work for unknown reasons.
One naive way to get through is to add the path declaration before every run [lol].



### GLIBCXX

```bash
# System dep is installed at "/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
# However, this dep cannot be pre-loaded for mujoco_py

# Export path from conda
echo "export LD_PRELOAD=$LD_PRELOAD:$CONDA_PREFIX/lib/libstdc++.so.6" >> ${ZDOTDIR:-$HOME}/.bashrc

# Unfortunately, conda may not be trust worthy
# Check availible GLIBCXX version if error occurs
strings $CONDA_PREFIX/lib/libstdc++.so.6 | grep GLIBCXX
# GLIBCXX_3.4.30 is probably not availble [lol]


# What makes things worse is that, libstdc++ is not availble for manual update from conda source
# What we do here is check system version, update (if needed) and map it to conda

# Check system availible GLIBCXX version
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX

# Update if needed (or build from source https://pkgs.org/)
sudo apt install libstdc++

# Create symbolic link from system package
mv $CONDA_PREFIX/lib/libstdc++.so.6 $CONDA_PREFIX/lib/libstdc++.so.6.old
ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 $CONDA_PREFIX/lib/libstdc++.so.6
# Reference: https://github.com/isl-org/Open3D/issues/5531
```
