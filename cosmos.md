# Cosmos

### Quick view

- conda & env check
- ckpt



## Installation

```bash
git clone https://github.com:nvidia-cosmos/cosmos-transfer1.git
cd cosmos-transfer1
git submodule update --init --recursive
```

- Conda env:

  ```bash
  # Create the cosmos-transfer1 conda environment.
  conda env create --file cosmos-transfer1.yaml
  # Activate the cosmos-transfer1 conda environment.
  conda activate cosmos-transfer1
  # Install the dependencies.
  pip install -r requirements.txt
  # Patch Transformer engine linking issues in conda environments.
  ln -sf $CONDA_PREFIX/lib/python3.10/site-packages/nvidia/*/include/* $CONDA_PREFIX/include/
  ln -sf $CONDA_PREFIX/lib/python3.10/site-packages/nvidia/*/include/* $CONDA_PREFIX/include/python3.10
  # Install Transformer engine.
  pip install transformer-engine[pytorch]==1.12.0

- TUNA mirror

  ```bash
  # Create the cosmos-transfer1 conda environment.
  conda env create --file cosmos-transfer1.yaml
  # Activate the cosmos-transfer1 conda environment.
  conda activate cosmos-transfer1
  # Install the dependencies.
  pip install -r requirements.txt -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
  # Patch Transformer engine linking issues in conda environments.
  ln -sf $CONDA_PREFIX/lib/python3.10/site-packages/nvidia/*/include/* $CONDA_PREFIX/include/
  ln -sf $CONDA_PREFIX/lib/python3.10/site-packages/nvidia/*/include/* $CONDA_PREFIX/include/python3.10
  # Install Transformer engine.
  pip install transformer-engine[pytorch]==1.12.0 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
  ```

- `no matches found`: `pip install --use-pep517 [pkg name]`



## Download checkpoint

### Gated repo

- `LlamaGuard-7b`: hard to get approved
- `nvidia/Cosmos-Transfer1-7B`
- `nvidia/Cosmos-Tokenize1-CV8x8x8-720p`

### Command to run

- Original hf:

  ```bash
  CUDA_HOME=$CONDA_PREFIX PYTHONPATH=$(pwd) python scripts/download_checkpoints.py --output_dir checkpoints/ --model 7b --hf_token [your token]
  ```

- Use hf-mirror:

  ```bash
  HF_ENDPOINT=https://hf-mirror.com CUDA_HOME=$CONDA_PREFIX PYTHONPATH=$(pwd) python scripts/download_checkpoints.py --output_dir checkpoints/ --model 7b --hf_token [your token]
  ```

  

