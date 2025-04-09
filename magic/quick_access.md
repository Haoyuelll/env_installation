# Quick access

- [Quick access](#quick-access)
  - [Github](#github)
    - [https://github.moeyy.xyz/](#httpsgithubmoeyyxyz)
      - [Commands](#commands)
    - [https://githubfast.com](#httpsgithubfastcom)
  - [HuggingFace](#huggingface)
    - [https://hf-mirror.com/](#httpshf-mirrorcom)
  - [PyPI](#pypi)
    - [Tuna](#tuna)


## Github

### https://github.moeyy.xyz/
- *Good for all*
- Use https instead of git@

#### Commands
- git clone
  ```bash
  git clone https://github.moeyy.xyz/[orignal git link]
  ```
- pip install
  ```bash
  pip install git+https://github.moeyy.xyz/[orignal git link]
  ```


### https://githubfast.com
- **Tips**:
  - Partly available, no web ui
  - https only
- **Usage**: Change `github.com` into `githubfast.com`



## HuggingFace

### https://hf-mirror.com/
Change `https://huggingface.co/` into `https://hf-mirror.com/`

- `hfd` tool: 

  ```bash
  wget https://hf-mirror.com/hfd/hfd.sh
  chmod a+x hfd.sh
  ```

- Env var: `HF_ENDPOINT=https://hf-mirror.com`

  - Originally: `HF_ENDPOINT=https://huggingface.co/`

- Usage: `./hfd.sh [model name or script]`
  - Gated models: `hfd meta-llama/Llama-2-7b --hf_username YOUR_HF_USERNAME --hf_token hf_***`, username may not be necesary for model downlading



### https://www.modelscope.cn/home

- Installation: `pip install modelscope`
- Download model: `modelscope download --model deepseek-ai/DeepSeek-V3-0324 README.md --local_dir ./dir`



## PyPI

### Tuna
- Single package

  ```bash
  pip install [pkg] -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```
- Same for `requirements.txt`
  ```bash
  pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```
