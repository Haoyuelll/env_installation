# ZSH

## Installation

```bash
sudo apt install zsh
```

- Oh my zsh: `sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"`
- Change shell with: `chsh -s $(which zsh)`, reboot to apply changes 



## Configuration

- Add config in `.zshrc`

### Theme

- p10k

```bash
git clone --depth=1 https://gitee.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
```



### Plugins

- `sudo`
- `zsh-syntax-highlighting`

```
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```

