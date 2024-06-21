# Clash magic

Fix your proxy selection for Chat

### Step 0: install clash (linux)

- Install clash to system

```bash
sudo cp magic /usr/local/bin/clash
```

- Add `config.yaml` (find this from your service provider)
- Check availability by running `clash`



#### Optional: add clash to system service

```bash
sudo gedit /etc/systemd/system/clash.service
```

Replace [path-to-config.yaml] with your own path

 ```
 Description=Clash - A rule-based tunnel in Go
 
 Documentation=https://github.com/Dreamacro/clash/wiki
 
 [Service]
 
 OOMScoreAdjust=-1000
 
 ExecStart=/usr/local/bin/clash -f [path-to-config.yaml]
 
 Restart=on-failure
 
 RestartSec=5
 
 [Install]
 
 WantedBy=multi-user.target
 ```



Run:

```bash
sudo systemctl enable clash
sudo systemctl start clash
sudo systemctl status clash
```





### Step 1: configure new proxy group

Check the US `proxies` in your config. Proxy config looks like: 

```yaml
  -
    name: 'HK PRO 11'
    type: xxxx
    server: xxxx
    port: xxxx
    cipher: xxxx
    password: xxxx
    protocol: xxxx
    protocolparam: xxxx
    protocol-param: xxxx
    obfs: xxxx
    obfsparam: xxxx
    obfs-param: xxxx

```



Copy the names into a list and add them to this proxy group: 

```yaml
  -
    name: ChatGPT
    type: select
    proxies:
      - 'US PRO' # Replace this with your own proxy name
      - '🇺🇸 美国 IEPL' # Replace this with your own proxy name
```



Add this new proxy group to `proxy-groups`

This is a only a ***<u>sample</u>*** !!! Change the list of proxies into your own ones



### Step 2: add rules for proxy selection

Add the following lines into `rules`:

```yaml
  - DOMAIN-SUFFIX,openai.com,ChatGPT
  - DOMAIN-SUFFIX,chatgpt.com,ChatGPT
  - DOMAIN-SUFFIX,gravatar.com,ChatGPT
  - DOMAIN-SUFFIX,oaistatic.com,ChatGPT
  - DOMAIN-SUFFIX,scholar.google.com,ChatGPT
  
  # For proxy testing
  - DOMAIN-SUFFIX,whatismyipaddress.com,ChatGPT
```



### Step 3: check proxy connection

Go to `whatismyipaddress.com` to check if your ip is landed in the US