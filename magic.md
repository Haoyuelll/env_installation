# Clash magic

Fix your proxy selection for Chat



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