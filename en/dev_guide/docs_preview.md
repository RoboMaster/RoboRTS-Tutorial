# Document Writing Guide

## Installation dependency

### NVM

Install NVM to manage node.js

```
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash
```

Install node.js

```
nvm install node # "node" is an alias for the latest version
```

Or install set version node.js

```
nvm install 6.14.4 # or 10.10.0, 8.9.1, etc
```

You can see about the available version of node.js through the following command.

```
nvm ls-remote
```



### docsify-cli

It is recommended to install docsify-cli globally for local preview.

```bash
npm i docsify-cli -g
```

Reference link: [docsify.js](https://docsify.js.org/#/)

## Open local preview

Once the dependencies are installed, they can be previewed locally in the browser via the `docsify serve` command.
Switch to the tutorial directory and run the following command.

```bash
docsify serve ./ --port 8000
```

Open browser，input`http://localhost:8000` or `http://127.0.0.1:8000`,then you can browse locally。



## Function Support

### KaTex

This file open the plug-in [docsify-katex](https://github.com/upupming/docsify-katex)by default，and it is supported to use KaTex to write mathematical symbols and formulas.

### Flexible Alerts

This file open [docsify-plugin-flexible-alerts](https://github.com/zanfab/docsify-plugin-flexible-alerts)by default，you can refer to the instructions text to know more details about the use.