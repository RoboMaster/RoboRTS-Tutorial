# 文档编写指南

## 安装依赖

### NVM

安装NVM用于管理node.js。

```
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh | bash
```

安装node.js

```
nvm install node # "node" is an alias for the latest version
```

或安装制定版本node.js

```
nvm install 6.14.4 # or 10.10.0, 8.9.1, etc
```

可以通过以下命令查询node.js可用版本

```
nvm ls-remote
```



### docsify-cli

推荐全局安装docsify-cli用于本地预览。

```bash
npm i docsify-cli -g
```

参考链接: [docsify.js](https://docsify.js.org/#/)

## 开启本地预览

安装好依赖后，可以通过`docsify serve`命令来在浏览器中本地预览。

切换到教程目录，运行如下命令。

```bash
docsify serve ./ --port 8000
```

打开浏览器，输入`http://localhost:8000` 或 `http://127.0.0.1:8000`即可实时本地预览。



## 功能支持

### KaTex

本文档默认开启[docsify-katex](https://github.com/upupming/docsify-katex)插件，支持利用KaTex编写数学符号和数学公式。

### Flexible Alerts

本文档默认开启[docsify-plugin-flexible-alerts](https://github.com/zanfab/docsify-plugin-flexible-alerts)，详细使用可以参考其说明文档。