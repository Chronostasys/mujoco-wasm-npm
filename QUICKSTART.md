# MuJoCo WASM NPM Package - 快速指南

## 🎯 项目概述

这是一个自动化构建和发布 MuJoCo WebAssembly 到 npm 的完整项目。

**核心功能**：
- ✅ GitHub Actions 自动编译 MuJoCo WASM
- ✅ 自动发布到 GitHub Packages Registry
- ✅ 支持发布到公共 npm registry
- ✅ TypeScript 类型定义
- ✅ 完整的 CI/CD 流程

## 📂 项目结构

```
mujoco-wasm-npm/
├── .github/workflows/
│   └── build-and-publish.yml    # GitHub Actions CI/CD
├── scripts/
│   ├── build.js                 # 编译脚本
│   └── test.js                  # 测试脚本
├── examples/
│   ├── basic-usage.html         # 使用示例
│   └── server.js                # 本地测试服务器
├── package.json                 # npm 包配置
├── README.md                    # 包文档
├── SETUP.md                     # 详细设置指南
├── CONTRIBUTING.md              # 贡献指南
└── LICENSE                      # Apache 2.0
```

## 🚀 5分钟快速设置

### 1. 创建 GitHub 仓库

```bash
# 在 GitHub 创建新仓库，然后克隆
git clone https://github.com/YOUR-ORG/mujoco-wasm.git
cd mujoco-wasm

# 复制所有文件到仓库
cp -r /path/to/mujoco-wasm-npm/* .
```

### 2. 修改配置文件

**package.json**:
```json
{
  "name": "@YOUR-ORG/mujoco-wasm",  // 改成你的组织名
  "repository": {
    "url": "https://github.com/YOUR-ORG/mujoco-wasm.git"
  }
}
```

### 3. 配置 GitHub Actions

1. 仓库设置 → Secrets and variables → Actions
2. 添加 New repository secret:
   - Name: `NPM_TOKEN`
   - Value: 你的 GitHub Personal Access Token (需要 `write:packages` 权限)

3. 仓库设置 → Actions → General
   - Workflow permissions: ✅ Read and write permissions
   - ✅ Allow GitHub Actions to create and approve pull requests

### 4. 推送代码

```bash
git add .
git commit -m "feat: initial commit"
git push origin main
```

### 5. 发布第一个版本

```bash
# 创建并推送 tag
git tag v1.0.0
git push origin v1.0.0

# GitHub Actions 会自动：
# - 编译 MuJoCo WASM
# - 运行测试
# - 发布到 GitHub Packages
# - 创建 GitHub Release
```

## 📦 使用已发布的包

### 安装

```bash
# 从 GitHub Packages 安装
npm install @YOUR-ORG/mujoco-wasm --registry https://npm.pkg.github.com

# 或从公共 npm 安装（如果已发布）
npm install @YOUR-ORG/mujoco-wasm
```

### 在浏览器中使用

```javascript
import loadMujoco from '@YOUR-ORG/mujoco-wasm';

const mujoco = await loadMujoco();
const model = mujoco.MjModel.mj_loadXML('/path/to/model.xml');
const data = new mujoco.MjData(model);

// 运行仿真
mujoco.mj_step(model, data);
```

## 🔧 本地开发

### 安装依赖

```bash
npm install
```

### 本地构建（需要 Emscripten）

```bash
# 安装 Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest
source ./emsdk_env.sh

# 返回项目目录并构建
cd ..
npm run build
```

### 本地测试

```bash
# 运行测试
npm test

# 启动测试服务器
node examples/server.js

# 打开浏览器访问
open http://localhost:8000
```

## 📝 工作流程

### 日常开发

```bash
# 1. 创建功能分支
git checkout -b feature/add-new-feature

# 2. 修改代码
vim scripts/build.js

# 3. 测试
npm test

# 4. 提交
git add .
git commit -m "feat: add new feature"
git push origin feature/add-new-feature

# 5. 创建 Pull Request
# 在 GitHub 上创建 PR
```

### 发布新版本

```bash
# 1. 更新版本号
npm version minor  # 或 major, patch

# 2. 推送 tag
git push origin main
git push origin v1.1.0

# 3. GitHub Actions 自动发布
```

## 🎯 CI/CD 流程

### Push 到 main 分支

- ✅ 编译 MuJoCo WASM
- ✅ 运行测试
- ✅ 上传构建产物（不发布）

### 创建 Tag (v*.*.*)

- ✅ 编译 MuJoCo WASM
- ✅ 运行测试
- ✅ 发布到 GitHub Packages
- ✅ 创建 GitHub Release
- ✅ （可选）发布到公共 npm

## 🐛 常见问题

### Q: 构建失败，提示找不到 Emscripten

**A**: 需要安装 Emscripten:
```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest
source ./emsdk_env.sh
```

### Q: 发布失败，提示认证错误

**A**: 检查:
1. NPM_TOKEN 是否正确设置在 GitHub Secrets
2. GitHub Actions 是否有 write:packages 权限
3. package.json 中的包名是否正确

### Q: 无法安装发布的包

**A**:
- GitHub Packages: 需要添加 `.npmrc` 文件：
  ```
  @YOUR-ORG:registry=https://npm.pkg.github.com
  ```
- 公共 npm: 直接 `npm install @YOUR-ORG/mujoco-wasm`

## 📚 更多文档

- [SETUP.md](SETUP.md) - 详细设置指南
- [README.md](README.md) - 包使用文档
- [CONTRIBUTING.md](CONTRIBUTING.md) - 贡献指南
- [examples/](examples/) - 使用示例

## 🆘 获取帮助

- GitHub Issues: https://github.com/YOUR-ORG/mujoco-wasm/issues
- GitHub Actions: 查看仓库的 Actions 标签
- 构建日志: 在 GitHub Actions 的 workflow run 中查看

---

**准备好开始了吗？** 🚀

1. 复制所有文件到你的 GitHub 仓库
2. 修改 `package.json` 中的组织名
3. 配置 GitHub Secrets
4. 推送代码
5. 创建 tag 并触发自动发布

就这么简单！✨
