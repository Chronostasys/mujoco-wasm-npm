# MuJoCo WASM NPM Package - Setup Guide

这个项目提供了一个自动化的 CI/CD 流程，用于编译 MuJoCo 物理引擎到 WebAssembly 并发布到 npm registry。

## 📁 项目结构

```
mujoco-wasm-npm/
├── .github/
│   └── workflows/
│       └── build-and-publish.yml    # GitHub Actions 工作流
├── scripts/
│   ├── build.js                      # MuJoCo WASM 编译脚本
│   └── test.js                       # 测试脚本
├── package.json                      # npm 包配置
├── README.md                         # 包文档
├── LICENSE                           # Apache 2.0 许可证
├── .gitignore
└── .npmignore
```

## 🚀 快速开始

### 1. 创建 GitHub 仓库

```bash
# 在 GitHub 上创建新仓库
# 然后克隆到本地
git clone https://github.com/your-org/mujoco-wasm.git
cd mujoco-wasm

# 复制项目文件
# （确保所有文件都在仓库中）
```

### 2. 配置 GitHub Packages

#### a. 启用 GitHub Packages

1. 进入 GitHub 仓库设置
2. 点击 "Settings" → "Actions" → "General"
3. 滚动到 "Workflow permissions"
4. 选择 "Read and write permissions"
5. 勾选 "Allow GitHub Actions to create and approve pull requests"
6. 保存

#### b. 配置 npm registry

GitHub Packages 需要认证。创建一个 Personal Access Token:

```bash
# 生成 Personal Access Token (classic)
# GitHub Settings → Developer settings → Personal access tokens → Tokens (classic)
# 权限: write:packages, repo
```

将 token 添加到 GitHub Secrets:
1. 仓库设置 → "Secrets and variables" → "Actions"
2. 添加新的 secret:
   - Name: `NPM_TOKEN`
   - Value: 你的 personal access token

### 3. 配置 package.json

修改 `package.json` 中的字段:

```json
{
  "name": "@your-org/mujoco-wasm",  // 替换为你的组织名
  "repository": {
    "url": "https://github.com/your-org/mujoco-wasm.git"
  }
}
```

### 4. 配置 GitHub Actions

编辑 `.github/workflows/build-and-publish.yml`:

```yaml
# 第 61 行，修改为你的组织名
registry-url: 'https://npm.pkg.github.com'

# 第 120 行，改为你的仓库
destination_dir: preview/${{ github.sha }}
```

## 📦 发布流程

### 自动发布（推荐）

当创建新的 git tag 时，GitHub Actions 会自动：

1. 编译 MuJoCo WASM
2. 运行测试
3. 发布到 GitHub Packages Registry
4. 创建 GitHub Release
5. （可选）发布到公共 npm registry

```bash
# 创建并推送 tag
git tag v2.3.8
git push origin v2.3.8
```

### 手动发布

```bash
# 1. 安装 Emscripten
# (参见: https://emscripten.org/docs/getting_started/downloads.html)

# 2. 构建
npm run build

# 3. 测试
npm test

# 4. 发布到 GitHub Packages
npm publish

# 或发布到公共 npm
npm publish --access public --registry https://registry.npmjs.org
```

## 🔧 开发工作流

### 本地构建

```bash
# 安装 Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh

# 返回项目目录并构建
cd ../mujoco-wasm-npm
npm run build

# 查看构建产物
ls -lh dist/
```

### 测试

```bash
# 运行测试脚本
npm test

# 或手动测试（在 Node.js 或浏览器中）
node test/manual-test.js
```

### 提交更改

```bash
# 修改代码
vim scripts/build.js

# 提交
git add .
git commit -m "feat: improve build performance"

# 推送
git push origin main

# GitHub Actions 会自动运行构建测试（但不发布）
```

## 📊 监控发布

### 检查 GitHub Actions

1. 访问仓库的 "Actions" 标签
2. 查看最新的 workflow run
3. 检查每个 job 的状态

### 检查已发布的包

**GitHub Packages**:
```bash
npm view @your-org/mujoco-wasm --registry https://npm.pkg.github.com
```

**公共 npm**:
```bash
npm view @your-org/mujoco-wasm
```

### 安装测试

```bash
# 从 GitHub Packages 安装
npm install @your-org/mujoco-wasm --registry https://npm.pkg.github.com

# 或从公共 npm 安装
npm install @your-org/mujoco-wasm
```

## 🐛 故障排查

### 构建失败

**问题**: Emscripten 未找到
```bash
# 解决: 安装并激活 Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest
source ./emsdk_env.sh
```

**问题**: MuJoCo 克隆失败
```bash
# 解决: 检查网络连接或使用代理
git config --global http.proxy http://proxy.example.com:8080
```

**问题**: 内存不足
```bash
# 解决: 限制 Emscripten 内存使用
export EMCC_DEBUG=1
emcc -s INITIAL_MEMORY=256MB ...
```

### 发布失败

**问题**: 认证失败
```bash
# 解决: 检查 NPM_TOKEN 是否正确设置
# GitHub Settings → Secrets → Actions → NPM_TOKEN
```

**问题**: 权限不足
```bash
# 解决: 确保 GitHub Actions 有 write:packages 权限
# Settings → Actions → Workflow permissions → Read and write permissions
```

## 📝 版本管理

推荐使用语义化版本 (Semantic Versioning):

- **MAJOR.MINOR.PATCH** (如 2.3.8)
- MAJOR: 不兼容的 API 变更
- MINOR: 向后兼容的功能新增
- PATCH: 向后兼容的问题修复

### 发布新版本

```bash
# 1. 更新 package.json 中的版本
npm version patch  # 或 minor, major

# 2. 提交并打 tag
git add package.json
git commit -m "chore: bump version to 2.3.9"
git tag v2.3.9

# 3. 推送
git push origin main
git push origin v2.3.9

# 4. GitHub Actions 自动发布
```

## 🔐 安全考虑

1. **不要提交敏感信息**:
   - ❌ NPM_TOKEN
   - ❌ API keys
   - ❌ 密码

2. **使用 GitHub Secrets**:
   - ✅ NPM_TOKEN 存储在 GitHub Secrets
   - ✅ GITHUB_TOKEN 自动注入

3. **限制 token 权限**:
   - 只授予必要的权限 (write:packages)
   - 定期轮换 token

4. **审查依赖**:
   - 定期更新依赖
   - 使用 `npm audit` 检查漏洞

## 📚 相关资源

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Emscripten Documentation](https://emscripten.org/docs/)
- [GitHub Packages Guide](https://docs.github.com/en/packages/learn-github-packages/introduction-to-github-packages)
- [npm Registry Documentation](https://docs.npmjs.com/)
- [Semantic Versioning](https://semver.org/)

## 🆘 获取帮助

- GitHub Issues: https://github.com/your-org/mujoco-wasm/issues
- MuJoCo Forum: https://mujoco.org/forum/
- Emscripten Discord: https://discord.gg/EMj86GXp6U

---

**Happy building! 🚀**
