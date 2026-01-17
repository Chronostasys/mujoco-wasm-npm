# Contributing to MuJoCo WASM

感谢你对 MuJoCo WASM 项目的兴趣！我们欢迎各种形式的贡献。

## 🤝 如何贡献

### 报告 Bug

在创建 issue 之前，请先搜索现有的 issues。

报告 bug 时请提供：

- 清晰的标题和描述
- 复现步骤
- 期望行为
- 实际行为
- 环境信息（浏览器、Node.js 版本、操作系统）
- 最小化的复现代码

### 提交 Feature Request

- 使用清晰的标题
- 详细描述功能需求
- 说明使用场景和好处
- 提供示例代码（如果可能）

### 提交 Pull Request

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'feat: add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建 Pull Request

### Commit Message 规范

使用 [Conventional Commits](https://www.conventionalcommits.org/) 格式:

- `feat:` 新功能
- `fix:` Bug 修复
- `docs:` 文档更新
- `style:` 代码格式（不影响功能）
- `refactor:` 重构
- `perf:` 性能优化
- `test:` 测试相关
- `chore:` 构建/工具相关

示例:
```
feat: add support for MuJoCo 2.3.9
fix: resolve memory leak in wasm module
docs: update installation instructions
```

## 🛠️ 开发环境设置

### 必需工具

- Node.js 16+
- Emscripten 3.1+
- Git

### 安装 Emscripten

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

### 本地构建

```bash
# 克隆仓库
git clone https://github.com/your-org/mujoco-wasm.git
cd mujoco-wasm

# 构建
npm install
npm run build

# 测试
npm test
```

## 📐 代码规范

### JavaScript/TypeScript

- 使用 ES6+ 语法
- 遵循 [Standard Style](https://standardjs.com/)
- 添加适当的注释

### Python Scripts

- 遵循 PEP 8
- 添加类型提示
- 编写文档字符串

## 🧪 测试

### 运行测试

```bash
npm test
```

### 测试覆盖

确保新功能有相应的测试：

- 单元测试
- 集成测试
- 浏览器测试

## 📚 文档

保持文档更新：

- README.md - 主要文档
- API.md - API 参考
- EXAMPLES.md - 使用示例
- CHANGELOG.md - 变更日志

## 🎯 代码审查流程

1. 所有 PR 需要通过至少一个维护者审查
2. 确保所有测试通过
3. 更新相关文档
4. 遵循代码规范
5. 添加适当的测试

## 📝 发布流程

只有维护者可以发布新版本：

1. 更新版本号
2. 更新 CHANGELOG
3. 创建 git tag
4. 推送到 GitHub
5. GitHub Actions 自动发布

## 💬 讨论

对于较大的变更或设计讨论：

1. 先创建 issue 讨论
2. 获得社区反馈
3. 达成一致后再实现

## 🌟 贡献者

感谢所有贡献者！你的名字会被添加到贡献者列表中。

## 📜 行为准则

- 尊重所有贡献者
- 欢迎不同意见
- 建设性的批评
- 关注什么对社区最有利

## 📧 联系方式

- GitHub Issues: https://github.com/your-org/mujoco-wasm/issues
- Email: your.email@example.com

---

再次感谢你的贡献！🎉
