#!/bin/bash
# MuJoCo WASM NPM Package - 初始化脚本

set -e

echo "=========================================="
echo "MuJoCo WASM NPM Package - 初始化"
echo "=========================================="
echo ""

# 检查是否在正确的目录
if [ ! -f "package.json" ]; then
    echo "❌ 错误: 请在项目根目录运行此脚本"
    exit 1
fi

# 获取组织名称
read -p "请输入你的 GitHub 组织名称 (例如: your-org): " ORG_NAME

if [ -z "$ORG_NAME" ]; then
    echo "❌ 组织名称不能为空"
    exit 1
fi

# 替换 package.json 中的占位符
echo ""
echo "📝 更新 package.json..."
sed -i.bak "s/@your-org/@$ORG_NAME/g" package.json
rm package.json.bak

# 创建 .npmrc（如果需要）
if [ ! -f ".npmrc" ]; then
    echo ""
    read -p "是否创建 .npmrc 文件? (y/n): " CREATE_NPMRC

    if [ "$CREATE_NPMRC" = "y" ]; then
        cat > .npmrc << EOF
@$ORG_NAME:registry=https://npm.pkg.github.com
EOF
        echo "✅ .npmrc 文件已创建"
    fi
fi

# 创建 .gitignore（如果不存在）
if [ ! -f ".gitignore" ]; then
    cat > .gitignore << 'EOF'
node_modules/
dist/
build/
*.log
.DS_Store
EOF
    echo "✅ .gitignore 文件已创建"
fi

echo ""
echo "=========================================="
echo "✅ 初始化完成！"
echo "=========================================="
echo ""
echo "📋 下一步:"
echo ""
echo "1. 创建 GitHub 仓库:"
echo "   https://github.com/new"
echo "   仓库名: mujoco-wasm"
echo ""
echo "2. 初始化 Git 仓库:"
echo "   git init"
echo "   git add ."
echo "   git commit -m 'feat: initial commit'"
echo "   git remote add origin https://github.com/$ORG_NAME/mujoco-wasm.git"
echo "   git push -u origin main"
echo ""
echo "3. 在 GitHub 配置 Secrets:"
echo "   Settings → Secrets and variables → Actions"
echo "   添加: NPM_TOKEN"
echo ""
echo "4. 创建并推送 tag (触发发布):"
echo "   git tag v1.0.0"
echo "   git push origin v1.0.0"
echo ""
echo "📚 更多信息:"
echo "   - QUICKSTART.md: 快速开始指南"
echo "   - SETUP.md: 详细设置指南"
echo "   - README.md: 包使用文档"
echo ""
echo "=========================================="
