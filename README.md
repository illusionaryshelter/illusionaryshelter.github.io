# 🐱 太空猫的个人博客

一个现代化的纯前端个人博客，零依赖，可直接部署到 GitHub Pages。

## ✨ 特性

- 🌸 **背景动画**：CSS + JS 驱动的随机花瓣效果
- 🖱️ **鼠标光标跟随**：发光光晕 + 8 粒子拖尾
- ⏰ **实时时钟**：导航栏显示时间 + 问候语（早上好/下午好/傍晚好/晚上好）
- 📝 **Markdown 驱动**：文章用 `.md` 文件编写，带 YAML frontmatter
- 🎯 **C++ / Python / JS 语法高亮**：内置轻量代码高亮器
- 🏷️ **标签 & 分类导航**：点击跳转筛选
- 📱 **完全响应式**：移动端自适应 + 汉堡菜单
- 🔍 **自动目录生成**：文章超过 3 个标题自动生成 TOC

## 📁 项目结构

```
├── index.html          # 入口页面（SEO、导航栏、页脚）
├── style.css           # 设计系统（配色、布局、动画、响应式）
├── app.js              # 核心逻辑（路由、MD渲染、动画、交互）
├── posts.js            # 文章索引（只需列出 .md 文件名）
├── _posts/             # 📂 文章目录 ← 在这里写文章
│   └── 2025-04-21-fast-lio-xxx.md
└── assets/images/
    ├── cat.jpg          # 头像
    └── themes/          # 时段背景图（morning/afternoon/evening/night.jpg）
```

## 📝 如何发布新文章

### 第一步：写文章

在 `_posts/` 目录下创建 `.md` 文件，文件名格式推荐 `YYYY-MM-DD-标题.md`：

```markdown
---
title: "我的文章标题"
date: 2026-03-19
categories: [分类1, 分类2]
tags: [标签A, 标签B, 标签C]
coverImage: assets/images/my-cover.jpg
excerpt: "这篇文章的简短摘要..."
codeUrl: https://github.com/xxx/yyy
---

## 正文从这里开始

支持完整的 Markdown 语法：**加粗**、*斜体*、`行内代码`、[链接](url)、![图片](url)

### 代码块（自动语法高亮）

​```cpp
int main() {
    std::cout << "Hello World" << std::endl;
    return 0;
}
​```
```

#### Frontmatter 字段说明

| 字段 | 必填 | 说明 |
|------|------|------|
| `title` | ✅ | 文章标题 |
| `date` | ✅ | 发布日期，格式 `YYYY-MM-DD` |
| `categories` | ❌ | 分类数组，如 `[SLAM, LIO]` |
| `tags` | ❌ | 标签数组，如 `[C++, Fast-LIO]` |
| `coverImage` | ❌ | 封面图路径（相对路径，不要以 `/` 开头） |
| `coverColor` | ❌ | 无封面图时的渐变色，如 `linear-gradient(135deg, #E8734A, #F5C6D0)` |
| `excerpt` | ❌ | 摘要，缺省时自动截取正文前 120 字 |
| `codeUrl` | ❌ | 源代码链接，显示在文章头部 |

### 第二步：注册文章

打开 `posts.js`，在 `POST_FILES` 数组中添加一行文件名：

```javascript
const POST_FILES = [
  '2025-04-21-fast-lio-enhancement-with-gicp-and-scan-context.md',
  '2026-03-19-my-new-post.md',  // ← 加这一行
];
```

### 第三步：推送

```bash
git add _posts/your-new-post.md posts.js
git commit -m "feat: add new post"
git push
```

## 🎨 DIY 定制指南

### 修改个人信息

- **昵称 / 头像 / 简介**：在 `app.js` 中搜索 `太空猫` 修改
- **头像图片**：替换 `assets/images/cat.jpg`
- **社交链接**：在 `index.html` 的导航栏和页脚中修改

### 修改配色

编辑 `style.css` 顶部的 CSS 变量：

```css
:root {
  --color-primary:      #E8734A;   /* 主色 — 改这个 */
  --color-sakura:       #F5C6D0;   /* 樱花色 */
  --color-secondary:    #7BAE7F;   /* 辅助绿色 */
  --color-bg:           #FDF8F0;   /* 背景色 */
}
```

### 时段背景图

将 4 张图片放入 `assets/images/themes/` 目录：

| 文件名 | 时段 | 建议风格 |
|--------|------|----------|
| `morning.jpg` | 6:00-11:59 | 日出、晨光 |
| `afternoon.jpg` | 12:00-16:59 | 蓝天、花田 |
| `evening.jpg` | 17:00-20:59 | 夕阳、黄昏 |
| `night.jpg` | 21:00-5:59 | 星空、月夜 |

透明度可在 `style.css` 中搜索 `theme-morning::after` 调整 `opacity` 值。

### 插入图片

文章中的图片支持两种方式：

```markdown
<!-- 本地图片（放在 assets/images/ 下） -->
![描述](assets/images/my-photo.jpg)

<!-- 远程图片 -->
![描述](https://example.com/photo.jpg)
```

所有图片自动响应式适配、圆角、阴影。

## 🚀 本地预览

```bash
cd illusionaryshelter.github.io
python3 -m http.server 8888
# 打开 http://localhost:8888
```

## 📄 License

MIT
