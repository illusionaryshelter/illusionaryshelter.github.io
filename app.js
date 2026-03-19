/* ========================================
   太空猫的博客 — 核心应用逻辑
   路由 / Markdown文件加载 / YAML解析 / 动画 / 交互
   ======================================== */

(function () {
  'use strict';

  /* ========== Global State ========== */
  let POSTS = [];         // Loaded post objects
  let postsLoaded = false;


  /* ========== YAML Frontmatter Parser ========== */
  function parseFrontmatter(raw) {
    const match = raw.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n([\s\S]*)$/);
    if (!match) return { meta: {}, content: raw };

    const yamlStr = match[1];
    const content = match[2];
    const meta = {};

    yamlStr.split('\n').forEach(line => {
      line = line.trim();
      if (!line || line.startsWith('#')) return;

      const colonIdx = line.indexOf(':');
      if (colonIdx === -1) return;

      const key = line.slice(0, colonIdx).trim();
      let value = line.slice(colonIdx + 1).trim();

      // Remove surrounding quotes
      if ((value.startsWith('"') && value.endsWith('"')) ||
          (value.startsWith("'") && value.endsWith("'"))) {
        value = value.slice(1, -1);
      }

      // Parse inline arrays: [a, b, c]
      if (value.startsWith('[') && value.endsWith(']')) {
        value = value.slice(1, -1).split(',').map(s => {
          s = s.trim();
          if ((s.startsWith('"') && s.endsWith('"')) ||
              (s.startsWith("'") && s.endsWith("'"))) {
            s = s.slice(1, -1);
          }
          return s;
        }).filter(Boolean);
      }

      meta[key] = value;
    });

    return { meta, content };
  }

  /* Generate slug from filename */
  function slugFromFilename(filename) {
    // "2025-04-21-fast-lio-xxx.md" → "fast-lio-xxx"
    return filename
      .replace(/\.md$/i, '')
      .replace(/^\d{4}-\d{2}-\d{2}-/, '');
  }


  /* ========== Load Posts from _posts/*.md ========== */
  async function loadPosts() {
    if (postsLoaded) return;

    const promises = POST_FILES.map(async (filename) => {
      try {
        const resp = await fetch(`_posts/${filename}`);
        if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
        const raw = await resp.text();
        const { meta, content } = parseFrontmatter(raw);

        // Ensure arrays
        const categories = Array.isArray(meta.categories) ? meta.categories : (meta.categories ? [meta.categories] : []);
        const tags = Array.isArray(meta.tags) ? meta.tags : (meta.tags ? [meta.tags] : []);

        return {
          slug: slugFromFilename(filename),
          title: meta.title || filename,
          date: meta.date || '',
          categories,
          tags,
          coverImage: meta.coverImage || '',
          coverColor: meta.coverColor || 'linear-gradient(135deg, #E8734A, #F5C6D0)',
          excerpt: meta.excerpt || content.slice(0, 120).replace(/[#*`\n]/g, '') + '...',
          codeUrl: meta.codeUrl || '',
          content: content.trim()
        };
      } catch (e) {
        console.warn(`Failed to load post: ${filename}`, e);
        return null;
      }
    });

    const results = await Promise.all(promises);
    POSTS = results.filter(Boolean);

    // Sort by date descending
    POSTS.sort((a, b) => (b.date || '').localeCompare(a.date || ''));
    postsLoaded = true;
  }

  /* Helper functions */
  function getAllCategories() {
    const cats = new Set();
    POSTS.forEach(p => p.categories.forEach(c => cats.add(c)));
    return Array.from(cats);
  }

  function getAllTags() {
    const tags = new Set();
    POSTS.forEach(p => p.tags.forEach(t => tags.add(t)));
    return Array.from(tags);
  }


  /* ========== Markdown Parser ========== */
  const MD = {
    parse(md) {
      if (!md) return '';
      // Normalize line endings to \n
      let html = md.replace(/\r\n/g, '\n').replace(/\r/g, '\n');

      // Code blocks (fenced) — must be first
      html = html.replace(/```(\w*)\n([\s\S]*?)```/g, (_, lang, code) => {
        const escaped = this.escapeHtml(code.trim());
        const highlighted = this.highlightCode(escaped, lang || 'text');
        const label = lang ? `<span class="code-lang-label">${lang}</span>` : '';
        return `<pre>${label}<code>${highlighted}</code></pre>`;
      });

      // Inline code (must come after code blocks)
      html = html.replace(/`([^`\n]+)`/g, '<code>$1</code>');

      // Headers
      html = html.replace(/^#### (.+)$/gm, '<h4>$1</h4>');
      html = html.replace(/^### (.+)$/gm, '<h3>$1</h3>');
      html = html.replace(/^## (.+)$/gm, '<h2>$1</h2>');
      html = html.replace(/^# (.+)$/gm, '<h1>$1</h1>');

      // Bold & italic
      html = html.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
      html = html.replace(/\*(.+?)\*/g, '<em>$1</em>');

      // Images
      html = html.replace(/!\[([^\]]*)\]\(([^)]+)\)/g, '<img src="$2" alt="$1" loading="lazy">');

      // Links
      html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener">$1</a>');

      // Unordered lists
      html = html.replace(/^(\s*)[-*] (.+)$/gm, (match, indent, text) => `<li>${text}</li>`);
      html = html.replace(/((?:<li>.*<\/li>\n?)+)/g, '<ul>$1</ul>');

      // Blockquotes
      html = html.replace(/^> (.+)$/gm, '<blockquote><p>$1</p></blockquote>');
      html = html.replace(/<\/blockquote>\n<blockquote>/g, '\n');

      // Horizontal rules
      html = html.replace(/^---$/gm, '<hr>');

      // Paragraphs
      html = html.split('\n\n').map(block => {
        block = block.trim();
        if (!block) return '';
        if (/^<(h[1-6]|ul|ol|pre|blockquote|hr|img|div|table)/.test(block)) return block;
        if (block.startsWith('<li>')) return block;
        return `<p>${block.replace(/\n/g, '<br>')}</p>`;
      }).join('\n');

      return html;
    },

    escapeHtml(str) {
      return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
    },

    highlightCode(code, lang) {
      const cppLangs = ['cpp', 'c', 'c++', 'cc', 'cxx', 'h', 'hpp'];
      if (cppLangs.includes(lang.toLowerCase())) return this.highlightCpp(code);
      if (lang === 'python' || lang === 'py') return this.highlightPython(code);
      if (lang === 'javascript' || lang === 'js') return this.highlightJS(code);
      return code;
    },

    highlightCpp(code) {
      code = code.replace(/(\/\/[^\n]*)/g, '<span class="code-comment">$1</span>');
      code = code.replace(/(\/\*[\s\S]*?\*\/)/g, '<span class="code-comment">$1</span>');
      code = code.replace(/(#\w+)/g, '<span class="code-preprocessor">$1</span>');
      code = code.replace(/("(?:[^"\\]|\\.)*")/g, '<span class="code-string">$1</span>');
      code = code.replace(/\b(\d+\.?\d*[fFdDlL]?)\b/g, '<span class="code-number">$1</span>');
      const kw = ['if','else','for','while','do','return','void','int','float','double','char','bool','const','auto','static','class','struct','template','typename','namespace','using','new','delete','true','false','nullptr','public','private','protected','virtual','override','inline','constexpr','extern','typedef','enum','switch','case','default','break','continue','throw','try','catch','sizeof','this'];
      code = code.replace(new RegExp('\\b(' + kw.join('|') + ')\\b', 'g'), '<span class="code-keyword">$1</span>');
      const types = ['string','vector','map','set','pair','shared_ptr','unique_ptr','Ptr','Matrix\\w*','PointCloud','PointType','Eigen','pcl','std','MatrixXd','KNNResultSet','SearchParams','RegistrationOutput'];
      code = code.replace(new RegExp('\\b(' + types.join('|') + ')\\b', 'g'), '<span class="code-type">$1</span>');
      code = code.replace(/(&amp;|::|-&gt;|\.\.|&lt;&lt;|&gt;&gt;)/g, '<span class="code-operator">$1</span>');
      return code;
    },

    highlightPython(code) {
      code = code.replace(/(#[^\n]*)/g, '<span class="code-comment">$1</span>');
      code = code.replace(/("""[\s\S]*?"""|'''[\s\S]*?''')/g, '<span class="code-string">$1</span>');
      code = code.replace(/("(?:[^"\\]|\\.)*"|'(?:[^'\\]|\\.)*')/g, '<span class="code-string">$1</span>');
      code = code.replace(/\b(\d+\.?\d*)\b/g, '<span class="code-number">$1</span>');
      const kw = ['def','class','if','elif','else','for','while','return','import','from','as','try','except','finally','with','yield','lambda','pass','break','continue','raise','True','False','None','and','or','not','in','is','global','nonlocal','assert','async','await'];
      code = code.replace(new RegExp('\\b(' + kw.join('|') + ')\\b', 'g'), '<span class="code-keyword">$1</span>');
      return code;
    },

    highlightJS(code) {
      code = code.replace(/(\/\/[^\n]*)/g, '<span class="code-comment">$1</span>');
      code = code.replace(/(\/\*[\s\S]*?\*\/)/g, '<span class="code-comment">$1</span>');
      code = code.replace(/(`(?:[^`\\]|\\.)*`|"(?:[^"\\]|\\.)*"|'(?:[^'\\]|\\.)*')/g, '<span class="code-string">$1</span>');
      code = code.replace(/\b(\d+\.?\d*)\b/g, '<span class="code-number">$1</span>');
      const kw = ['const','let','var','function','return','if','else','for','while','do','switch','case','default','break','continue','new','delete','typeof','instanceof','class','extends','import','export','default','from','async','await','try','catch','finally','throw','yield','true','false','null','undefined','this','super','of','in'];
      code = code.replace(new RegExp('\\b(' + kw.join('|') + ')\\b', 'g'), '<span class="code-keyword">$1</span>');
      return code;
    }
  };


  /* ========== TOC Generator ========== */
  function generateTOC(markdown) {
    const headings = [];
    const re = /^(#{2,4}) (.+)$/gm;
    let m;
    while ((m = re.exec(markdown)) !== null) {
      headings.push({ level: m[1].length, text: m[2].trim(),
        id: m[2].trim().toLowerCase().replace(/[^\w\u4e00-\u9fff]+/g, '-').replace(/(^-|-$)/g, '')
      });
    }
    if (headings.length < 3) return '';
    let html = '<nav class="toc"><div class="toc-title">📑 目录</div><ul class="toc-list">';
    headings.forEach(h => {
      html += `<li class="toc-h${h.level}"><a href="javascript:void(0)" onclick="document.getElementById('${h.id}')?.scrollIntoView({behavior:'smooth'})">${h.text}</a></li>`;
    });
    return html + '</ul></nav>';
  }

  function addHeadingIds(html) {
    return html.replace(/<h([2-4])>(.+?)<\/h[2-4]>/g, (match, level, text) => {
      const plain = text.replace(/<[^>]+>/g, '');
      const id = plain.toLowerCase().replace(/[^\w\u4e00-\u9fff]+/g, '-').replace(/(^-|-$)/g, '');
      return `<h${level} id="${id}">${text}</h${level}>`;
    });
  }


  /* ========== Router ========== */
  function getRoute() {
    const hash = window.location.hash || '#/';
    if (hash === '#/' || hash === '#') return { page: 'home' };
    if (hash === '#/about') return { page: 'about' };
    if (hash.startsWith('#/post/')) return { page: 'post', slug: hash.slice(7) };
    if (hash.startsWith('#/category/')) return { page: 'category', name: decodeURIComponent(hash.slice(11)) };
    if (hash.startsWith('#/tag/')) return { page: 'tag', name: decodeURIComponent(hash.slice(6)) };
    return { page: 'home' };
  }


  /* ========== Renderers ========== */
  const $app = () => document.getElementById('app');

  /* ----- Cover HTML helper ----- */
  function coverHTML(post, aspectClass) {
    if (post.coverImage) {
      return `<img class="${aspectClass === 'card' ? 'post-card-cover' : ''}" src="${post.coverImage}" alt="${post.title}" loading="lazy" style="${aspectClass === 'article' ? 'width:100%;aspect-ratio:21/9;object-fit:cover;border-radius:var(--radius-lg);' : ''}">`;
    }
    const style = aspectClass === 'article'
      ? `background:${post.coverColor};aspect-ratio:21/9;border-radius:var(--radius-lg);font-size:4rem;`
      : `background:${post.coverColor}`;
    return `<div class="post-card-cover-placeholder" style="${style}">${post.categories[0]?.[0] || '📝'}</div>`;
  }

  /* ----- Home Page ----- */
  async function renderHome() {
    await loadPosts();

    const categoriesHtml = renderFilterBar();
    const postsHtml = POSTS.length === 0
      ? `<div class="empty-state"><div class="empty-state-icon">📝</div><div class="empty-state-text">暂无文章，敬请期待...</div></div>`
      : `<div class="posts-grid stagger">${POSTS.map(renderPostCard).join('')}</div>`;

    $app().innerHTML = `
      <section class="hero">
        <div class="hero-content">
          <div class="hero-avatar-wrapper">
            <img class="hero-avatar" src="assets/images/cat.jpg" alt="太空猫" onerror="this.style.background='var(--color-sakura)';this.style.display='flex';">
          </div>
          <div class="hero-greeting">Welcome</div>
          <h1 class="hero-name">你好，我是<span class="highlight">太空猫</span></h1>
          <p class="hero-tagline">
            一名热爱机器人技术与人工智能的学生，喜欢摇滚、歌剧和爵士，<br>
            在这里记录 SLAM、自动驾驶与视觉领域的探索旅程 🚀
          </p>
          <div class="hero-tags">
            <span class="hero-tag">🤖 Robotics</span>
            <span class="hero-tag">🗺️ SLAM</span>
            <span class="hero-tag">🚗 Autonomous Driving</span>
            <span class="hero-tag">👁️ Computer Vision</span>
            <span class="hero-tag">🎸 Rock & Jazz</span>
          </div>
          <div class="hero-actions">
            <a class="btn btn-primary" href="#blog-section" onclick="document.getElementById('blog-section')?.scrollIntoView({behavior:'smooth'});return false;">📖 查看文章</a>
            <a class="btn btn-outline" href="#/about">👤 关于我</a>
          </div>
        </div>
      </section>
      <section class="section" id="blog-section">
        <div class="section-header reveal">
          <div class="section-label">Blog</div>
          <h2 class="section-title">最新文章</h2>
          <div class="section-divider"></div>
        </div>
        ${categoriesHtml}
        ${postsHtml}
      </section>
    `;

    initScrollReveal();
    initFilterBar();
  }

  function renderFilterBar() {
    const cats = getAllCategories();
    if (cats.length === 0) return '';
    let html = '<div class="filter-bar reveal">';
    html += '<button class="filter-btn active" data-filter="all">全部</button>';
    cats.forEach(c => { html += `<button class="filter-btn" data-filter="${c}">${c}</button>`; });
    html += '</div>';
    return html;
  }

  function initFilterBar() {
    document.querySelectorAll('.filter-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const filter = btn.dataset.filter;
        document.querySelectorAll('.post-card').forEach(card => {
          if (filter === 'all' || card.dataset.categories.includes(filter)) {
            card.style.display = '';
            card.style.animation = 'fadeUp 0.4s var(--ease-out)';
          } else {
            card.style.display = 'none';
          }
        });
      });
    });
  }

  function renderPostCard(post) {
    const cover = coverHTML(post, 'card');
    const tagsHtml = post.tags.slice(0, 4).map(t =>
      `<a class="post-card-tag" href="#/tag/${encodeURIComponent(t)}" onclick="event.stopPropagation();">${t}</a>`
    ).join('');

    return `
      <article class="post-card" data-categories="${post.categories.join(',')}" onclick="window.location.hash='#/post/${post.slug}'">
        <div style="overflow:hidden;">${cover}</div>
        <div class="post-card-body">
          <div class="post-card-meta">
            <span class="post-card-date">📅 ${formatDate(post.date)}</span>
            ${post.categories.map(c => `<a class="post-card-category" href="#/category/${encodeURIComponent(c)}" onclick="event.stopPropagation();">${c}</a>`).join('')}
          </div>
          <h3 class="post-card-title">${post.title}</h3>
          <p class="post-card-excerpt">${post.excerpt}</p>
          <div class="post-card-tags">${tagsHtml}</div>
        </div>
      </article>
    `;
  }

  /* ----- Post Detail ----- */
  async function renderPost(slug) {
    await loadPosts();
    const post = POSTS.find(p => p.slug === slug);
    if (!post) {
      $app().innerHTML = `<div class="article-container"><a class="article-back" href="#/">← 返回首页</a><div class="empty-state"><div class="empty-state-icon">🔍</div><div class="empty-state-text">文章未找到</div></div></div>`;
      return;
    }

    const toc = generateTOC(post.content);
    let body = addHeadingIds(MD.parse(post.content));
    const readingTime = Math.max(1, Math.ceil(post.content.length / 400));

    // Article tags (clickable)
    const tagsHtml = post.tags.map(t =>
      `<a class="post-card-tag" href="#/tag/${encodeURIComponent(t)}" style="font-size:0.82rem;padding:4px 12px;">${t}</a>`
    ).join('');

    // Prev / next nav
    const idx = POSTS.findIndex(p => p.slug === slug);
    let navHtml = '';
    if (POSTS.length > 1) {
      navHtml = '<div class="article-nav">';
      if (idx > 0) navHtml += `<a class="article-nav-item" href="#/post/${POSTS[idx-1].slug}"><div class="article-nav-label">← 上一篇</div><div class="article-nav-title">${POSTS[idx-1].title}</div></a>`;
      if (idx < POSTS.length - 1) navHtml += `<a class="article-nav-item" href="#/post/${POSTS[idx+1].slug}" style="text-align:right;"><div class="article-nav-label">下一篇 →</div><div class="article-nav-title">${POSTS[idx+1].title}</div></a>`;
      navHtml += '</div>';
    }

    $app().innerHTML = `
      <div class="article-container">
        <a class="article-back" href="#/">← 返回所有文章</a>
        <div class="article-cover">${coverHTML(post, 'article')}</div>
        <header class="article-header">
          <div class="article-categories">
            ${post.categories.map(c => `<a class="article-category-badge" href="#/category/${encodeURIComponent(c)}">${c}</a>`).join('')}
          </div>
          <h1 class="article-title">${post.title}</h1>
          <div class="article-meta">
            <span class="article-meta-item">📅 ${formatDate(post.date)}</span>
            <span class="article-meta-item">⏱️ 约 ${readingTime} 分钟</span>
            ${post.codeUrl ? `<a class="article-meta-item" href="${post.codeUrl}" target="_blank" rel="noopener">💻 源代码</a>` : ''}
          </div>
          <div class="post-card-tags" style="margin-top:var(--space-md);gap:var(--space-sm);">${tagsHtml}</div>
        </header>
        ${toc}
        <div class="article-body">${body}</div>
        ${navHtml}
      </div>
    `;

    window.scrollTo({ top: 0, behavior: 'smooth' });
    initScrollReveal();
  }

  /* ----- About Page ----- */
  function renderAbout() {
    $app().innerHTML = `
      <div class="about-hero">
        <img class="about-avatar reveal" src="assets/images/cat.jpg" alt="太空猫" onerror="this.style.background='var(--color-sakura)'">
        <h1 class="about-name reveal">太空猫</h1>
        <p class="about-bio reveal">
          博主是一名大四学生，来自成都。<br>
          喜欢摇滚、歌剧和爵士，爱好历史和文学。<br>
          热情主要集中在机器人技术与人工智能的交叉领域。
        </p>
      </div>
      <section class="about-section">
        <h2 class="about-section-title reveal">🚀 技能树</h2>
        <div class="skills-grid stagger">
          <div class="skill-card"><div class="skill-card-icon" style="background:rgba(232,115,74,0.1);color:var(--color-primary)">🤖</div><h3 class="skill-card-title">机器人学</h3><p class="skill-card-desc">从机械臂的运动规划到底盘的导航控制，涵盖 FOC、MPC、sim2real 等技术方向。</p></div>
          <div class="skill-card"><div class="skill-card-icon" style="background:rgba(123,174,127,0.1);color:var(--color-secondary)">👁️</div><h3 class="skill-card-title">计算机视觉</h3><p class="skill-card-desc">学习传统视觉方法的同时，积极探索 learning-based 方法的融合应用。</p></div>
          <div class="skill-card"><div class="skill-card-icon" style="background:rgba(184,169,201,0.1);color:var(--color-wisteria)">🗺️</div><h3 class="skill-card-title">三维重建 & SLAM</h3><p class="skill-card-desc">关注多视图几何、点云处理以及视觉-惯性融合，如 real2sim、SLAM 系统构建。</p></div>
          <div class="skill-card"><div class="skill-card-icon" style="background:rgba(135,206,235,0.1);color:var(--color-sky)">🚗</div><h3 class="skill-card-title">自动驾驶</h3><p class="skill-card-desc">关注的终极应用领域之一——将感知、规划与控制融合于一体的综合系统。</p></div>
        </div>
      </section>
      <section class="about-section">
        <h2 class="about-section-title reveal">📬 联系方式</h2>
        <div class="contact-grid stagger">
          <a class="contact-card" href="mailto:illusionaryshelter@gmail.com"><div class="contact-card-icon" style="background:rgba(232,115,74,0.1);color:var(--color-primary)">✉️</div><div><div class="contact-card-label">邮箱</div><div class="contact-card-value">illusionaryshelter@gmail.com</div></div></a>
          <a class="contact-card" href="https://github.com/illusionaryshelter" target="_blank" rel="noopener"><div class="contact-card-icon" style="background:rgba(60,50,38,0.08);color:var(--color-text)">🐙</div><div><div class="contact-card-label">GitHub</div><div class="contact-card-value">@illusionaryshelter</div></div></a>
        </div>
      </section>
      <section class="about-section">
        <div class="reveal" style="background:var(--color-surface-alt);border:1px solid var(--color-border-light);border-radius:var(--radius-lg);padding:var(--space-xl);text-align:center;">
          <p style="font-size:1.05rem;color:var(--color-text-light);line-height:1.8;">
            在这个博客里，你会看到我复现经典算法的笔记、阅读前沿论文的思考，<br>
            以及在实践项目中踩过的"坑"和总结的经验。当然也会有一些别的东西 ✨
          </p>
        </div>
      </section>
    `;
    window.scrollTo({ top: 0, behavior: 'smooth' });
    initScrollReveal();
  }

  /* ----- Category Page ----- */
  async function renderCategory(name) {
    await loadPosts();
    const filtered = POSTS.filter(p => p.categories.includes(name));
    $app().innerHTML = `
      <section class="section">
        <div class="section-header reveal">
          <div class="section-label">Category</div>
          <h2 class="section-title">${name}</h2>
          <div class="section-divider"></div>
        </div>
        <div style="text-align:center;margin-bottom:var(--space-xl);"><a class="btn btn-outline" href="#/">← 返回首页</a></div>
        ${filtered.length === 0
          ? '<div class="empty-state"><div class="empty-state-icon">📭</div><div class="empty-state-text">该分类下暂无文章</div></div>'
          : `<div class="posts-grid stagger">${filtered.map(renderPostCard).join('')}</div>`}
      </section>
    `;
    initScrollReveal();
  }

  /* ----- Tag Page ----- */
  async function renderTag(name) {
    await loadPosts();
    const filtered = POSTS.filter(p => p.tags.includes(name));
    $app().innerHTML = `
      <section class="section">
        <div class="section-header reveal">
          <div class="section-label">Tag</div>
          <h2 class="section-title">#${name}</h2>
          <div class="section-divider"></div>
        </div>
        <div style="text-align:center;margin-bottom:var(--space-xl);"><a class="btn btn-outline" href="#/">← 返回首页</a></div>
        ${filtered.length === 0
          ? '<div class="empty-state"><div class="empty-state-icon">🏷️</div><div class="empty-state-text">该标签下暂无文章</div></div>'
          : `<div class="posts-grid stagger">${filtered.map(renderPostCard).join('')}</div>`}
      </section>
    `;
    initScrollReveal();
  }


  /* ========== Navigation ========== */
  function updateNav() {
    const route = getRoute();
    document.querySelectorAll('.navbar-links a').forEach(a => {
      a.classList.remove('active');
      const href = a.getAttribute('href');
      if (route.page === 'home' && (href === '#/' || href === '#')) a.classList.add('active');
      else if (route.page === 'about' && href === '#/about') a.classList.add('active');
    });
  }


  /* ========== Scroll Reveal ========== */
  function initScrollReveal() {
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
          observer.unobserve(entry.target);
        }
      });
    }, { threshold: 0.1, rootMargin: '0px 0px -40px 0px' });
    document.querySelectorAll('.reveal, .stagger').forEach(el => observer.observe(el));
  }


  /* ========== Sakura Petals ========== */
  function initSakura() {
    const colors = ['#F5C6D0', '#F8D7DD', '#FDE2E8', '#E8A0AE'];

    function createPetal() {
      const petal = document.createElement('div');
      petal.className = 'sakura-petal';
      const size = 8 + Math.random() * 14;
      const startX = Math.random() * window.innerWidth;
      const duration = 6 + Math.random() * 8;
      const delay = Math.random() * 10;
      petal.style.cssText = `width:${size}px;height:${size*0.6}px;background:${colors[Math.floor(Math.random()*colors.length)]};border-radius:${size}px 0 ${size}px 0;left:${startX}px;top:-20px;opacity:${0.3+Math.random()*0.5};animation:sakuraFall ${duration}s ${delay}s linear infinite;transform:rotate(${Math.random()*360}deg);`;
      document.body.appendChild(petal);
      setTimeout(() => petal.remove(), (duration + delay) * 1000 + 500);
    }

    if (!document.getElementById('sakura-style')) {
      const style = document.createElement('style');
      style.id = 'sakura-style';
      style.textContent = `@keyframes sakuraFall { 0%{transform:translateY(0) translateX(0) rotate(0deg);opacity:0.7} 25%{transform:translateY(25vh) translateX(30px) rotate(90deg);opacity:0.6} 50%{transform:translateY(50vh) translateX(-20px) rotate(180deg);opacity:0.5} 75%{transform:translateY(75vh) translateX(40px) rotate(270deg);opacity:0.3} 100%{transform:translateY(105vh) translateX(-10px) rotate(360deg);opacity:0} }`;
      document.head.appendChild(style);
    }

    for (let i = 0; i < 6; i++) createPetal();
    setInterval(() => { if (document.querySelectorAll('.sakura-petal').length < 25) createPetal(); }, 1500);
  }


  /* ========== Navbar Scroll ========== */
  function initNavbarScroll() {
    const navbar = document.querySelector('.navbar');
    if (!navbar) return;
    let ticking = false;
    window.addEventListener('scroll', () => {
      if (!ticking) {
        window.requestAnimationFrame(() => { navbar.classList.toggle('scrolled', window.scrollY > 30); ticking = false; });
        ticking = true;
      }
    });
  }


  /* ========== Mobile Menu ========== */
  function initMobileMenu() {
    const toggle = document.querySelector('.mobile-toggle');
    const links = document.querySelector('.navbar-links');
    if (!toggle || !links) return;

    toggle.addEventListener('click', () => {
      links.classList.toggle('open');
      const isOpen = links.classList.contains('open');
      const spans = toggle.querySelectorAll('span');
      spans[0].style.transform = isOpen ? 'rotate(45deg) translateY(7px)' : '';
      spans[1].style.opacity = isOpen ? '0' : '1';
      spans[2].style.transform = isOpen ? 'rotate(-45deg) translateY(-7px)' : '';
    });
    links.querySelectorAll('a').forEach(a => {
      a.addEventListener('click', () => {
        links.classList.remove('open');
        const spans = toggle.querySelectorAll('span');
        spans[0].style.transform = ''; spans[1].style.opacity = '1'; spans[2].style.transform = '';
      });
    });
  }


  /* ========== Cursor Trail Effect ========== */
  function initCursorTrail() {
    if ('ontouchstart' in window || navigator.maxTouchPoints > 0) return;

    const glow = document.createElement('div');
    glow.className = 'cursor-glow';
    document.body.appendChild(glow);

    const TRAIL_COUNT = 8;
    const trails = [];
    for (let i = 0; i < TRAIL_COUNT; i++) {
      const dot = document.createElement('div');
      dot.className = 'cursor-trail';
      const size = 6 - (i * 0.5);
      const opacity = 0.4 - (i * 0.04);
      dot.style.width = size + 'px';
      dot.style.height = size + 'px';
      dot.style.background = `radial-gradient(circle, rgba(232,115,74,${opacity}) 0%, rgba(245,198,208,${opacity*0.4}) 100%)`;
      document.body.appendChild(dot);
      trails.push({ el: dot, x: 0, y: 0 });
    }

    let mouseX = -100, mouseY = -100, glowX = -100, glowY = -100;
    document.addEventListener('mousemove', e => { mouseX = e.clientX; mouseY = e.clientY; });
    document.addEventListener('mouseleave', () => { glow.style.opacity = '0'; trails.forEach(t => t.el.style.opacity = '0'); });
    document.addEventListener('mouseenter', () => { glow.style.opacity = '1'; });

    function animate() {
      glowX += (mouseX - glowX) * 0.15;
      glowY += (mouseY - glowY) * 0.15;
      glow.style.transform = `translate(${glowX - 14}px, ${glowY - 14}px)`;
      let px = glowX, py = glowY;
      for (let i = 0; i < TRAIL_COUNT; i++) {
        const t = trails[i];
        t.x += (px - t.x) * (0.25 - i * 0.02);
        t.y += (py - t.y) * (0.25 - i * 0.02);
        const s = parseFloat(t.el.style.width) || 5;
        t.el.style.transform = `translate(${t.x - s/2}px, ${t.y - s/2}px)`;
        t.el.style.opacity = '1';
        px = t.x; py = t.y;
      }
      requestAnimationFrame(animate);
    }
    requestAnimationFrame(animate);
  }


  /* ========== Time Display & Theme ========== */
  function getTimeInfo() {
    const now = new Date();
    const h = now.getHours(), m = now.getMinutes(), s = now.getSeconds();
    let period, icon, greeting;
    if (h >= 6 && h < 12)       { period = 'morning';   icon = '🌅'; greeting = '早上好'; }
    else if (h >= 12 && h < 17) { period = 'afternoon'; icon = '☀️';  greeting = '下午好'; }
    else if (h >= 17 && h < 21) { period = 'evening';   icon = '🌇'; greeting = '傍晚好'; }
    else                        { period = 'night';     icon = '🌙'; greeting = '晚上好'; }
    const pad = n => String(n).padStart(2, '0');
    return { period, icon, greeting, timeStr: `${pad(h)}:${pad(m)}:${pad(s)}` };
  }

  function applyTimeTheme() {
    const { period } = getTimeInfo();
    document.body.classList.remove('theme-morning', 'theme-afternoon', 'theme-evening', 'theme-night');
    document.body.classList.add(`theme-${period}`);
  }

  function initTimeWidget() {
    const navInner = document.querySelector('.navbar-inner');
    if (!navInner) return;
    const timeEl = document.createElement('div');
    timeEl.className = 'navbar-time';
    timeEl.id = 'navbar-time';
    navInner.insertBefore(timeEl, navInner.querySelector('.mobile-toggle'));

    function update() {
      const { icon, greeting, timeStr } = getTimeInfo();
      timeEl.innerHTML = `<span class="time-icon">${icon}</span><span class="time-greeting">${greeting}</span>${timeStr}`;
    }
    applyTimeTheme();
    update();
    setInterval(() => { update(); applyTimeTheme(); }, 1000);
  }


  /* ========== Utility ========== */
  function formatDate(dateStr) {
    if (!dateStr) return '';
    const d = new Date(dateStr);
    if (isNaN(d)) return String(dateStr);
    const months = ['一月','二月','三月','四月','五月','六月','七月','八月','九月','十月','十一月','十二月'];
    return `${d.getFullYear()} 年 ${months[d.getMonth()]} ${d.getDate()} 日`;
  }


  /* ========== Page Loading ========== */
  function hideLoader() {
    const loader = document.querySelector('.page-loading');
    if (loader) {
      setTimeout(() => { loader.classList.add('done'); setTimeout(() => loader.remove(), 600); }, 300);
    }
  }


  /* ========== Router Dispatch ========== */
  async function handleRoute() {
    const route = getRoute();
    switch (route.page) {
      case 'home':     await renderHome(); break;
      case 'post':     await renderPost(route.slug); break;
      case 'about':    renderAbout(); break;
      case 'category': await renderCategory(route.name); break;
      case 'tag':      await renderTag(route.name); break;
      default:         await renderHome();
    }
    updateNav();
  }


  /* ========== Init ========== */
  async function init() {
    await handleRoute();
    hideLoader();
    initNavbarScroll();
    initMobileMenu();
    initSakura();
    initCursorTrail();
    initTimeWidget();
  }

  window.addEventListener('hashchange', handleRoute);
  document.addEventListener('DOMContentLoaded', init);

})();
