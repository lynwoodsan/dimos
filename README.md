# gh-pages — Dimensional GitHub Pages

**Live at:** https://dimensionalos.github.io/dimos/

This branch hosts static HTML pages for Dimensional's public web presence (careers, resources, etc.). It's completely separate from the DimOS codebase.

## Structure

```
gh-pages/
├── index.html          # Main landing page
├── README.md           # This file (instructions)
├── careers/            # Example subpage directory
│   └── index.html      # /dimos/careers/
└── assets/             # Shared images, CSS, etc. (optional)
    ├── style.css
    └── logo.png
```

## How to Add Content

### Edit the main page

Edit `index.html` directly. Add new `<section>` blocks:

```html
<section id="my-section">
  <h2>Section Title</h2>
  <p>Content here.</p>
</section>
```

### Add a subpage

Create a directory with an `index.html`:

```bash
mkdir -p careers
cat > careers/index.html << 'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Careers — DimensionalOS</title>
  <style>
    /* Copy base styles from index.html, or link a shared stylesheet */
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, system-ui, sans-serif; background: #0a0e1a; color: #e2e8f0; min-height: 100vh; }
    a { color: #3b82f6; }
    .container { max-width: 720px; margin: 0 auto; padding: 3rem 1.5rem; }
    h1 { font-size: 1.8rem; margin-bottom: 1rem; }
    p { color: #94a3b8; line-height: 1.7; margin-bottom: 1rem; }
    .back { color: #64748b; font-size: 0.9rem; }
  </style>
</head>
<body>
  <div class="container">
    <a href="/dimos/" class="back">← Back to home</a>
    <h1>Careers at Dimensional</h1>
    <p>Your content here...</p>
  </div>
</body>
</html>
EOF
```

The page will be available at `https://dimensionalos.github.io/dimos/careers/`.

### Link subpages from the main page

Add links in `index.html`:

```html
<li><a href="/dimos/careers/">Careers</a></li>
```

**Important:** All paths must be prefixed with `/dimos/` because this is a project site (not a user/org site).

## How to Deploy

Just push to the `gh-pages` branch. GitHub auto-deploys within ~30 seconds.

```bash
# From any local clone
git checkout gh-pages
# Make your changes...
git add -A
git commit -m "update: description of changes"
git push origin gh-pages
```

Or edit files directly on GitHub (web UI) — commits to `gh-pages` auto-deploy.

## Rules

- **Raw HTML only.** No build tools, no frameworks, no npm. Just `.html`, `.css`, `.js`, and images.
- **All paths use `/dimos/` prefix.** This is a GitHub Project Pages site, not a root site.
- **Don't touch other branches.** This branch is isolated from the DimOS codebase.
- **Keep it lightweight.** This is a simple web presence, not a full app.
- **Dark theme.** Match the existing style (background `#0a0e1a`, text `#e2e8f0`, accent `#3b82f6`).

## Style Reference

| Token | Value | Usage |
|-------|-------|-------|
| `--bg` | `#0a0e1a` | Page background |
| `--surface` | `#111827` | Card/section backgrounds |
| `--border` | `#1e3a5f` | Borders, dividers |
| `--accent` | `#3b82f6` | Links, buttons, highlights |
| `--text` | `#e2e8f0` | Primary text |
| `--muted` | `#94a3b8` | Body text, descriptions |
| `--dim` | `#64748b` | Footer, subtle text |

## For AI Agents

If you're an AI agent adding pages:

1. **Check out `gh-pages`:** `GIT_LFS_SKIP_SMUDGE=1 git checkout gh-pages`
2. **Create your page** as a directory with `index.html` (see examples above)
3. **Link it** from `index.html` in the appropriate section
4. **Commit with `PRE_COMMIT_ALLOW_NO_CONFIG=1`** — there's no pre-commit config on this branch
5. **Push to `gh-pages`** — auto-deploys in ~30s
6. **Switch back:** `GIT_LFS_SKIP_SMUDGE=1 git checkout -f dev`

**Path gotcha:** Always prefix paths with `/dimos/`. A link to `careers/` works relatively, but absolute paths need `/dimos/careers/`.

**No pre-commit:** This branch has no `.pre-commit-config.yaml`. Use `PRE_COMMIT_ALLOW_NO_CONFIG=1 git commit ...` to avoid errors.
