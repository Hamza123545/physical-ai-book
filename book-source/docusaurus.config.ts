import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
import * as dotenv from "dotenv";
import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";

// Load environment variables from .env file (for local development)
// Production uses actual environment variables set in CI/CD
dotenv.config();

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Docs path for Physical AI & Humanoid Robotics book
const docsPath = "docs";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline:
    "Hamza Swati - Building Intelligent Humanoid Robots with AI",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  // Update with your GitHub Pages URL
  url: "https://Hamza123545.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/physical-ai-book/",

  // GitHub pages deployment config.
  organizationName: "Hamza123545", // Your GitHub username
  projectName: "physical-ai-book", // Your repo name
  trailingSlash: false,

  onBrokenLinks: "warn",

  // Add Font Awesome for social media icons
  headTags: [
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css",
        integrity:
          "sha512-DTOQO9RWCH3ppGqcWaEA1BIZOC6xxalwEsw9c2QQeAIftl+Vegovlnee1c9QX4TctnWMn13TZye+giMm8e2LwA==",
        crossorigin: "anonymous",
        referrerpolicy: "no-referrer",
      },
    },
    // KaTeX stylesheet for mathematical notation rendering
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css",
        integrity:
          "sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV",
        crossorigin: "anonymous",
      },
    },
    // Pyodide CDN for browser-based Python execution
    {
      tagName: "script",
      attributes: {
        src: "https://cdn.jsdelivr.net/pyodide/v0.24.1/full/pyodide.js",
        crossorigin: "anonymous",
      },
    },
    // OpenAI ChatKit for RAG chatbot
    {
      tagName: "script",
      attributes: {
        src: "https://cdn.platform.openai.com/deployments/chatkit/chatkit.js",
        async: "true",
      },
    },
    // Google Analytics 4 (GA4) - Configure with environment variable
    // See docs/ANALYTICS/ga4-setup.md for setup instructions
    ...(process.env.GA4_MEASUREMENT_ID
      ? [
          {
            tagName: "script",
            attributes: {
              async: "true",
              src: `https://www.googletagmanager.com/gtag/js?id=${process.env.GA4_MEASUREMENT_ID}`,
            },
          },
          {
            tagName: "script",
            attributes: {},
            innerHTML: `
          window.dataLayer = window.dataLayer || [];
          function gtag(){dataLayer.push(arguments);}
          gtag('js', new Date());
          gtag('config', '${process.env.GA4_MEASUREMENT_ID}', {
            'anonymize_ip': true,
            'allow_google_signals': false,
            'allow_ad_personalization_signals': false
          });
        `,
          },
        ]
      : []),
  ],

  // Internationalization (i18n) configuration
  // Supports English (default) and Urdu translations
  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
    localeConfigs: {
      en: {
        label: "English",
        direction: "ltr",
        htmlLang: "en-US",
      },
      ur: {
        label: "اردو",
        direction: "rtl", // Right-to-left for Urdu
        htmlLang: "ur-PK",
      },
    },
  },
  
  presets: [
    [
      "classic",
      {
        docs: {
          path: docsPath, // 'docs' (local) or 'docsfs' (from MCP server)
          sidebarPath: "./sidebars.ts",
          // Exclude .summary.md files from being rendered as pages
          // They are injected into lesson frontmatter by the summary injector plugin
          exclude: ["**/*.summary.md"],
          // Enable i18n for docs - routeBasePath ensures consistent routing
          routeBasePath: "docs",
          // Ensure proper locale routing - editLocalizedFiles allows editing translated files
          editLocalizedFiles: false,
          showLastUpdateAuthor: false,
          showLastUpdateTime: false,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          // beforeDefaultRemarkPlugins run BEFORE Docusaurus's internal plugins
          // This is critical for modifying frontmatter via file.data.frontMatter
          beforeDefaultRemarkPlugins: [
            // Summary injection handled by docusaurus-summaries-plugin (global data approach)
          ],
          remarkPlugins: [
            remarkMath,
            // Optional: Add interactive Python plugin if needed
            // [require('./plugins/remark-interactive-python'), {
            //   includePaths: ['/'],
            //   excludeMeta: ['nointeractive', 'static'],
            // }],
          ],
          rehypePlugins: [
            rehypeKatex,
          ],
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
        // Sitemap configuration for search engines
        sitemap: {
          changefreq: "weekly",
          priority: 0.5,
          filename: "sitemap.xml",
          ignorePatterns: ["**/tags/**"],
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    // Optional plugins - uncomment if needed
    // "./plugins/docusaurus-plugin-og-image-generator",
    // "./plugins/docusaurus-plugin-structured-data",
    function (context, options) {
      return {
        name: "custom-webpack-config",
        configureWebpack(config, isServer, utils) {
          const path = require("path");
          return {
            resolve: {
              alias: {
                "@": path.resolve(__dirname, "src"),
              },
            },
          };
        },
      };
    },
    // Webpack fix for Pyodide compatibility
    // This BannerPlugin adds a global __webpack_require__ stub to prevent runtime errors when Pyodide is loaded from CDN
    function (context, options) {
      return {
        name: "pyodide-webpack-fix",
        configureWebpack(config, isServer, utils) {
          if (isServer) return {};
          return {
            plugins: [
              new (require("webpack").BannerPlugin)({
                banner: `if (typeof __webpack_require__ === 'undefined') {
                  var __webpack_require__ = {};}`,
                raw: true,
                test: /\.js$/,
              }),
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/main.jpeg",

    // Open Graph metadata for social media sharing
    metadata: [
      { property: "og:title", content: "Physical AI & Humanoid Robotics" },
      {
        property: "og:description",
        content: "Learn to build intelligent robots with AI",
      },
      { property: "og:type", content: "website" },
      { name: "twitter:card", content: "summary_large_image" },
    ],

    colorMode: {
      respectPrefersColorScheme: true,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: "Physical AI & Robotics",
      hideOnScroll: false,
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
        },
        {
          type: "localeDropdown",
          position: "right",
        },
        {
          href: "https://github.com/Hamza123545/physical-ai-book",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Resources",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/Hamza123545/physical-ai-book",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
