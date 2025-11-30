/**
 * Automatic Urdu Translation Script
 *
 * Uses FREE Gemini 2.5 Flash to translate all docs from English to Urdu.
 * Preserves markdown formatting, frontmatter, and code blocks.
 *
 * Cost: $0 (uses Gemini free tier - 2M tokens/month)
 */

import { GoogleGenerativeAI } from '@google/generative-ai';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Initialize Gemini AI
const GEMINI_API_KEY = process.env.GEMINI_API_KEY;
if (!GEMINI_API_KEY) {
  console.error('❌ GEMINI_API_KEY not found in environment variables');
  console.error('Please add GEMINI_API_KEY to your .env file');
  process.exit(1);
}

const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);
// Use stable gemini-1.5-flash with better rate limits (60 requests/min)
const model = genAI.getGenerativeModel({
  model: 'gemini-1.5-flash',
  generationConfig: {
    temperature: 0.3,  // Lower temperature for more consistent translations
    maxOutputTokens: 8192,
  }
});

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const URDU_DIR = path.join(__dirname, '..', 'i18n', 'ur', 'docusaurus-plugin-content-docs', 'current');

// Translation statistics
let stats = {
  total: 0,
  translated: 0,
  skipped: 0,
  errors: 0,
  tokensUsed: 0
};

/**
 * Translate markdown content to Urdu using Gemini
 */
async function translateToUrdu(content, filePath) {
  const prompt = `You are a professional translator specializing in technical documentation translation from English to Urdu.

IMPORTANT INSTRUCTIONS:
1. Translate the content below from English to Urdu
2. Preserve ALL markdown formatting (headers, lists, code blocks, links, etc.)
3. Do NOT translate:
   - Code blocks (anything between \`\`\`)
   - URLs and file paths
   - Variable names, function names, class names
   - YAML frontmatter (keep it in English)
   - Technical terms that are commonly used in English (e.g., "AI", "API", "Python", "ROS", etc.)
4. Use formal, technical Urdu suitable for educational content
5. Preserve the exact same structure and formatting
6. Return ONLY the translated markdown, no explanations

CONTENT TO TRANSLATE:
${content}`;

  try {
    const result = await model.generateContent(prompt);
    const response = await result.response;
    const translatedText = response.text();

    // Track token usage (approximate)
    stats.tokensUsed += (content.length / 4) + (translatedText.length / 4);

    return translatedText;
  } catch (error) {
    console.error(`❌ Translation error for ${filePath}:`, error.message);
    stats.errors++;
    return null;
  }
}

/**
 * Get all markdown files recursively
 */
function getAllMarkdownFiles(dir, fileList = []) {
  const files = fs.readdirSync(dir);

  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      getAllMarkdownFiles(filePath, fileList);
    } else if (file.endsWith('.md')) {
      fileList.push(filePath);
    }
  });

  return fileList;
}

/**
 * Translate a single file
 */
async function translateFile(sourceFile) {
  const relativePath = path.relative(DOCS_DIR, sourceFile);
  const targetFile = path.join(URDU_DIR, relativePath);

  console.log(`\n📄 Translating: ${relativePath}`);

  // Check if already translated (skip to save API calls)
  if (fs.existsSync(targetFile)) {
    const targetContent = fs.readFileSync(targetFile, 'utf-8');
    // Simple heuristic: if file contains Urdu characters, assume it's translated
    if (/[\u0600-\u06FF]/.test(targetContent)) {
      console.log('   ⏭️  Already translated (contains Urdu characters)');
      stats.skipped++;
      return;
    }
  }

  // Read source file
  const content = fs.readFileSync(sourceFile, 'utf-8');

  // Translate
  const translatedContent = await translateToUrdu(content, relativePath);

  if (!translatedContent) {
    console.log('   ⚠️  Translation failed, keeping English version');
    return;
  }

  // Ensure target directory exists
  const targetDir = path.dirname(targetFile);
  if (!fs.existsSync(targetDir)) {
    fs.mkdirSync(targetDir, { recursive: true });
  }

  // Write translated file
  fs.writeFileSync(targetFile, translatedContent, 'utf-8');
  console.log('   ✅ Translated successfully');
  stats.translated++;

  // Rate limiting: wait 1.5 seconds between requests to stay under 60/min limit
  // Gemini 1.5 Flash free tier: 15 RPM (requests per minute)
  await new Promise(resolve => setTimeout(resolve, 4500));
}

/**
 * Main translation function
 */
async function translateAllDocs() {
  console.log('🌍 Starting Urdu Translation using FREE Gemini 2.5 Flash\n');
  console.log(`Source: ${DOCS_DIR}`);
  console.log(`Target: ${URDU_DIR}\n`);

  // Get all markdown files
  const markdownFiles = getAllMarkdownFiles(DOCS_DIR);
  stats.total = markdownFiles.length;

  console.log(`Found ${stats.total} markdown files to translate\n`);
  console.log('=' .repeat(60));

  // Translate each file
  for (const file of markdownFiles) {
    await translateFile(file);
  }

  // Print summary
  console.log('\n' + '='.repeat(60));
  console.log('\n📊 Translation Summary:');
  console.log(`   Total files: ${stats.total}`);
  console.log(`   ✅ Translated: ${stats.translated}`);
  console.log(`   ⏭️  Skipped: ${stats.skipped}`);
  console.log(`   ❌ Errors: ${stats.errors}`);
  console.log(`   📈 Approximate tokens used: ${Math.round(stats.tokensUsed)}`);
  console.log(`   💰 Cost: $0 (FREE with Gemini)`);
  console.log('\n✨ Translation complete!\n');
}

// Run translation
translateAllDocs().catch(console.error);
