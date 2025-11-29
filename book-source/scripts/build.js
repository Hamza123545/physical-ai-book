// Cross-platform build script (Node.js)
// Works on Windows, Mac, and Linux

const { execSync } = require('child_process');
const path = require('path');
const os = require('os');

// Change to book-source directory
process.chdir(path.join(__dirname, '..'));

// Get Node version
const nodeVersionOutput = execSync('node -v', { encoding: 'utf-8' });
const nodeVersionMatch = nodeVersionOutput.match(/v(\d+)/);
const nodeVersion = nodeVersionMatch ? parseInt(nodeVersionMatch[1]) : 20;

// Build command
let buildCommand;

if (nodeVersion >= 25) {
  // Node.js 25+ requires --localstorage-file flag
  const tempPath = os.tmpdir();
  const localStorageFile = path.join(tempPath, 'docusaurus-localstorage');
  buildCommand = `node --localstorage-file=${localStorageFile} ./node_modules/.bin/docusaurus build`;
} else {
  // Node.js 20-24 doesn't need the flag
  buildCommand = 'node ./node_modules/.bin/docusaurus build';
}

// Execute build
console.log(`Building with Node.js ${nodeVersion}...`);
execSync(buildCommand, { stdio: 'inherit' });

