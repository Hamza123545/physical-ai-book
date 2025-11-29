# PowerShell Build Script for Windows
# Build script that conditionally adds --localstorage-file flag for Node.js 25+

# Change to book-source directory (parent of scripts/)
$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Join-Path $scriptPath "..")

# Get Node version
$nodeVersionOutput = node -v
$nodeVersion = [int]($nodeVersionOutput -replace 'v(\d+)\..*', '$1')

if ($nodeVersion -ge 25) {
    # Node.js 25+ requires --localstorage-file flag
    # Use Windows temp path
    $tempPath = [System.IO.Path]::GetTempPath()
    $localStorageFile = Join-Path $tempPath "docusaurus-localstorage"
    node --localstorage-file=$localStorageFile ./node_modules/.bin/docusaurus build
} else {
    # Node.js 20-24 doesn't need the flag
    node ./node_modules/.bin/docusaurus build
}

