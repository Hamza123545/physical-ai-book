@echo off
REM Windows Batch Build Script
REM Build script that conditionally adds --localstorage-file flag for Node.js 25+

REM Change to book-source directory
cd /d "%~dp0.."

REM Get Node version
for /f "tokens=1 delims=." %%a in ('node -v') do set NODE_VERSION=%%a
set NODE_VERSION=%NODE_VERSION:v=%

REM Check if Node.js 25+
if %NODE_VERSION% geq 25 (
    REM Node.js 25+ requires --localstorage-file flag
    set TEMP_FILE=%TEMP%\docusaurus-localstorage
    node --localstorage-file=%TEMP_FILE% ./node_modules/.bin/docusaurus build
) else (
    REM Node.js 20-24 doesn't need the flag
    node ./node_modules/.bin/docusaurus build
)

