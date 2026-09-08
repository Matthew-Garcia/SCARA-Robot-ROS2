@echo off
setlocal
cd /d "%~dp0"
where git >nul 2>&1
if errorlevel 1 (
  echo Git is required. Install Git for Windows, then run this again.
  exit /b 1
)
where gh >nul 2>&1
if errorlevel 1 (
  echo GitHub CLI is required. Install it, then run gh auth login.
  exit /b 1
)
gh auth status >nul 2>&1
if errorlevel 1 (
  echo Sign in first with: gh auth login
  exit /b 1
)
set "SCARA_GITHUB_LOGIN="
for /f "delims=" %%I in ('gh api user --jq .login') do set "SCARA_GITHUB_LOGIN=%%I"
if /I not "%SCARA_GITHUB_LOGIN%"=="Matthew-Garcia" (
  echo The active GitHub account must be Matthew-Garcia.
  exit /b 1
)
gh repo view Matthew-Garcia/SCARA-Robot-ROS2 >nul 2>&1
if not errorlevel 1 (
  echo SCARA-Robot-ROS2 already exists. This script will not overwrite it.
  exit /b 1
)
set "SCARA_VISIBILITY=--private"
if /I "%~1"=="public" set "SCARA_VISIBILITY=--public"
if not exist .git (
  git init -b main
  if errorlevel 1 exit /b 1
)
git remote get-url origin >nul 2>&1
if not errorlevel 1 (
  echo This directory already has an origin remote. No repository was created.
  exit /b 1
)
git add --all
if errorlevel 1 exit /b 1
git diff --cached --quiet
if errorlevel 1 (
  git commit -m "Add organized SCARA Humble workspace and manipulation scene"
  if errorlevel 1 (
    echo Set your Git user.name and user.email if Git requested them, then rerun.
    exit /b 1
  )
)
gh repo create Matthew-Garcia/SCARA-Robot-ROS2 %SCARA_VISIBILITY% --description "Original SCARA CAD with ROS 2 Humble, ros2_control, MoveIt 2, Gazebo, RViz2, analytical kinematics and manipulation props." --source=. --remote=origin --push
if errorlevel 1 (
  echo Publishing failed. Read the GitHub CLI error above before retrying.
  exit /b 1
)
echo Created and uploaded SCARA-Robot-ROS2. Your original SCARA-Robot repo was not changed.
gh repo view Matthew-Garcia/SCARA-Robot-ROS2 --web
