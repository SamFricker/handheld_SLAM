$ErrorActionPreference = "Stop"

Set-Location (Split-Path -Parent $MyInvocation.MyCommand.Path)

if (-not (Test-Path ".git")) {
    throw "Put these files in the root of your handheld_SLAM Git repository."
}

if (Test-Path ".\.venv\Scripts\python.exe") {
    $python = ".\.venv\Scripts\python.exe"
}
else {
    $python = "python"
}

& $python -c "import cv2" 2>$null

if ($LASTEXITCODE -ne 0) {
    Write-Host "Installing opencv-python..."
    & $python -m pip install opencv-python
}

& $python ".\prepare_media.py"

if ($LASTEXITCODE -ne 0) {
    throw "Media preparation failed."
}

Write-Host ""
Write-Host "Removing raw videos from Git tracking..."
Write-Host "The files themselves will stay on your computer."

git rm -r --cached --ignore-unmatch -- `
    "Media/*.mp4" `
    "Media/*.mov" `
    "Media/*.avi" `
    "Media/*.mkv"

git add .

$target = "https://github.com/SamFricker/handheld_SLAM.git"

$origin = git remote get-url origin 2>$null

if ($LASTEXITCODE -ne 0) {
    git remote add origin $target
}
elseif ($origin -ne $target) {
    git remote set-url origin $target
}

git branch -M main

git rev-parse --verify HEAD *> $null

if ($LASTEXITCODE -eq 0) {
    git commit --amend --no-edit
}
else {
    git commit -m "Initial commit: handheld SLAM prototype"
}

Write-Host ""
Write-Host "Pushing to GitHub..."

git push -u origin main

if ($LASTEXITCODE -ne 0) {
    throw "Git push failed. Read the Git error above before retrying."
}

Write-Host ""
Write-Host "Done."
Write-Host "https://github.com/SamFricker/handheld_SLAM"