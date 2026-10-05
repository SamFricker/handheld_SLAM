$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

Write-Host "Handheld SLAM YouTube + GitHub publisher" -ForegroundColor Cyan

if (-not (Test-Path ".git")) {
    throw "Run this from the handheld_SLAM repository root."
}

$python = $null
if (Test-Path ".\.venv\Scripts\python.exe") {
    $python = ".\.venv\Scripts\python.exe"
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    $python = "python"
} elseif (Get-Command py -ErrorAction SilentlyContinue) {
    $python = "py"
} else {
    throw "Python was not found."
}

$secret = Get-ChildItem -Path . -File -Include "client_secret*.json", "client_secrets*.json" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $secret) {
    Write-Host ""
    Write-Host "You need a one-time YouTube OAuth credential before this can upload to your channel." -ForegroundColor Yellow
    Write-Host "1. In Google Cloud, enable YouTube Data API v3."
    Write-Host "2. Create an OAuth client ID with application type 'Desktop app'."
    Write-Host "3. Download the JSON into this folder (its default client_secret_....json name is fine)."
    Write-Host "4. Re-run this same command."
    throw "YouTube OAuth client JSON not found."
}

$requiredVideos = @(
    "Media\wireless-orientation-tracking.mov",
    "Media\complete-mapping.mp4",
    "Media\test.mov"
)
foreach ($video in $requiredVideos) {
    if (-not (Test-Path $video)) {
        throw "Missing required video: $video"
    }
}

Write-Host "Installing/updating the small publishing dependencies..." -ForegroundColor Cyan
& $python -m pip install --quiet --upgrade google-api-python-client google-auth-oauthlib google-auth-httplib2 opencv-python-headless pillow
if ($LASTEXITCODE -ne 0) { throw "Python dependency installation failed." }

Write-Host ""
Write-Host "Uploading the three large videos to YouTube..." -ForegroundColor Cyan
& $python ".\youtube_publish.py"
if ($LASTEXITCODE -ne 0) {
    throw "YouTube upload failed. Git has not been changed."
}

Write-Host ""
Write-Host "Removing the raw videos from Git tracking (not deleting your local copies)..." -ForegroundColor Cyan
foreach ($video in $requiredVideos) {
    git rm --cached --ignore-unmatch -- "$video"
}

# The JSON is useful locally but not necessary in the public repository.
if (-not (Select-String -Path ".gitignore" -Pattern '^youtube_links\.json$' -Quiet)) {
    Add-Content -Path ".gitignore" -Value "`nyoutube_links.json"
}

git add README.md .gitignore youtube_publish.py publish.ps1 Media/readme

Write-Host ""
Write-Host "Updating the existing initial commit..." -ForegroundColor Cyan
git commit --amend --no-edit
if ($LASTEXITCODE -ne 0) { throw "Could not amend the Git commit." }

Write-Host ""
Write-Host "Pushing the cleaned repository to GitHub..." -ForegroundColor Cyan
git push -u origin main
if ($LASTEXITCODE -ne 0) { throw "GitHub push failed." }

Write-Host ""
Write-Host "Done." -ForegroundColor Green
Write-Host "The videos remain on your computer, are now hosted on YouTube, and are no longer stored in Git."
Write-Host "README.md now contains clickable first-frame thumbnails linking to the uploaded videos."
