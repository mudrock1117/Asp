# Run in PowerShell after extracting this package, with Git for Windows installed.
# Log in to GitHub (Git Credential Manager or gh auth login) before pushing.
$ErrorActionPreference = 'Stop'
$repo = 'https://github.com/mudrock1117/Asp.git'
$branch = 'main'
$work = Join-Path $env:TEMP ('asp-push-' + [guid]::NewGuid().ToString('N'))
git clone --branch $branch $repo $work
if ($LASTEXITCODE -ne 0) { throw 'git clone failed' }
$source = Split-Path -Parent $MyInvocation.MyCommand.Path
robocopy $source $work /E /XD .git build install log __pycache__ .idea /XF *.pyc *.pyo *.log *.tmp | Out-Null
if ($LASTEXITCODE -gt 7) { throw 'robocopy failed' }
Push-Location $work
try {
  git add .
  git status --short
  git diff --cached --quiet
  if ($LASTEXITCODE -eq 0) { Write-Host 'No changes to commit'; exit 0 }
  git commit -m 'Import ESTUN S3-60 ROS 2 sources and YOLOv5 OBB detector'
  if ($LASTEXITCODE -ne 0) { throw 'git commit failed' }
  git push origin $branch
  if ($LASTEXITCODE -ne 0) { throw 'git push failed' }
  Write-Host 'Upload succeeded: https://github.com/mudrock1117/Asp'
} finally { Pop-Location }
