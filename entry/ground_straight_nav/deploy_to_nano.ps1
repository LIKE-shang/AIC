# Upload ground_straight_nav to the Nano and verify it.
#
# ASCII-only on purpose: Windows PowerShell 5.1 reads a BOM-less script as ANSI,
# which corrupts non-ASCII text and breaks quoting. Keep this file ASCII.
#
# Usage (Windows PowerShell, from the workspace root):
#   powershell -ExecutionPolicy Bypass -File .\ground_straight_nav\deploy_to_nano.ps1
#
# Optional:
#   -Target abot@192.168.0.100
#   -Key    C:\Users\jiangzx\.ssh\id_rsa_abot

param(
    [string]$Target = 'abot@192.168.0.100',
    [string]$Key = (Join-Path $env:USERPROFILE '.ssh\id_rsa_abot'),
    [string]$Package = 'ground_straight_nav_20261018_v3.tar.gz'
)

$ErrorActionPreference = 'Stop'

# Make sure whatever the remote side prints survives the trip to the log file.
try {
    [Console]::OutputEncoding = [System.Text.Encoding]::UTF8
    $OutputEncoding = [System.Text.Encoding]::UTF8
} catch { }

$repo = Split-Path -Parent $PSScriptRoot
$deployDir = Join-Path $repo '_deploy'
$tarPath = Join-Path $deployDir $Package
$shaPath = "$tarPath.sha256"
$runner = Join-Path $PSScriptRoot 'tools\verify_ground_nav.sh'

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$diagDir = Join-Path $repo '_diagnostics'
$logPath = Join-Path $diagDir ("ground_nav_deploy_{0}.log" -f $stamp)

foreach ($p in @($tarPath, $shaPath, $runner, $Key)) {
    if (-not (Test-Path -LiteralPath $p)) {
        throw "Missing file: $p"
    }
}
if (-not (Test-Path -LiteralPath $diagDir)) {
    New-Item -ItemType Directory -Path $diagDir | Out-Null
}

Write-Host '== 1. connection probe ==' -ForegroundColor Cyan
& ssh -i $Key -o BatchMode=yes -o ConnectTimeout=8 $Target 'hostname; date; uname -r'
if ($LASTEXITCODE -ne 0) {
    throw "SSH failed (exit $LASTEXITCODE)"
}

Write-Host ''
Write-Host '== 2. upload ==' -ForegroundColor Cyan
& scp -i $Key -o BatchMode=yes $tarPath $shaPath $runner "${Target}:/home/abot/"
if ($LASTEXITCODE -ne 0) {
    throw "scp failed (exit $LASTEXITCODE)"
}
Write-Host "uploaded $Package, its .sha256, and verify_ground_nav.sh"

Write-Host ''
Write-Host '== 3. remote install and verify ==' -ForegroundColor Cyan
$remote = & ssh -i $Key -o BatchMode=yes $Target 'bash /home/abot/verify_ground_nav.sh' 2>&1
$rc = $LASTEXITCODE
$remote | ForEach-Object { Write-Host $_ }
# UTF-8 without BOM so the log stays readable in any editor.
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllLines($logPath, [string[]]$remote, $utf8NoBom)

Write-Host ''
Write-Host '== 4. verdict ==' -ForegroundColor Cyan
$text = ($remote | Out-String)
if ($text -match 'RESULT: PASSED') {
    Write-Host 'PASSED - install, tests, map check and read-only preflight all green' -ForegroundColor Green
} else {
    Write-Host 'NOT PASSED - see the [FAIL] lines below' -ForegroundColor Yellow
    $remote | Where-Object { $_ -match '\[FAIL\]|RESULT:' } | ForEach-Object { Write-Host $_ }
}
Write-Host "local log : $logPath"
Write-Host 'remote log: /home/abot/ground_nav_verify_*.log'
exit $rc
