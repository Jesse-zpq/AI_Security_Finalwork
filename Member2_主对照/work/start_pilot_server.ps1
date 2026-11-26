# Usage from the project root:
# .\work\start_pilot_server.ps1 -CarlaRoot D:\CARLA\Carla-1.0-Win64-Shipping -Suite formal -Output outputs\CARLA_formal_results
# Pass -Python and optionally -PythonClientPath for an explicitly installed client.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$CarlaRoot,
    [string]$Python = 'python',
    [string]$PythonClientPath,
    [ValidateSet('pilot','qualification','formal')][string]$Suite = 'pilot',
    [ValidateSet('offscreen','null')][string]$Renderer = 'null',
    [ValidateRange(1024,65534)][int]$Port = 2200,
    [string]$Output = 'outputs\CARLA_run',
    [string[]]$Cases,
    [switch]$Probe
)

$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$root = (Resolve-Path -LiteralPath $CarlaRoot).Path
$game = Join-Path $root 'CarlaUnreal\Binaries\Win64\CarlaUnreal-Win64-Shipping.exe'
if (-not (Test-Path -LiteralPath $game -PathType Leaf)) {
    throw "CARLA 1.0 Windows server executable missing at $game"
}
$pythonCommand = Get-Command -Name $Python -CommandType Application -ErrorAction Stop | Select-Object -First 1
$pythonExe = $pythonCommand.Source
$outputRoot = if ([IO.Path]::IsPathRooted($Output)) { $Output } else { Join-Path $project $Output }
$serverName = 'server_{0}_{1}_{2}' -f $Renderer, $Port, (Get-Date -Format 'yyyyMMdd_HHmmss_fff')
$base = (New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot $serverName)).FullName
$stdout = Join-Path $base 'stdout.txt'
$stderr = Join-Path $base 'stderr.txt'
$options = @("-carla-rpc-port=$Port", ('-carla-streaming-port={0}' -f ($Port + 1)),
             '-nosound', '-unattended', '-stdout', '-FullStdOutLogOutput', ('-UserDir="{0}"' -f $base))
if ($Renderer -eq 'null') { $options += '-nullrhi' }
else { $options += '-RenderOffScreen','-quality-level=Low' }

$oldPythonPath = $env:PYTHONPATH
$server = $null
$exitCode = 1
try {
    if ($PythonClientPath) {
        $clientFolder = (Resolve-Path -LiteralPath $PythonClientPath).Path
        $env:PYTHONPATH = if ($oldPythonPath) { "$clientFolder;$oldPythonPath" } else { $clientFolder }
    }
    # Verify the matching wheel and make sure the experiment will use its own port.
    & $pythonExe -c 'import carla'
    if ($LASTEXITCODE -ne 0) { throw 'Install the CARLA 1.0 Python wheel for this Python interpreter.' }
    if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
        throw "Port $Port is already in use. Select a separate experiment port."
    }
    $server = Start-Process -FilePath $game -WorkingDirectory (Split-Path -Parent $game) `
        -ArgumentList $options -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $ready = $false
    for ($attempt = 0; $attempt -lt 22; $attempt++) {
        if ($server.HasExited) { throw "CARLA stopped before opening RPC. See $stdout and $stderr" }
        & $pythonExe -c "import socket; socket.create_connection(('127.0.0.1',$Port),timeout=0.7).close()" 2>$null
        if ($LASTEXITCODE -eq 0) { $ready = $true; break }
        Start-Sleep -Seconds 2
    }
    if (-not $ready) { throw "CARLA did not open port $Port. See $stdout and $stderr" }
    if ($Probe) {
        & $pythonExe -B (Join-Path $project 'outputs\carla_preflight.py') --geometry --port $Port
    } else {
        $arguments = @('--mode','carla','--port',$Port,'--suite',$Suite,'--output',$outputRoot)
        if ($Cases) { $arguments += '--cases'; $arguments += $Cases }
        & $pythonExe -B (Join-Path $project 'outputs\carla_pilot.py') @arguments
    }
    $exitCode = $LASTEXITCODE
} finally {
    if ($null -ne $server -and -not $server.HasExited) { Stop-Process -Id $server.Id }
    $env:PYTHONPATH = $oldPythonPath
}
exit $exitCode
