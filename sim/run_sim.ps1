# Compile and run the vis_* module testbenches with ModelSim SE-64.
# Usage:  powershell -ExecutionPolicy Bypass -File run_sim.ps1

$ErrorActionPreference = 'Continue'
$Msi = 'E:\modeltech64_10.6d\win64'
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Here

if (-not (Test-Path (Join-Path $Msi 'vsim.exe'))) {
    Write-Host "ModelSim not found at $Msi"
    exit 1
}

if (-not (Test-Path '.\work')) { & "$Msi\vlib.exe" work | Out-Null }

# Each testbench drives one leaf module on its own, so the unit under test and
# the testbench are always listed together. vis_top is deliberately excluded:
# it instantiates the whole chain and is verified on hardware, not here.
$rtl = @(
    '..\user_source\hdl_source\vis\vis_frame_stats.v',
    '..\user_source\hdl_source\vis\vis_otsu_thresh.v',
    '..\user_source\hdl_source\vis\vis_motion_grid.v',
    '..\user_source\hdl_source\vis\vis_binarize.v',
    '..\user_source\hdl_source\vis\vis_gauss3_96.v',
    '..\user_source\hdl_source\vis\vis_edge_96.v'
)
$tb = @(
    'tb_vis_frame_stats.v',
    'tb_vis_otsu_thresh.v',
    'tb_vis_motion_grid.v',
    'tb_vis_binarize.v',
    'tb_vis_edge.v'
)

& "$Msi\vlog.exe" $rtl $tb
if ($LASTEXITCODE -ne 0) { Write-Host 'COMPILE FAILED'; exit $LASTEXITCODE }

foreach ($top in @('tb_vis_frame_stats', 'tb_vis_otsu_thresh',
                   'tb_vis_motion_grid', 'tb_vis_binarize',
                   'tb_vis_edge')) {
    Write-Host ""
    Write-Host "=== $top ==="
    & "$Msi\vsim.exe" -c -do "run -all; quit -f" $top
}
