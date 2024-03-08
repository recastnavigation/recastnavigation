# Set the path to the RecastCLI.exe executable
$RecastCLIPath = Join-Path $PWD "cmake-build-release\RecastCLI\RecastCLI.exe"
$RecastCLIDirectory = Split-Path -Path $RecastCLIPath -Parent
Set-Location -Path $RecastCLIDirectory

$sizes = @(0.1, 0.2, 0.3, 0.4, 0.5)
$meshes = @("Zelda", "Military", "City")
foreach ($size in $sizes)
{
    Write-Host "Grid Size: " $size
    foreach ($mesh in $meshes)
    {
        $args = "-f Meshes/$mesh.obj -lcmr CSV/minima-$Mesh.csv -o Data -cs $size -ar 0 -lcm"
        Start-Process -FilePath $RecastCLIPath -ArgumentList $args -NoNewWindow -Wait
    }
}