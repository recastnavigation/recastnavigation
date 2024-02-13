$appDir = $args[0]

$meshes = @("Zelda", "Military", "City")

Write-Host $appDir
foreach ($mesh in $meshes) {
    Write-Host "Mesh name: $mesh"
    Start-Process -FilePath $appDir -ArgumentList "-f", "Meshes\$mesh.obj", "-lcmr", "CSV\minima-$mesh.csv", "-o", "Data", "-cs", "0.1", "-ar", "0", "-lcm" -PassThru

    $processInfo = New-Object System.Diagnostics.ProcessStartInfo
    $processInfo.FileName = $appDir
    $processInfo.Arguments = "-f", "Meshes\$mesh.obj", "-lcmr", "CSV\minima-$mesh.csv", "-o", "Data", "-cs", "0.1", "-ar", "0", "-lcm"
    $processInfo.RedirectStandardError = $true
    $processInfo.RedirectStandardOutput = $true
    $processInfo.UseShellExecute = $false

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $processInfo
    $process.Start() | Out-Null

    $process.WaitForExit()
    $stdout = $process.StandardOutput.ReadToEnd()
    $stderr = $process.StandardError.ReadToEnd()
    Write-Host "stdout: $stdout"
    Write-Host "stderr: $stderr"
    Write-Host "exit code: " + $p.ExitCode
}