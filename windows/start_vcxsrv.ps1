# Start the VcXsrv X server if it isn't already running.
# Needed because WSLg's built-in GUI compositor does not render on this machine.
$exe = 'C:\Program Files\VcXsrv\vcxsrv.exe'
if (Get-Process vcxsrv -ErrorAction SilentlyContinue) {
    Write-Output "vcxsrv already running."
} else {
    Start-Process -FilePath $exe -ArgumentList ':0','-multiwindow','-clipboard','-ac','-silent-dup-error'
    Start-Sleep -Seconds 2
    if (Get-Process vcxsrv -ErrorAction SilentlyContinue) {
        Write-Output "vcxsrv started, listening on TCP 6000."
    } else {
        Write-Output "ERROR: vcxsrv failed to start."
    }
}
