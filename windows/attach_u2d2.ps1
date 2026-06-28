# Robust U2D2 attach: auto-attach mode re-binds the device whenever WSL is up
# and re-attaches automatically after an unplug/VM restart.
# Does NOT require admin (the one-time 'bind' already ran in 03_forward_u2d2.ps1).
# Keep this window open, and keep an Ubuntu terminal open, while using the motor.
$usbipd = 'C:\Program Files\usbipd-win\usbipd.exe'
$line = (& $usbipd list) | Select-String '0403:6014'
if (-not $line) { Write-Error "U2D2 (0403:6014) not found. Plugged in?"; exit 1 }
$busid = ($line.ToString().Trim() -split '\s+')[0]
Write-Host "Auto-attaching U2D2 at busid $busid to Ubuntu-22.04 (Ctrl+C to stop)..."
& $usbipd attach -a --busid $busid --wsl Ubuntu-22.04
