# --- KONFIGURATION ---

$ObjCopy  = "C:\ST\STM32CubeIDE_1.15.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406132123\tools\bin\arm-none-eabi-objcopy.exe"

# --- KONFIGURATION ---
$CubeProg = "C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
$SignTool = "C:\ST\STM32CubeProgrammer\bin\STM32_SigningTool_CLI.exe"
$ExtLoader = "C:\ST\STM32CubeProgrammer\bin\ExternalLoader\MX25UM51245G_STM32N6570-NUCLEO.stldr"

$ProjectName = "PibHand_NucleoN6"
$FsblBin     = "FSBL\Debug\$($ProjectName)_FSBL.bin"
$AppliBin    = "Appli\Debug\$($ProjectName)_Appli.bin"

# --- 1. SIGNIEREN ---
Write-Host "--- Signiere FSBL (SYSRAM) ---" -ForegroundColor Cyan
& $SignTool -bin $FsblBin -nk -of 0x80000000 -t fsbl -o "$($ProjectName)_FSBL-trusted.bin" -hv 2.3 -align

Write-Host "--- Signiere Appli (AXISRAM1) ---" -ForegroundColor Cyan
& $SignTool -bin $AppliBin -nk -of 0x34000000 -t fsbl -o "$($ProjectName)_Appli-trusted.bin" -hv 2.3 -align

# --- 2. FLASHEN ---
Write-Host "--- Starte Flash-Vorgang ---" -ForegroundColor Yellow
$FlashArgs = @("-c", "port=SWD", "mode=UR", "reset=HWrst", "-el", $ExtLoader)
$FlashArgs += @("-w", "$($ProjectName)_FSBL-trusted.bin", "0x70000000", "-v")
$FlashArgs += @("-w", "$($ProjectName)_Appli-trusted.bin", "0x70100000", "-v")
$FlashArgs += @("-rst")

& $CubeProg $FlashArgs

Write-Host "" 
Write-Host "--- Flashen und Verifizieren abgeschlossen ---" -ForegroundColor Green
Write-Host "Hinweis: Der abschließende MCU-Reset kann im Development Mode mit" -ForegroundColor Yellow
Write-Host "'Unable to run MCU' / Fehlercode 32 enden. Das bedeutet nicht, dass" -ForegroundColor Yellow
Write-Host "das Flashen fehlgeschlagen ist, sofern beide Downloads verifiziert wurden." -ForegroundColor Yellow
Write-Host "Für den Start aus dem externen Flash: BOOT1 auf 'Boot from flash' umstellen" -ForegroundColor Cyan
Write-Host "und danach den Reset-Taster drücken oder das Board neu einschalten." -ForegroundColor Cyan
