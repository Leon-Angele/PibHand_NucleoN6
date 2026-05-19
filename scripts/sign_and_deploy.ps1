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
$FlashArgs = @("-c", "port=SWD", "mode=UR", "reset=HwReset", "-el", $ExtLoader)
$FlashArgs += @("-w", "$($ProjectName)_FSBL-trusted.bin", "0x70000000", "-v")
$FlashArgs += @("-w", "$($ProjectName)_Appli-trusted.bin", "0x70100000", "-v")
$FlashArgs += @("-rst")

& $CubeProg $FlashArgs

Write-Host "--- Fertig! Drücke ggf. Reset am Board. ---" -ForegroundColor Green