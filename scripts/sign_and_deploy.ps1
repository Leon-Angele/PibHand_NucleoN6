# --- CONFIGURATION ---
$ObjCopy  = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe"
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

# --- 2. GEWICHTE: .raw → .bin → .hex (Adresse 0x71000000, xSPI2) ---
Write-Host "--- Konvertiere Modell-Gewichte ---" -ForegroundColor Cyan
$WeightsBin = "$($ProjectName)_weights.xSPI2.bin"
$WeightsHex = "$($ProjectName)_weights.xSPI2.hex"

Copy-Item "blockernn_atonbuf.xSPI2.raw" $WeightsBin
& $ObjCopy -I binary $WeightsBin --change-addresses 0x71000000 -O ihex $WeightsHex

# --- 3. FLASHEN ---
Write-Host "--- Starte Flash-Vorgang ---" -ForegroundColor Yellow

# Basis-Argumente mit korrigiertem Reset-Modus (HWrst statt HwReset)
$CommonArgs = @("-c", "port=SWD", "mode=UR", "reset=HWrst", "-el", $ExtLoader)

# Durchgang 1: Flashen der Binaries (xSPI1)
Write-Host "-> Schreibe FSBL und Appli..." -ForegroundColor White
$FlashArgs1 = $CommonArgs + @(
    "-w", "$($ProjectName)_FSBL-trusted.bin", "0x70000000", "-v",
    "-w", "$($ProjectName)_Appli-trusted.bin", "0x70100000", "-v"
)
& $CubeProg $FlashArgs1

# Durchgang 2: Flashen der Gewichte (xSPI2 via HEX)
# Getrennt, damit das Adress-Parsing der HEX-Datei in der CLI nicht mit den Binär-Offsets kollidiert
Write-Host "-> Schreibe Modell-Gewichte (HEX)..." -ForegroundColor White
$FlashArgs2 = $CommonArgs + @(
    "-w", $WeightsHex, "-v"
)
& $CubeProg $FlashArgs2

Write-Host "--- Fertig! Bitte drücke jetzt den Reset-Taster (Schwarz/Blau) am Board. ---" -ForegroundColor Green